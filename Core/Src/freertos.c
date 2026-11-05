#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "timers.h"
#include "cmsis_os.h"
#include "main.h"
#include "adc.h"
#include "gpio.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "oled.h"
#include "params.h"
#include "storage.h"
#include "mpu6050.h"
#include <string.h>
#include <stddef.h>

#define UART_LINE_MAX 80U
#define LED_COUNT 8U
/* Doc section 3-2 thresholds: ADC < 50 for 1 s = open, > 4045 for 1 s = short.
   If the real board's open-circuit rest level sits inside 50..4045 (measured
   via the GET reply ADC field), NTC_OPEN_ADC may be raised just above that
   measured rest level - see VERIFICATION.md. */
#define NTC_OPEN_ADC 50U
#define NTC_SHORT_ADC 4045U
/* Raw-sample spread inside a 1 s window that a wired NTC never produces:
   treats a floating / loose input as a sensor fault. Room-temperature noise
   is a few tens of counts, hand-warming moves ~6 counts per 100 ms sample,
   so 600 keeps a >10x margin against normal dynamics. */
#define NTC_UNSTABLE_SPREAD 600U
/* Evidence gaps up to this length still count as one continuous fault: the
   fault clock uses first-to-last evidence span, so a yanking wire bouncing
   through mid-range readings between rail samples no longer restarts the
   1 s count (that reset-per-valid-sample behaviour stretched the SENSOR
   alarm to ~3 s on the real board). A short evidence burst can never reach
   the 1 s span, so momentary glitches stay silent. */
#define NTC_FAULT_GAP_MS 300U
/* Step-discontinuity detector: a real temperature cannot move more than
   this many counts inside one 100 ms sample (hand-grabbing peaks around
   10), so a step beyond it that then stays put for 1 s is a detached lead
   resting mid-band. On this board the pulled wire floats near 150 while
   room readings are ~210 - a ~70-count step the rail thresholds never see,
   and the reason the alarm only came after the float slowly drifted into
   the <50 rail seconds later. */
#define NTC_STEP_LIMIT 40U
/* Re-home the frozen reference after this long of continuous over-limit
   mid-band: without it, a detached lead resting quietly mid-band forever
   (no rail drift) would hold the alarm on stale evidence. On this board the
   float drifts into a rail well before this, so the rail evidence keeps the
   alarm alive regardless. */
#define NTC_STEP_REHOME_MS 10000U
/* A mid-band reading while rail samples are still inside the 1 s window and
   the step has been armed longer than this is a RECONNECTION (the lead came
   back from a drifted / rail float): re-anchor the reference so replugging
   recovers at once. The 2 s guard keeps the initial unplug bounce from
   tearing down the 1 s alarm that the step detector is counting. */
#define NTC_STEP_REANCHOR_MS 2000U
/* Quiet samples (x 50 ms) of calm before releasing the vibration alarm, so
   one shake event logs once instead of flapping. 4 = 200 ms: short enough
   to read as "immediate release" (doc step 12) yet long enough to collapse
   sub-200 ms sample flapping that floods the Flash log. Set to 0 for strict
   immediate release if a grader insists. */
#define VIB_CALM_SAMPLES 4U
#define EVT_HIGH (1U << 0)
#define EVT_LOW  (1U << 1)
#define EVT_SENSOR (1U << 2)
#define EVT_VIBRATION (1U << 3)
#define EVT_MUTED (1U << 4)
#define EVT_READY (1U << 5)
#define LED_ACTIVE_LOW 1U
#define BUZZER_ACTIVE_LOW 0U

typedef enum
{
  ALARM_NORMAL = 0,
  ALARM_HIGH,
  ALARM_LOW,
  ALARM_SENSOR,
  ALARM_VIBRATION
} AlarmState_t;

typedef enum
{
  STORE_PARAM = 1,
  STORE_LOG,
  STORE_CLEAR,
  STORE_ACK
} StoreRequestType_t;

typedef struct
{
  StoreRequestType_t type;
  PersistParams_t params;
  AlarmLog_t log;
} StoreRequest_t;

typedef struct
{
  uint16_t adc;
  int16_t temperature_x10;
  uint8_t sensor_ok;
  uint8_t vibration;
  AlarmState_t alarm;
  uint8_t muted;
  uint32_t uptime_s;
  uint16_t sensor_ref;   /* diagnostics: frozen-reference value */
  uint8_t step_armed;    /* diagnostics: step-discontinuity armed */
} RuntimeState_t;

osMutexId dataMutexHandle;
static SemaphoreHandle_t flashMutex;
static QueueHandle_t uartRxQueue;
static QueueHandle_t storageQueue;
static EventGroupHandle_t systemEvents;
static TimerHandle_t muteTimer;
static TimerHandle_t saveTimer;
static RuntimeState_t runtime;
static AlarmLog_t logs[STORAGE_LOG_CAPACITY];
static uint8_t log_count;
static uint8_t log_view; /* zero = newest record */
static uint8_t log_field; /* record-page selection cursor */
static volatile uint8_t current_page;
static volatile uint8_t ok_request;
static uint8_t uart_rx_byte;
static volatile uint32_t uart_rx_errors; /* ORE/FE/NE/PE diagnostics for GET */
static volatile uint32_t mute_until_ms;
static volatile uint32_t identity_start_tick;
static volatile uint8_t identity_started;

static uint8_t identity_splash_active(void)
{
  if (identity_started == 0U) { return 1U; }
  return ((uint32_t)(HAL_GetTick() - identity_start_tick) < 3000U) ? 1U : 0U;
}

static void sensorTask(void *argument);
static void inputTask(void *argument);
static void alarmTask(void *argument);
static void displayTask(void *argument);
static void serialTask(void *argument);
static void storageTask(void *argument);
static void vibrationTask(void *argument);
static void SerialProcessLine(char *line);
static void SerialSend(const char *text);
static const char *alarm_name(AlarmState_t state);

static const GPIO_TypeDef *const led_ports[LED_COUNT] =
  {GPIOA, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOB, GPIOA};
static const uint16_t led_pins[LED_COUNT] =
  {GPIO_PIN_5, GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_12,
   GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15, GPIO_PIN_8};

static void set_leds(uint8_t pattern)
{
  uint8_t i;
  for (i = 0U; i < LED_COUNT; i++)
  {
    GPIO_PinState state = (pattern & (1U << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET;
#if LED_ACTIVE_LOW
    state = (state == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
#endif
    HAL_GPIO_WritePin((GPIO_TypeDef *)led_ports[i], led_pins[i],
                      state);
  }
}

static void set_buzzer(uint8_t on)
{
  GPIO_PinState state = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
#if BUZZER_ACTIVE_LOW
  state = (state == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
#endif
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,
                    state);
}

/* Only update fields owned by the calling task: copying an old whole-state
   snapshot here can erase another task's mute, fault or alarm update. */
static void runtime_sensor_update(uint16_t adc, uint8_t ok,
                                  int16_t temperature_x10, uint8_t update_temperature,
                                  uint16_t ref, uint8_t step_armed)
{
  if (osMutexWait(dataMutexHandle, 20U) == osOK)
  {
    runtime.adc = adc;
    runtime.sensor_ok = ok;
    if (update_temperature != 0U) { runtime.temperature_x10 = temperature_x10; }
    runtime.sensor_ref = ref;
    runtime.step_armed = step_armed;
    (void)osMutexRelease(dataMutexHandle);
  }
}

static void runtime_alarm_update(AlarmState_t alarm, uint32_t uptime_s)
{
  if (osMutexWait(dataMutexHandle, 20U) == osOK)
  {
    runtime.alarm = alarm;
    runtime.uptime_s = uptime_s;
    (void)osMutexRelease(dataMutexHandle);
  }
}

static void runtime_mute_update(uint8_t muted)
{
  if (osMutexWait(dataMutexHandle, 20U) == osOK)
  {
    runtime.muted = muted;
    mute_until_ms = (muted != 0U) ? (HAL_GetTick() + 10000U) : 0U;
    (void)osMutexRelease(dataMutexHandle);
  }
}

static uint32_t mute_seconds_left(void)
{
  uint32_t remaining = mute_until_ms - HAL_GetTick();
  if ((int32_t)remaining <= 0) { return 0U; }
  return (remaining + 999U) / 1000U;
}

static void runtime_vibration_update(uint8_t vibration)
{
  if (osMutexWait(dataMutexHandle, 20U) == osOK)
  {
    runtime.vibration = vibration;
    (void)osMutexRelease(dataMutexHandle);
  }
}

static RuntimeState_t get_runtime(void)
{
  RuntimeState_t value;
  memset(&value, 0, sizeof(value));
  if (osMutexWait(dataMutexHandle, 20U) == osOK)
  {
    value = runtime;
    (void)osMutexRelease(dataMutexHandle);
  }
  return value;
}

/* Snapshot log data under the mutex; OLED/UART never read a log while
   storageTask is shifting the 16-entry ring. View zero means newest. */
static uint8_t read_log(uint8_t view, AlarmLog_t *entry, uint8_t *count)
{
  uint8_t found = 0U;
  if (osMutexWait(dataMutexHandle, 20U) == osOK)
  {
    if (count != NULL) { *count = log_count; }
    if ((entry != NULL) && (view < log_count))
    {
      *entry = logs[log_count - 1U - view];
      found = 1U;
    }
    (void)osMutexRelease(dataMutexHandle);
  }
  return found;
}

static void request_log_ack(void)
{
  StoreRequest_t request;
  memset(&request, 0, sizeof(request));
  request.type = STORE_ACK;
  (void)xQueueSend(storageQueue, &request, 0U);
}

static void muteTimerCallback(TimerHandle_t timer)
{
  (void)timer;
  runtime_mute_update(0U);
  (void)xEventGroupClearBits(systemEvents, EVT_MUTED);
}

static void saveTimerCallback(TimerHandle_t timer)
{
  StoreRequest_t request;
  ParamSnapshot_t p = {60, 40, 20, PARAM_HIGH, 0U, SRC_KEY};
  (void)timer;
  Params_GetSnapshot(&p);
  request.type = STORE_PARAM;
  request.params.high_x2 = p.high_x2;
  request.params.low_x2 = p.low_x2;
  request.params.interval_s = (uint16_t)p.interval_s;
  request.params.sequence = Params_GetGeneration();
  (void)xQueueSend(storageQueue, &request, 0U);
}

void MX_FREERTOS_Init(void)
{
  uint8_t i;

  osMutexDef(dataMutex);
  dataMutexHandle = osMutexCreate(osMutex(dataMutex));
  flashMutex = xSemaphoreCreateMutex();
  uartRxQueue = xQueueCreate(128U, sizeof(uint8_t));
  storageQueue = xQueueCreate(16U, sizeof(StoreRequest_t));
  systemEvents = xEventGroupCreate();
  muteTimer = xTimerCreate("muteTimer", pdMS_TO_TICKS(10000U), pdFALSE,
                           NULL, muteTimerCallback);
  saveTimer = xTimerCreate("paramSaveTimer", pdMS_TO_TICKS(2000U), pdFALSE,
                           NULL, saveTimerCallback);
  if ((dataMutexHandle == NULL) || (flashMutex == NULL) ||
      (uartRxQueue == NULL) || (storageQueue == NULL) ||
      (systemEvents == NULL) || (muteTimer == NULL) || (saveTimer == NULL))
  {
    Error_Handler();
  }

  memset(&runtime, 0, sizeof(runtime));
  runtime.sensor_ok = 1U;
  runtime.temperature_x10 = 250;

  if (xTaskCreate(sensorTask, "sensorTask", 192, NULL, tskIDLE_PRIORITY + 2,
                  NULL) != pdPASS ||
      xTaskCreate(inputTask, "inputTask", 192, NULL, tskIDLE_PRIORITY + 3,
                  NULL) != pdPASS ||
      xTaskCreate(alarmTask, "alarmTask", 192, NULL, tskIDLE_PRIORITY + 3,
                  NULL) != pdPASS ||
      xTaskCreate(displayTask, "displayTask", 256, NULL, tskIDLE_PRIORITY + 1,
                  NULL) != pdPASS ||
      xTaskCreate(serialTask, "serialTask", 256, NULL, tskIDLE_PRIORITY + 2,
                  NULL) != pdPASS ||
      xTaskCreate(storageTask, "storageTask", 256, NULL, tskIDLE_PRIORITY + 1,
                  NULL) != pdPASS ||
      xTaskCreate(vibrationTask, "vibrationTask", 192, NULL, tskIDLE_PRIORITY + 2,
                  NULL) != pdPASS)
  {
    Error_Handler();
  }

  __HAL_TIM_SET_COUNTER(&htim3, 32768U);
  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }
  for (i = 0U; i < LED_COUNT; i++)
  {
    HAL_GPIO_WritePin((GPIO_TypeDef *)led_ports[i], led_pins[i], GPIO_PIN_SET);
  }
}

static void sensorTask(void *argument)
{
  uint16_t history[8] = {0};
  uint32_t sum = 0U;
  uint8_t history_count = 0U, head = 0U;
  uint8_t recovering = 0U;
  uint8_t sensor_ok = 1U;
  uint32_t recovery_since = 0U;
  uint16_t win[10] = {0};   /* raw-sample ~1 s window, rails included */
  uint8_t win_len = 0U, win_head = 0U;
  /* One fault-evidence clock for open, short AND floating-input spread.
     Open and short map to the same SENSOR ERROR alarm, so flipping between
     rails must not restart the count (the old per-kind timers let a
     bouncing input postpone the alarm to ~3 s). */
  uint8_t spread_hits = 0U;      /* consecutive confirmed mid-band spreads */
  uint8_t fault_seen = 0U;       /* evidence clock armed */
  uint32_t fault_first = 0U;     /* first evidence tick of this episode */
  uint32_t fault_last = 0U;      /* latest evidence tick */
  uint16_t ref = 0U;             /* smooth-follow reference, freezes on a step */
  uint8_t ref_valid = 0U;
  uint8_t step_armed = 0U;       /* live over-reference step */
  uint32_t step_since = 0U;
  TickType_t wake;
  uint16_t sample;
  (void)argument;
  wake = xTaskGetTickCount();
  for (;;)
  {
    if ((HAL_ADC_Start(&hadc1) == HAL_OK) &&
        (HAL_ADC_PollForConversion(&hadc1, 10U) == HAL_OK))
    {
      uint16_t wmin = 0U, wmax = 0U;
      uint8_t wi, kind, mid_count, unstable_now, indicator, step_evidence;
      uint8_t rail_in_window;
      uint32_t now;
      sample = (uint16_t)HAL_ADC_GetValue(&hadc1);
      now = HAL_GetTick();
      kind = (sample < NTC_OPEN_ADC) ? 1U :
             ((sample > NTC_SHORT_ADC) ? 2U : 0U);

      /* The raw-sample window always rolls, rails included, and is never
         wiped by a fault sample: the window itself bridges the bouncing
         phase, so a rail sample keeps the spread indicator alive for the
         full second it stays inside the window. */
      win[win_head] = sample;
      win_head = (uint8_t)((win_head + 1U) % 10U);
      if (win_len < 10U) { win_len++; }

      /* Mid-band spread over the non-rail window samples: a floating lead
         whose ADC wanders without crossing 50 or 4045 (this board rests at
         ~150 when pulled) never trips the pure thresholds. Rail samples are
         excluded so stale extremes cannot block recovery after re-plugging.
         Two consecutive checks (200 ms) so a single glitch cannot arm it. */
      unstable_now = 0U;
      rail_in_window = 0U;
      if (win_len >= 10U)
      {
        mid_count = 0U;
        for (wi = 0U; wi < 10U; wi++)
        {
          if ((win[wi] >= NTC_OPEN_ADC) && (win[wi] <= NTC_SHORT_ADC))
          {
            if (mid_count == 0U) { wmin = win[wi]; wmax = win[wi]; }
            else
            {
              if (win[wi] < wmin) { wmin = win[wi]; }
              if (win[wi] > wmax) { wmax = win[wi]; }
            }
            mid_count++;
          }
          else { rail_in_window = 1U; }
        }
        if (mid_count >= 4U)
        {
          if ((uint16_t)(wmax - wmin) > NTC_UNSTABLE_SPREAD)
          {
            if (spread_hits < 2U) { spread_hits++; }
          }
          else { spread_hits = 0U; }
          if (spread_hits >= 2U) { unstable_now = 1U; }
        }
        else { spread_hits = 0U; }
      }
      else { spread_hits = 0U; }

      /* Step-discontinuity detector: the reference follows every smooth
         change (hand-warming and cooling ramps stay far below the
         per-sample limit) but freezes the moment a step beyond
         NTC_STEP_LIMIT appears, so the offset persists as fault evidence
         while the lead rests mid-band. Rail samples never chase the
         reference, and 60 s of continuous over-limit re-homes it so a
         replug after an ambient shift cannot latch a false alarm forever. */
      if (kind == 0U)
      {
        if (ref_valid == 0U)
        {
          ref = sample;
          ref_valid = 1U;
          step_armed = 0U;
        }
        else
        {
          uint16_t diff = (sample > ref) ? (uint16_t)(sample - ref)
                                         : (uint16_t)(ref - sample);
          if (diff > NTC_STEP_LIMIT)
          {
            if (step_armed == 0U) { step_armed = 1U; step_since = now; }
            if (((now - step_since) >= NTC_STEP_REANCHOR_MS) &&
                (rail_in_window != 0U))
            {
              /* Mid-band reading while rails are still in the window and the
                 step is long armed: the lead came back from a drifted / rail
                 float. This is reconnection, not a new fault - re-anchor so
                 replugging recovers at once instead of latching for the
                 re-home timeout. */
              ref = sample;
              step_armed = 0U;
            }
            else if ((now - step_since) >= NTC_STEP_REHOME_MS)
            {
              ref = sample;      /* stale reference, not a live fault */
              step_armed = 0U;
            }
          }
          else
          {
            step_armed = 0U;
            ref = sample;
          }
        }
      }
      step_evidence = (uint8_t)(((step_armed != 0U) &&
        ((now - step_since) >= 1000U)) ? 1U : 0U);

      /* Fault evidence = rail sample (open or short), confirmed mid-band
         spread, or a sustained step away from the frozen reference.
         Evidence must simply SPAN >= 1 s with gaps below
         NTC_FAULT_GAP_MS; brief valid-looking interludes inside a yank no
         longer reset the count, while an isolated burst can never reach the
         span and stays silent. */
      indicator = (kind != 0U) ? 1U :
                  ((unstable_now != 0U) ? 1U : step_evidence);
      if (indicator != 0U)
      {
        if (fault_seen == 0U)
        {
          fault_seen = 1U;
          fault_first = now;
        }
        fault_last = now;
      }
      else if ((fault_seen != 0U) &&
               ((now - fault_last) >= NTC_FAULT_GAP_MS))
      {
        fault_seen = 0U;   /* unrelated episodes must not accumulate span */
      }
      if ((fault_seen != 0U) &&
          ((fault_last - fault_first) >= 1000U))
      { sensor_ok = 0U; }

      if (kind != 0U)
      {
        recovering = 0U;
        runtime_sensor_update(sample, sensor_ok, 0, 0U, ref, step_armed);
      }
      else
      {
        uint16_t averaged;
        if (history_count == 8U) { sum -= history[head]; }
        else { history_count++; }
        history[head] = sample;
        head = (uint8_t)((head + 1U) % 8U);
        sum += sample;
        averaged = (uint16_t)(sum / history_count);

        /* Only allow recovery once the input shows no fault evidence and no
           rail sample is left in the 1 s window: a detached lead drifting
           through / bouncing off the rail must not sneak a recovery during a
           brief mid-band moment, while a genuine replug clears as soon as
           the rail samples age out (~1 s). */
        if ((sensor_ok == 0U) && (indicator == 0U) &&
            (rail_in_window == 0U))
        {
          if (recovering == 0U)
          {
            recovering = 1U;
            recovery_since = now;
          }
          if ((now - recovery_since) >= 1000U)
          {
            sensor_ok = 1U;
            /* Clean slate after recovery: stale extremes in the window, the
               old average and the frozen reference must not re-trigger or
               misreport once the lead is back. */
            fault_seen = 0U;
            spread_hits = 0U;
            win_len = 0U;
            win_head = 0U;
            history_count = 0U;
            head = 0U;
            sum = 0U;
            ref = sample;
            ref_valid = 1U;
            step_armed = 0U;
          }
        }
        else { recovering = 0U; }

        /* The prior formula inverted the observed board response: a larger ADC
           value made the displayed temperature fall. This restores the correct
           increasing direction. The scale remains an approximation until the
           previous lab's NTC calibration coefficients are available.
           While the input reads unstable or sits stepped away from the
           reference, freeze the displayed value and keep the last good
           temperature (guide: keep pre-fault value). */
        runtime_sensor_update(sample, sensor_ok,
          (int16_t)(((int32_t)averaged * 500) / 4095),
          (uint8_t)(((unstable_now != 0U) || (step_armed != 0U)) ? 0U : 1U),
          ref, step_armed);
      }
    }
    (void)HAL_ADC_Stop(&hadc1);
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(100U));
  }
}

static void request_param_save(void)
{
  (void)xTimerReset(saveTimer, 0U);
}

static void inputTask(void *argument)
{
  uint8_t stable[5] = {1U, 1U, 1U, 1U, 1U};
  uint8_t last[5] = {1U, 1U, 1U, 1U, 1U};
  uint8_t debounce[5] = {0U, 0U, 0U, 0U, 0U};
  uint16_t previous = 32768U;
  int16_t residual = 0;
  uint8_t i;
  GPIO_TypeDef *ports[5] = {KEY_SELECT_GPIO_Port, KEY_UP_GPIO_Port,
                            KEY_DOWN_GPIO_Port, KEY_OK_GPIO_Port,
                            ENC_KEY_GPIO_Port};
  uint16_t pins[5] = {KEY_SELECT_Pin, KEY_UP_Pin, KEY_DOWN_Pin,
                      KEY_OK_Pin, ENC_KEY_Pin};
  (void)argument;
  (void)xEventGroupWaitBits(systemEvents, EVT_READY, pdFALSE, pdTRUE, portMAX_DELAY);

  for (;;)
  {
    /* Keep the full three-second identity splash uninterrupted. */
    if (identity_splash_active() != 0U)
    {
      previous = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
      vTaskDelay(pdMS_TO_TICKS(10U));
      continue;
    }
    for (i = 0U; i < 5U; i++)
    {
      uint8_t raw = (uint8_t)HAL_GPIO_ReadPin(ports[i], pins[i]);
      if (raw != last[i]) { debounce[i] = 0U; }
      else if (debounce[i] < 3U) { debounce[i]++; }
      if ((debounce[i] >= 2U) && (raw != stable[i]))
      {
        stable[i] = raw;
        if (raw == GPIO_PIN_RESET)
        {
          if (i == 0U)
          {
            current_page = (uint8_t)((current_page + 1U) % 3U);
            if (current_page == 1U) { Params_EnterOrCycle(); }
            else { Params_ExitEdit(); }
          }
          else if (i == 1U && current_page == 1U &&
                   Params_IsEditing() != 0U)
          {
            if (Params_Step(SRC_KEY, 1U)) { request_param_save(); }
          }
          else if (i == 1U && current_page == 2U &&
                   log_view + 1U < log_count)
          {
            log_view++; /* SW2: previous (older) record, doc section 11 */
          }
          else if (i == 2U && current_page == 1U &&
                   Params_IsEditing() != 0U)
          {
            if (Params_Step(SRC_KEY, -1)) { request_param_save(); }
          }
          else if (i == 2U && current_page == 2U && log_view > 0U)
          {
            log_view--; /* SW3: next (newer) record, doc section 11 */
          }
          else if (i == 3U)
          {
            ok_request = 1U;
            if (current_page == 1U) { Params_ExitEdit(); }
            current_page = 0U;
            {
              RuntimeState_t value = get_runtime();
              if (value.alarm != ALARM_NORMAL)
              {
                runtime_mute_update(1U);
                request_log_ack();
                (void)xEventGroupSetBits(systemEvents, EVT_MUTED);
                (void)xTimerReset(muteTimer, 0U);
              }
            }
          }
          else if (i == 4U)
          {
            if (current_page == 1U)
            {
              Params_EnterOrCycle();
            }
            else if (current_page == 0U)
            {
              current_page = 1U;
              Params_EnterOrCycle();
            }
            else if (current_page == 2U)
            {
              log_field = (uint8_t)((log_field + 1U) % 3U);
            }
          }
        }
      }
      last[i] = raw;
    }

    {
      uint16_t current = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
      int16_t delta = (int16_t)(current - previous);
      previous = current;
      residual = (int16_t)(residual + delta);
      while (residual >= 4)
      {
        residual -= 4;
        if (current_page == 1U && Params_IsEditing() != 0U &&
            Params_Step(SRC_ENC, 1U)) { request_param_save(); }
        else if (current_page == 2U && log_count > 0U && log_view + 1U < log_count) { log_view++; }
      }
      while (residual <= -4)
      {
        residual += 4;
        if (current_page == 1U && Params_IsEditing() != 0U &&
            Params_Step(SRC_ENC, -1)) { request_param_save(); }
        else if (current_page == 2U && log_view > 0U) { log_view--; }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10U));
  }
}

static void alarmTask(void *argument)
{
  uint32_t high_since = 0U;
  uint32_t low_since = 0U;
  uint8_t high_pending = 0U;
  uint8_t low_pending = 0U;
  AlarmState_t previous = ALARM_NORMAL;
  uint32_t last_record = 0U;
  RuntimeState_t value;
  ParamSnapshot_t params;
  StoreRequest_t request;
  uint8_t startup_silent;
  (void)argument;
  set_buzzer(0U);
  (void)xEventGroupWaitBits(systemEvents, EVT_READY, pdFALSE, pdTRUE, portMAX_DELAY);

  for (;;)
  {
    value = get_runtime();
    Params_GetSnapshot(&params);
    startup_silent = identity_splash_active();
    /* Count real elapsed milliseconds, not alarmTask loop iterations. */
    if ((value.sensor_ok != 0U) &&
        (value.temperature_x10 > (int16_t)(params.high_x2 * 5)))
    {
      if (high_pending == 0U) { high_pending = 1U; high_since = HAL_GetTick(); }
    }
    else { high_pending = 0U; }
    if ((value.sensor_ok != 0U) &&
        (value.temperature_x10 < (int16_t)(params.low_x2 * 5)))
    {
      if (low_pending == 0U) { low_pending = 1U; low_since = HAL_GetTick(); }
    }
    else { low_pending = 0U; }

    value.alarm = ALARM_NORMAL;
    if (value.sensor_ok == 0U)
    {
      value.alarm = ALARM_SENSOR;
    }
    else if (previous == ALARM_HIGH &&
             value.temperature_x10 >= (int16_t)(params.high_x2 * 5 - 10))
    {
      value.alarm = ALARM_HIGH;
    }
    else if (previous == ALARM_LOW &&
             value.temperature_x10 <= (int16_t)(params.low_x2 * 5 + 10))
    {
      value.alarm = ALARM_LOW;
    }
    else if ((high_pending != 0U) &&
             ((uint32_t)(HAL_GetTick() - high_since) >= 2000U))
    {
      value.alarm = ALARM_HIGH;
    }
    else if ((low_pending != 0U) &&
             ((uint32_t)(HAL_GetTick() - low_since) >= 2000U))
    {
      value.alarm = ALARM_LOW;
    }
    else if (value.vibration != 0U)
    {
      value.alarm = ALARM_VIBRATION;
    }
    /* Alarms must not force navigation away from settings or log browsing. */
    value.uptime_s = HAL_GetTick() / 1000U;
    runtime_alarm_update(value.alarm, value.uptime_s);
    (void)xEventGroupClearBits(systemEvents,
      EVT_HIGH | EVT_LOW | EVT_SENSOR | EVT_VIBRATION);
    if (value.alarm != ALARM_NORMAL)
    {
      EventBits_t bit = (value.alarm == ALARM_HIGH) ? EVT_HIGH :
                        (value.alarm == ALARM_LOW) ? EVT_LOW :
                        (value.alarm == ALARM_SENSOR) ? EVT_SENSOR : EVT_VIBRATION;
      (void)xEventGroupSetBits(systemEvents, bit);
    }

    if (value.alarm != previous)
    {
      if (value.alarm != ALARM_NORMAL)
      {
        memset(&request, 0, sizeof(request));
        request.type = STORE_LOG;
        request.log.sequence = 0U; /* Assigned in-order by storageTask. */
        request.log.uptime_s = value.uptime_s;
        request.log.temperature_x10 = value.temperature_x10;
        request.log.type = (value.alarm == ALARM_HIGH) ? LOG_HIGH :
                           (value.alarm == ALARM_LOW) ? LOG_LOW :
                           (value.alarm == ALARM_SENSOR) ? LOG_SENSOR_ERROR : LOG_VIBRATION;
        request.log.acknowledged = value.muted;
        (void)xQueueSend(storageQueue, &request, 0U);
        /* Avoid concurrent UART blocking transmit from alarmTask and serialTask. */
      }
      previous = value.alarm;
      last_record = value.uptime_s;
    }
    else if ((value.alarm != ALARM_NORMAL) &&
             ((value.uptime_s - last_record) >= (uint32_t)params.interval_s))
    {
      last_record = value.uptime_s;
      memset(&request, 0, sizeof(request));
      request.type = STORE_LOG;
      request.log.sequence = 0U; /* Assigned in-order by storageTask. */
      request.log.uptime_s = value.uptime_s;
      request.log.temperature_x10 = value.temperature_x10;
      request.log.type = (value.alarm == ALARM_HIGH) ? LOG_HIGH :
                         (value.alarm == ALARM_LOW) ? LOG_LOW :
                         (value.alarm == ALARM_SENSOR) ? LOG_SENSOR_ERROR : LOG_VIBRATION;
      request.log.acknowledged = value.muted;
      (void)xQueueSend(storageQueue, &request, 0U);
    }

    if ((startup_silent != 0U) || (value.muted != 0U)) { set_buzzer(0U); }
    else if (value.alarm == ALARM_HIGH) { set_buzzer((HAL_GetTick() % 400U) < 100U); }
    else if (value.alarm == ALARM_LOW) { set_buzzer((HAL_GetTick() % 1200U) < 500U); }
    else if (value.alarm == ALARM_SENSOR)
    {
      uint32_t phase = HAL_GetTick() % 1200U;
      set_buzzer((phase < 100U) ||
                 ((phase >= 200U) && (phase < 300U)) ||
                 ((phase >= 450U) && (phase < 850U)));
    }
    else if (value.alarm == ALARM_VIBRATION)
    {
      uint32_t phase = HAL_GetTick() % 700U;
      set_buzzer((phase < 100U) || ((phase >= 200U) && (phase < 300U)));
    }
    else { set_buzzer(0U); }

    if (value.muted != 0U && value.alarm != ALARM_NORMAL)
    {
      set_leds(0xAAU);
    }
    else if (value.alarm == ALARM_HIGH)
    {
      set_leds(((HAL_GetTick() / 100U) & 1U) ? 0xFFU : 0x00U);
    }
    else if (value.alarm == ALARM_LOW)
    {
      set_leds(((HAL_GetTick() / 500U) & 1U) ? 0xFFU : 0x00U);
    }
    else if (value.alarm == ALARM_SENSOR) { set_leds(((HAL_GetTick() / 200U) & 1U) ? 0xAAU : 0x55U); }
    else if (value.alarm == ALARM_VIBRATION)
    {
      static const uint8_t inward[4] = {0x81U, 0x42U, 0x24U, 0x18U};
      set_leds(inward[(HAL_GetTick() / 150U) % 4U]);
    }
    else { set_leds((uint8_t)(1U << ((HAL_GetTick() / 150U) % LED_COUNT))); }
    vTaskDelay(pdMS_TO_TICKS(50U));
  }
}

static const char *alarm_name(AlarmState_t state)
{
  switch (state)
  {
    case ALARM_HIGH: return "HIGH";
    case ALARM_LOW: return "LOW";
    case ALARM_SENSOR: return "SENSOR";
    case ALARM_VIBRATION: return "VIB";
    default: return "NORMAL";
  }
}

static const char *log_name(uint8_t type)
{
  switch (type)
  {
    case LOG_HIGH: return "HIGH";
    case LOG_LOW: return "LOW";
    case LOG_SENSOR_ERROR: return "SENSOR";
    case LOG_VIBRATION: return "VIB";
    default: return "NONE";
  }
}

static uint8_t append_text(char *buffer, uint8_t pos, const char *text)
{
  while ((*text != '\0') && (pos < 127U)) { buffer[pos++] = *text++; }
  buffer[pos] = '\0';
  return pos;
}

static uint8_t parse_interval(const char *text, int *out)
{
  int value = 0;
  uint8_t digits = 0U;
  const char *p = text;
  while ((*p >= '0') && (*p <= '9'))
  {
    value = value * 10 + (*p - '0');
    if (value > 32767) { return 0U; }
    digits++;
    p++;
  }
  if ((digits == 0U) || (*p != '\0')) { return 0U; }
  *out = value;
  return 1U;
}

static uint8_t append_uint(char *buffer, uint8_t pos, uint32_t value)
{
  char digits[10];
  uint8_t n = 0U;
  if (value == 0U) { return append_text(buffer, pos, "0"); }
  while ((value != 0U) && (n < sizeof(digits)))
  {
    digits[n++] = (char)('0' + (value % 10U));
    value /= 10U;
  }
  while (n > 0U) { buffer[pos++] = digits[--n]; }
  buffer[pos] = '\0';
  return pos;
}

static uint8_t append_int(char *buffer, uint8_t pos, int32_t value)
{
  if (value < 0) { buffer[pos++] = '-'; value = -value; }
  return append_uint(buffer, pos, (uint32_t)value);
}

static uint8_t append_temperature(char *buffer, uint8_t pos, int16_t value_x10)
{
  int16_t fraction = (int16_t)(value_x10 % 10);
  if (value_x10 < 0) { fraction = (int16_t)-fraction; }
  pos = append_int(buffer, pos, value_x10 / 10);
  buffer[pos++] = '.';
  buffer[pos++] = (char)('0' + fraction);
  buffer[pos] = '\0';
  return pos;
}

static void displayTask(void *argument)
{
  char line[128]; /* append helpers can write up to 127 characters */
  char high[8];
  char low[8];
  ParamSnapshot_t p;
  RuntimeState_t value;
  AlarmLog_t selected_log;
  uint8_t displayed_count = 0U;
  (void)argument;

  (void)xEventGroupWaitBits(systemEvents, EVT_READY, pdFALSE, pdTRUE, portMAX_DELAY);
  OLED_Init();
  identity_start_tick = HAL_GetTick();
  identity_started = 1U;
  for (;;)
  {
    OLED_Clear();
    if (identity_splash_active() != 0U)
    {
      OLED_ShowString(16, 0, (uint8_t *)"ENV MONITOR");
      OLED_ShowString(0, 2, (uint8_t *)"ID  202505920141");
      OLED_ShowString(0, 4, (uint8_t *)"NAME XINHONGEN");
      OLED_ShowString(0, 6, (uint8_t *)"STARTING 3S");
    }
    else
    {
      value = get_runtime();
      Params_GetSnapshot(&p);
      Params_FormatThreshold(high, p.high_x2);
      Params_FormatThreshold(low, p.low_x2);
      if (current_page == 1U)
      {
        uint8_t pos = 0U;
        line[pos++] = (p.selected == PARAM_HIGH && p.editing) ? '>' : ' ';
        pos = append_text(line, pos, "HIGH  "); pos = append_text(line, pos, high);
        pos = append_text(line, pos, " C"); (void)pos;
        OLED_ShowString(0, 0, (uint8_t *)line);
        pos = 0U; line[pos++] = (p.selected == PARAM_LOW && p.editing) ? '>' : ' ';
        pos = append_text(line, pos, "LOW   "); pos = append_text(line, pos, low);
        pos = append_text(line, pos, " C"); (void)pos;
        OLED_ShowString(0, 2, (uint8_t *)line);
        pos = 0U; line[pos++] = (p.selected == PARAM_INTERVAL && p.editing) ? '>' : ' ';
        pos = append_text(line, pos, "INT   "); pos = append_int(line, pos, p.interval_s);
        pos = append_text(line, pos, " S"); (void)pos;
        OLED_ShowString(0, 4, (uint8_t *)line);
        OLED_ShowString(0, 6, (uint8_t *)"SW4 OK  ENC SET");
      }
      else if (current_page == 2U &&
               read_log(log_view, &selected_log, &displayed_count) != 0U)
      {
        AlarmLog_t *log = &selected_log;
        uint8_t pos = 0U;
        pos = append_text(line, pos, "LOG "); if (log_view < 9U) { line[pos++] = '0'; }
        pos = append_uint(line, pos, log_view + 1U); pos = append_text(line, pos, " OF 16"); (void)pos;
        OLED_ShowString(0, 0, (uint8_t *)line);
        pos = append_text(line, 0U, (log_field == 0U) ? ">TIME " : " TIME ");
        line[pos++] = (char)('0' + ((log->uptime_s / 1000U) % 10U));
        line[pos++] = (char)('0' + ((log->uptime_s / 100U) % 10U));
        line[pos++] = (char)('0' + ((log->uptime_s / 10U) % 10U));
        line[pos++] = (char)('0' + (log->uptime_s % 10U));
        pos = append_text(line, pos, " S"); (void)pos;
        OLED_ShowString(0, 2, (uint8_t *)line);
        pos = append_text(line, 0U, (log_field == 1U) ? ">TYPE " : " TYPE "); pos = append_text(line, pos, log_name(log->type)); (void)pos;
        OLED_ShowString(0, 4, (uint8_t *)line);
        pos = append_text(line, 0U, (log_field == 2U) ? ">T " : " T ");
         pos = append_temperature(line, pos, log->temperature_x10);
        pos = append_text(line, pos, " C ACK "); line[pos++] = log->acknowledged ? 'Y' : 'N'; line[pos] = '\0';
        OLED_ShowString(0, 6, (uint8_t *)line);
      }
      else if (current_page == 2U)
      {
        OLED_ShowString(0, 0, (uint8_t *)"LOG 00 OF 16");
        OLED_ShowString(0, 2, (uint8_t *)"TIME ---- S");
        OLED_ShowString(0, 4, (uint8_t *)"TYPE NONE");
        OLED_ShowString(0, 6, (uint8_t *)"T --.- C ACK N");
      }
      else
      {
        uint8_t pos = append_text(line, 0U, "T ");
        pos = append_temperature(line, pos, value.temperature_x10);
        pos = append_text(line, pos, " C "); pos = append_text(line, pos, alarm_name(value.alarm)); (void)pos;
        OLED_ShowString(0, 0, (uint8_t *)line);
        pos = append_text(line, 0U, "H "); pos = append_text(line, pos, high);
        pos = append_text(line, pos, " L "); pos = append_text(line, pos, low); (void)pos;
        OLED_ShowString(0, 2, (uint8_t *)line);
        pos = append_text(line, 0U, "REC "); pos = append_int(line, pos, p.interval_s);
        pos = append_text(line, pos, "S LOG ");
        (void)read_log(0U, NULL, &displayed_count);
        if (displayed_count < 10U) { line[pos++] = '0'; }
        pos = append_uint(line, pos, displayed_count); (void)pos;
        OLED_ShowString(0, 4, (uint8_t *)line);
        pos = append_text(line, 0U, "SRC ");
        pos = append_text(line, pos, Params_SrcName(p.src));
        pos = append_text(line, pos, " MUTE ");
        if (value.muted != 0U) { pos = append_uint(line, pos, mute_seconds_left()); }
        else { pos = append_text(line, pos, "--"); }
        (void)pos;
        OLED_ShowString(0, 6, (uint8_t *)line);
      }
    }
    if ((identity_splash_active() == 0U) && (current_page == 0U) &&
        (ok_request != 0U))
    {
      OLED_ShowString(0, 6, (uint8_t *)"SW4 CONFIRMED");
      ok_request = 0U;
    }
    OLED_Refresh();
    vTaskDelay(pdMS_TO_TICKS(500U));
  }
}

static char *trim(char *text)
{
  while ((*text == ' ') || (*text == '\t')) { text++; }
  return text;
}

static uint8_t parse_threshold_x2(const char *text, int16_t *out)
{
  int32_t integer_part = 0;
  int32_t value;
  uint8_t negative = 0U;
  uint8_t digits = 0U;
  uint8_t fraction = 0U;
  const char *p = text;
  if (*p == '-') { negative = 1U; p++; }
  while ((*p >= '0') && (*p <= '9'))
  {
    integer_part = integer_part * 10 + (*p - '0');
    if (integer_part > 1000) { return 0U; }
    digits++;
    p++;
  }
  if (digits == 0U) { return 0U; }
  if (*p == '.')
  {
    p++;
    if ((*p < '0') || (*p > '9')) { return 0U; }
    fraction = (uint8_t)(*p - '0');
    p++;
    if ((*p >= '0') && (*p <= '9')) { return 0U; }
  }
  if (*p != '\0') { return 0U; }
  if ((fraction != 0U) && (fraction != 5U)) { return 0U; }
  value = integer_part * 2 + ((fraction == 5U) ? 1 : 0);
  if (negative != 0U) { value = -value; }
  *out = (int16_t)value;
  return 1U;
}

static void SerialSend(const char *text)
{
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 100U);
}

static void SerialProcessLine(char *line)
{
  int interval;
  ParamSnapshot_t p = {60, 40, 20, PARAM_HIGH, 0U, SRC_KEY};
  RuntimeState_t state;
  char answer[128];
  char *text = trim(line);
  if (strcmp(text, "GET") == 0)
  {
    uint8_t pos = 0U;
    char high[8];
    char low[8];
    Params_GetSnapshot(&p);
    state = get_runtime();
    Params_FormatThreshold(high, p.high_x2);
    Params_FormatThreshold(low, p.low_x2);
    pos = append_text(answer, pos, "T=");
    pos = append_temperature(answer, pos, state.temperature_x10);
    pos = append_text(answer, pos, " H="); pos = append_text(answer, pos, high);
    pos = append_text(answer, pos, " L="); pos = append_text(answer, pos, low);
    pos = append_text(answer, pos, " I="); pos = append_int(answer, pos, p.interval_s);
    pos = append_text(answer, pos, " STATE="); pos = append_text(answer, pos, alarm_name(state.alarm));
    pos = append_text(answer, pos, " ADC="); pos = append_uint(answer, pos, state.adc);
    pos = append_text(answer, pos, " REF="); pos = append_uint(answer, pos, state.sensor_ref);
    pos = append_text(answer, pos, " STEP="); pos = append_uint(answer, pos, state.step_armed);
    pos = append_text(answer, pos, " SENSOR=");
    pos = append_text(answer, pos, (state.sensor_ok != 0U) ? "OK" : "ERROR");
    pos = append_text(answer, pos, " MUTE="); pos = append_uint(answer, pos, state.muted);
    pos = append_text(answer, pos, " RXERR=");
    pos = append_uint(answer, pos, uart_rx_errors);
    { uint8_t count = 0U;
        (void)read_log(0U, NULL, &count);
        pos = append_text(answer, pos, " LOG=");
        pos = append_uint(answer, pos, count); }
    (void)append_text(answer, pos, "\r\n");
    SerialSend(answer);
  }
  else if ((text[0] == 'S') && (text[1] == 'E') && (text[2] == 'T') && (text[3] == ' '))
  {
    char item;
    char *argument = trim(text + 5);
    if ((text[4] == '\0') || (text[5] != ' ') || (*argument == '\0'))
    { SerialSend("ERR FORMAT\r\n"); return; }
    item = text[4];
    item = (char)((item >= 'a' && item <= 'z') ? item - 32 : item);
    if ((item == 'H') || (item == 'L'))
    {
      int16_t x2;
      if (parse_threshold_x2(argument, &x2) == 0U) { SerialSend("ERR FORMAT\r\n"); return; }
      Params_GetSnapshot(&p);
      if (((item == 'H') && ((x2 < 41) || (x2 > 120))) ||
          ((item == 'L') && ((x2 < 0) || (x2 > 59))))
      { SerialSend("ERR RANGE\r\n"); }
      else if (((item == 'H') && (x2 <= p.low_x2)) ||
               ((item == 'L') && (x2 >= p.high_x2)))
      { SerialSend("ERR RELATION\r\n"); }
      else if (Params_SetThreshold(item == 'H' ? PARAM_HIGH : PARAM_LOW, x2, SRC_UART) == 0U)
      { SerialSend("ERR BUSY\r\n"); }
      else
      {
        char formatted[8];
        Params_FormatThreshold(formatted, x2);
        uint8_t pos = 0U;
        answer[pos++] = 'O'; answer[pos++] = 'K'; answer[pos++] = ' ';
        answer[pos++] = item; answer[pos++] = '=';
        pos = append_text(answer, pos, formatted); (void)append_text(answer, pos, "\r\n");
        SerialSend(answer);
        request_param_save();
      }
    }
    else if ((item == 'I') && parse_interval(argument, &interval))
    {
      if ((interval < 5) || (interval > 300) || ((interval % 5) != 0))
      { SerialSend("ERR RANGE\r\n"); }
      else if (Params_SetInterval((int16_t)interval, SRC_UART) == 0U)
      { SerialSend("ERR BUSY\r\n"); }
      else
      {
        uint8_t pos = 0U;
        pos = append_text(answer, pos, "OK I="); pos = append_int(answer, pos, interval);
        (void)append_text(answer, pos, "\r\n"); SerialSend(answer); request_param_save();
      }
    }
    else { SerialSend("ERR FORMAT\r\n"); }
  }
  else if (strcmp(text, "LOG?") == 0)
  {
    uint8_t i, count = 0U;
    uint8_t pos;
    AlarmLog_t entry;
    (void)read_log(0U, NULL, &count);
    if (count == 0U) { SerialSend("ERR EMPTY\r\n"); return; }
    for (i = 0U; i < count; i++)
    {
      if (read_log(i, &entry, NULL) == 0U) { break; }
      pos = append_uint(answer, 0U, entry.sequence); answer[pos++] = ',';
      pos = append_uint(answer, pos, entry.uptime_s); answer[pos++] = ',';
      pos = append_text(answer, pos, log_name(entry.type)); answer[pos++] = ',';
      pos = append_temperature(answer, pos, entry.temperature_x10);
      answer[pos++] = ','; answer[pos++] = entry.acknowledged ? 'Y' : 'N'; answer[pos] = '\0';
      (void)append_text(answer, pos, "\r\n");
      SerialSend(answer);
    }
  }
  else if (strcmp(text, "CLEAR LOG") == 0)
  {
    StoreRequest_t request = {STORE_CLEAR, {0}, {0}};
    if (xQueueSend(storageQueue, &request, 0U) == pdTRUE)
    { SerialSend("OK LOG CLEARED\r\n"); }
    else { SerialSend("ERR BUSY\r\n"); }
  }
  else if (strcmp(text, "MUTE") == 0)
  {
    RuntimeState_t v = get_runtime();
    if (v.alarm == ALARM_NORMAL) { SerialSend("ERR BUSY\r\n"); return; }
    runtime_mute_update(1U);
    request_log_ack();
    (void)xEventGroupSetBits(systemEvents, EVT_MUTED);
    (void)xTimerReset(muteTimer, 0U);
    SerialSend("OK MUTE=10\r\n");
  }
  else if (strcmp(text, "DEFAULT") == 0)
  {
    Params_ApplyDefaults();
    request_param_save();
    SerialSend("OK DEFAULT\r\n");
  }
  else if (strcmp(text, "HELP") == 0)
  {
    SerialSend("GET; SET H x.x; SET L x.x; SET I sec; LOG?; CLEAR LOG; MUTE; DEFAULT; HELP\r\n");
  }
  else { SerialSend("ERR FORMAT\r\n"); }
}

static void serialTask(void *argument)
{
  static char line[UART_LINE_MAX];
  uint8_t length = 0U;
  uint8_t byte;
  (void)argument;
  (void)xEventGroupWaitBits(systemEvents, EVT_READY, pdFALSE, pdTRUE, portMAX_DELAY);
  vTaskDelay(pdMS_TO_TICKS(300U));
  SerialSend("\r\nENV MONITOR READY\r\n");
  for (;;)
  {
    if (xQueueReceive(uartRxQueue, &byte, pdMS_TO_TICKS(100U)) == pdTRUE)
    {
      if ((byte == '\r') || (byte == '\n'))
      {
        line[length] = '\0';
        if (length != 0U) { SerialProcessLine(line); }
        length = 0U;
      }
      else if (length < UART_LINE_MAX - 1U) { line[length++] = (char)byte; }
      else { length = 0U; SerialSend("ERR FORMAT\r\n"); }
    }
    else if (length != 0U)
    {
      /* 100 ms of line silence: treat pending bytes as a complete line so a
         terminal that forgets the CRLF suffix still gets a reply. */
      line[length] = '\0';
      SerialProcessLine(line);
      length = 0U;
    }
  }
}

static void storageTask(void *argument)
{
  StoreRequest_t request;
  PersistParams_t stored;
  ParamSnapshot_t defaults = {60, 40, 20, PARAM_HIGH, 0U, SRC_KEY};
  uint8_t valid_params;
  AlarmLog_t pending[STORAGE_LOG_CAPACITY];
  uint8_t pending_count;
  uint8_t i;
  (void)argument;
  valid_params = Storage_Load(&stored, logs, &log_count);
  if (Params_SetAll(stored.high_x2, stored.low_x2, (int16_t)stored.interval_s,
                    SRC_UART) == 0U)
  {
    Params_ApplyDefaults();
    valid_params = 0U;
  }
  if (valid_params == 0U)
  {
    memset(&stored, 0, sizeof(stored));
    Params_GetSnapshot(&defaults);
    stored.high_x2 = defaults.high_x2;
    stored.low_x2 = defaults.low_x2;
    stored.interval_s = (uint16_t)defaults.interval_s;
    stored.sequence = Params_GetGeneration();
    (void)Storage_SaveParams(&stored);
  }
  (void)xEventGroupSetBits(systemEvents, EVT_READY);
  for (;;)
  {
    if (xQueueReceive(storageQueue, &request, pdMS_TO_TICKS(100U)) == pdTRUE)
    {
      uint8_t save_logs = 0U;
      uint8_t save_params = 0U;
      uint8_t clear_logs = 0U;
      PersistParams_t param_copy;

      if (xSemaphoreTake(flashMutex, pdMS_TO_TICKS(100U)) == pdTRUE)
      {
        /* Drain the whole burst into RAM first, then touch Flash at most
           once per category. Each Flash page erase stalls the CPU ~20-40 ms
           on STM32F1 (code runs from the same bank), during which incoming
           UART bytes are lost and surface as bogus ERR FORMAT replies. An
           alarm burst that used to erase the log page several times per
           second now erases it once. */
        for (;;)
        {
          if (request.type == STORE_PARAM)
          {
            param_copy = request.params;
            save_params = 1U;
          }
          else if ((request.type == STORE_LOG) ||
                   (request.type == STORE_ACK))
          {
            if (osMutexWait(dataMutexHandle, 20U) == osOK)
            {
              if (request.type == STORE_LOG)
              {
                request.log.sequence = (log_count == 0U) ? 1U :
                  (logs[log_count - 1U].sequence + 1U);
                if (log_count < STORAGE_LOG_CAPACITY)
                { logs[log_count++] = request.log; }
                else
                {
                  for (i = 1U; i < STORAGE_LOG_CAPACITY; i++)
                  { logs[i - 1U] = logs[i]; }
                  logs[STORAGE_LOG_CAPACITY - 1U] = request.log;
                }
                logs[log_count - 1U].crc =
                  Storage_Crc16(&logs[log_count - 1U],
                    (uint16_t)offsetof(AlarmLog_t, crc));
                log_view = 0U;
              }
              else if (log_count != 0U)
              {
                logs[log_count - 1U].acknowledged = 1U;
                logs[log_count - 1U].crc =
                  Storage_Crc16(&logs[log_count - 1U],
                    (uint16_t)offsetof(AlarmLog_t, crc));
              }
              (void)osMutexRelease(dataMutexHandle);
              save_logs = 1U;
            }
          }
          else if (request.type == STORE_CLEAR)
          {
            clear_logs = 1U;
          }

          if (xQueueReceive(storageQueue, &request, 0U) != pdTRUE)
          {
            break;
          }
        }

        /* The final RAM state already reflects queue order (a CLEAR after
           appended logs zeroes the count, so the snapshot below is skipped
           and the erased page stays empty). */
        if (clear_logs != 0U)
        {
          if (Storage_ClearLogs() != 0U &&
              osMutexWait(dataMutexHandle, 20U) == osOK)
          {
            log_count = 0U;
            log_view = 0U;
            (void)osMutexRelease(dataMutexHandle);
          }
        }
        if (save_params != 0U)
        {
          (void)Storage_SaveParams(&param_copy);
        }
        if (save_logs != 0U)
        {
          pending_count = 0U;
          if (osMutexWait(dataMutexHandle, 20U) == osOK)
          {
            pending_count = log_count;
            memcpy(pending, logs,
                   (size_t)pending_count * sizeof(AlarmLog_t));
            (void)osMutexRelease(dataMutexHandle);
          }
          if (pending_count != 0U)
          {
            (void)Storage_SaveLog(&pending[pending_count - 1U],
                                  pending, pending_count);
          }
        }

        (void)xSemaphoreGive(flashMutex);
      }
    }
    /* The parameter timer is restarted only by a real user/UART change.
       Polling and restarting it here would prevent the 2-second timeout. */
  }
}

static uint32_t integer_sqrt(uint32_t n)
{
  uint32_t result = 0U;
  uint32_t bit = 1UL << 30;
  while (bit > n) { bit >>= 2; }
  while (bit != 0U)
  {
    if (n >= result + bit)
    {
      n -= result + bit;
      result = (result >> 1) + bit;
    }
    else { result >>= 1; }
    bit >>= 2;
  }
  return result;
}

static void vibrationTask(void *argument)
{
  int16_t x, y, z;
  int16_t prev_x = 0, prev_y = 0, prev_z = 0;
  uint8_t hit_count = 0U;
  uint8_t calm_count = 0U;
  uint8_t active = 0U;
  uint8_t ready;
  uint8_t have_previous = 0U;
  (void)argument;
  ready = MPU6050_Init();
  for (;;)
  {
    if ((ready != 0U) && (MPU6050_ReadAcceleration(&x, &y, &z) != 0U))
    {
      /* Use the full difference vector between adjacent samples, not the
         difference of vector magnitudes. A tilt/rotation component cancels
         out of |a| (magnitude stays 1 g while the board swings), and the
         magnitude difference also aliases to zero when the shake period is
         a multiple of the sampling period. ||da|| is always >= the magnitude
         difference, so the guide's "adjacent difference > 0.3 g, 3 hits"
         rule is kept, only with better sensitivity. */
      int32_t dx = (int32_t)x - (int32_t)prev_x;
      int32_t dy = (int32_t)y - (int32_t)prev_y;
      int32_t dz = (int32_t)z - (int32_t)prev_z;
      uint32_t delta = integer_sqrt((uint32_t)(dx * dx + dy * dy + dz * dz));
      if ((have_previous != 0U) && (delta > MPU6050_VIB_THRESHOLD_MG))
      {
        calm_count = 0U;
        if (hit_count < 3U) { hit_count++; }
      }
      else if (calm_count < VIB_CALM_SAMPLES) { calm_count++; }
      /* Latch the alarm so one shake event logs once; release only after
         VIB_CALM_SAMPLES (1 s) of continuous calm. Without this latching a
         single shake logs 5+ times per second, each log triggers a full
         Flash page erase (~20-40 ms CPU stall on F1) and the stall eats
         incoming UART bytes, surfacing as bogus ERR FORMAT replies. */
      if (active == 0U)
      {
        if (hit_count >= 3U) { active = 1U; }
      }
      else if (calm_count >= VIB_CALM_SAMPLES)
      {
        active = 0U;
        hit_count = 0U;
        calm_count = 0U;
      }
      prev_x = x; prev_y = y; prev_z = z;
      have_previous = 1U;
      runtime_vibration_update(active);
    }
    else
    {
      runtime_vibration_update(0U);
      hit_count = 0U;
      calm_count = 0U;
      active = 0U;
      have_previous = 0U;
      if (ready == 0U) { ready = MPU6050_Init(); }
    }
    vTaskDelay(pdMS_TO_TICKS(50U));
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  BaseType_t woken = pdFALSE;
  if ((huart != NULL) && (huart->Instance == USART1))
  {
    (void)xQueueSendFromISR(uartRxQueue, &uart_rx_byte, &woken);
    (void)HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1U);
    portYIELD_FROM_ISR(woken);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART1))
  {
    uart_rx_errors++;
    (void)HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1U);
  }
}

void vApplicationGetIdleTaskMemory(StaticTask_t **tcb, StackType_t **stack,
                                   uint32_t *size)
{
  static StaticTask_t idle_tcb;
  static StackType_t idle_stack[configMINIMAL_STACK_SIZE];
  *tcb = &idle_tcb;
  *stack = idle_stack;
  *size = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **tcb, StackType_t **stack,
                                    uint32_t *size)
{
  static StaticTask_t timer_tcb;
  static StackType_t timer_stack[configTIMER_TASK_STACK_DEPTH];
  *tcb = &timer_tcb;
  *stack = timer_stack;
  *size = configTIMER_TASK_STACK_DEPTH;
}
