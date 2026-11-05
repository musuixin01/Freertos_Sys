/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    params.c
  * @brief   Single owner of the shared parameter set. Every key / encoder /
  *          UART modification goes through this module and is serialised by
  *          the FreeRTOS dataMutex (created in freertos.c).
  ******************************************************************************
  */
/* USER CODE END Header */

#include "params.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "semphr.h"

/* The mutex is created together with the other RTOS objects in freertos.c. */
extern osMutexId dataMutexHandle;

/* Guide: high 20.5..60 C, low 0..29.5 C, with at least 0.5 C separation. */
#define HIGH_X2_MIN       (41)
#define HIGH_X2_MAX       (120)
#define LOW_X2_MIN        (0)
#define LOW_X2_MAX        (59)
#define INTERVAL_MIN      (5)        /* 5 s     */
#define INTERVAL_MAX      (300)      /* 300 s   */

#define THRESHOLD_STEP_X2 (1)        /* 0.5 C in 0.5 C units */
#define INTERVAL_STEP_S   (5)        /* 5 s                  */

#define MUTEX_WAIT_MS     (20U)

/* Defaults: high 30.0 C, low 20.0 C, interval 20 s, H selected. */
static ParamSnapshot_t s_params = {60, 40, 20, PARAM_HIGH, 0U, SRC_KEY};
static uint32_t s_generation;

static void Params_FormatSignedHalf(char *buffer, int16_t value_x2)
{
  uint8_t pos = 0U;
  int16_t magnitude;
  uint32_t integer_part;
  char temp[6];
  int8_t n = 0;

  if (value_x2 < 0)
  {
    buffer[pos++] = '-';
    magnitude = (int16_t)(-value_x2);
  }
  else
  {
    magnitude = value_x2;
  }

  /* value_x2 is in 0.5 C units: integer C = value/2, fraction is .0/.5. */
  integer_part = (uint32_t)magnitude / 2U;
  do
  {
    temp[n++] = (char)('0' + (integer_part % 10U));
    integer_part /= 10U;
  } while ((integer_part > 0U) && (n < (int8_t)sizeof(temp)));
  while (n > 0)
  {
    buffer[pos++] = temp[--n];
  }
  buffer[pos++] = '.';
  buffer[pos++] = ((magnitude & 1) != 0) ? '5' : '0';
  buffer[pos] = '\0';
}

void Params_FormatThreshold(char *buffer, int16_t value_x2)
{
  Params_FormatSignedHalf(buffer, value_x2);
}

const char *Params_SrcName(ParamSrc_t src)
{
  static const char *const names[3] = {"KEY", "ENC", "UART"};
  if ((uint8_t)src > (uint8_t)SRC_UART)
  {
    return "KEY";
  }
  return names[src];
}

const char *Params_ItemName(ParamId_t id)
{
  static const char *const names[3] = {"H", "L", "I"};
  if ((uint8_t)id > (uint8_t)PARAM_INTERVAL)
  {
    return "H";
  }
  return names[id];
}

static uint8_t relation_ok(int16_t high_x2, int16_t low_x2)
{
  return (high_x2 >= (int16_t)(low_x2 + 1)) ? 1U : 0U;
}

void Params_EnterOrCycle(void)
{
  if (osMutexWait(dataMutexHandle, MUTEX_WAIT_MS) == osOK)
  {
    if (s_params.editing == 0U)
    {
      /* First press enters edit mode and stays on the current (default H)
         item; following presses cycle H -> L -> I -> H. */
      s_params.editing = 1U;
    }

    else
    {
      s_params.selected =
          (ParamId_t)(((uint8_t)s_params.selected + 1U) % 3U);
    }
    (void)osMutexRelease(dataMutexHandle);
  }
}

void Params_ExitEdit(void)
{
  if (osMutexWait(dataMutexHandle, MUTEX_WAIT_MS) == osOK)
  {
    s_params.editing = 0U;
    (void)osMutexRelease(dataMutexHandle);
  }
}

uint8_t Params_Step(ParamSrc_t src, int8_t dir)
{
  uint8_t changed = 0U;

  if (dir == 0)
  {
    return 0U;
  }

  if (osMutexWait(dataMutexHandle, MUTEX_WAIT_MS) != osOK)
  {
    return 0U;
  }

  if (s_params.selected == PARAM_INTERVAL)
  {
    int32_t next = (int32_t)s_params.interval_s +
                   (dir > 0 ? INTERVAL_STEP_S : -INTERVAL_STEP_S);
    if (next < INTERVAL_MIN) { next = INTERVAL_MIN; }
    if (next > INTERVAL_MAX) { next = INTERVAL_MAX; }
    if (next != (int32_t)s_params.interval_s)
    {
      s_params.interval_s = (int16_t)next;
      changed = 1U;
    }
  }
  else
  {
    int16_t *target = (s_params.selected == PARAM_HIGH) ?
                      &s_params.high_x2 : &s_params.low_x2;
    int32_t lo = (s_params.selected == PARAM_HIGH) ?
                 HIGH_X2_MIN : LOW_X2_MIN;
    int32_t hi = (s_params.selected == PARAM_HIGH) ?
                 HIGH_X2_MAX : LOW_X2_MAX;
    int32_t next = (int32_t)(*target) +
                   (dir > 0 ? THRESHOLD_STEP_X2 : -THRESHOLD_STEP_X2);
    if (next < lo) { next = lo; }
    if (next > hi) { next = hi; }
    {
      int16_t candidate_high = (s_params.selected == PARAM_HIGH) ?
                               (int16_t)next : s_params.high_x2;
      int16_t candidate_low = (s_params.selected == PARAM_LOW) ?
                              (int16_t)next : s_params.low_x2;
      if ((next != (int32_t)(*target)) &&
          (relation_ok(candidate_high, candidate_low) != 0U))
      {
        *target = (int16_t)next;
        changed = 1U;
      }
    }
  }

  if (changed != 0U)
  {
    s_params.src = src;
    s_generation++;
  }
  (void)osMutexRelease(dataMutexHandle);
  return changed;
}

uint8_t Params_SetThreshold(ParamId_t id, int16_t value_x2, ParamSrc_t src)
{
  int32_t lo;
  int32_t hi;
  int16_t *target;
  uint8_t ok = 0U;

  if (id == PARAM_HIGH)
  {
    lo = HIGH_X2_MIN;
    hi = HIGH_X2_MAX;
    target = &s_params.high_x2;
  }
  else if (id == PARAM_LOW)
  {
    lo = LOW_X2_MIN;
    hi = LOW_X2_MAX;
    target = &s_params.low_x2;
  }
  else
  {
    return 0U;
  }

  if (osMutexWait(dataMutexHandle, MUTEX_WAIT_MS) != osOK)
  {
    return 0U;
  }

  if (((int32_t)value_x2 >= lo) && ((int32_t)value_x2 <= hi))
  {
    int16_t high = (id == PARAM_HIGH) ? value_x2 : s_params.high_x2;
    int16_t low = (id == PARAM_LOW) ? value_x2 : s_params.low_x2;
    if (relation_ok(high, low) != 0U)
    {
      *target = value_x2;
      s_params.src = src;
      s_generation++;
      ok = 1U;
    }
  }

  (void)osMutexRelease(dataMutexHandle);
  return ok;
}

uint8_t Params_SetInterval(int16_t seconds, ParamSrc_t src)
{
  uint8_t ok = 0U;

  if (osMutexWait(dataMutexHandle, MUTEX_WAIT_MS) != osOK)
  {
    return 0U;
  }

  if ((seconds >= INTERVAL_MIN) && (seconds <= INTERVAL_MAX) &&
      ((seconds % INTERVAL_STEP_S) == 0))
  {
    s_params.interval_s = seconds;
    s_params.src = src;
    s_generation++;
    ok = 1U;
  }

  (void)osMutexRelease(dataMutexHandle);
  return ok;
}

uint8_t Params_SetAll(int16_t high_x2, int16_t low_x2, int16_t interval_s,
                      ParamSrc_t src)
{
  uint8_t ok = 0U;
  if (osMutexWait(dataMutexHandle, MUTEX_WAIT_MS) != osOK)
  {
    return 0U;
  }
  if ((high_x2 >= HIGH_X2_MIN) && (high_x2 <= HIGH_X2_MAX) &&
      (low_x2 >= LOW_X2_MIN) && (low_x2 <= LOW_X2_MAX) &&
      relation_ok(high_x2, low_x2) != 0U &&
      (interval_s >= INTERVAL_MIN) && (interval_s <= INTERVAL_MAX) &&
       ((interval_s % INTERVAL_STEP_S) == 0))
  {
    s_params.high_x2 = high_x2;
    s_params.low_x2 = low_x2;
    s_params.interval_s = interval_s;
    s_params.src = src;
    s_generation++;
    ok = 1U;
  }
  (void)osMutexRelease(dataMutexHandle);
  return ok;
}

void Params_ApplyDefaults(void)
{
  (void)Params_SetAll(60, 40, 20, SRC_UART);
}

uint32_t Params_GetGeneration(void)
{
  uint32_t generation = 0U;
  if (osMutexWait(dataMutexHandle, MUTEX_WAIT_MS) == osOK)
  {
    generation = s_generation;
    (void)osMutexRelease(dataMutexHandle);
  }
  return generation;
}

uint8_t Params_IsEditing(void)
{
  uint8_t editing = 0U;
  if (osMutexWait(dataMutexHandle, MUTEX_WAIT_MS) == osOK)
  {
    editing = s_params.editing;
    (void)osMutexRelease(dataMutexHandle);
  }
  return editing;
}

void Params_GetSnapshot(ParamSnapshot_t *snapshot)
{
  if (snapshot == NULL)
  {
    return;
  }

  /* Callers must never observe uninitialized thresholds if the mutex times out. */
  snapshot->high_x2 = 60;
  snapshot->low_x2 = 40;
  snapshot->interval_s = 20;
  snapshot->selected = PARAM_HIGH;
  snapshot->editing = 0U;
  snapshot->src = SRC_KEY;

  if (osMutexWait(dataMutexHandle, MUTEX_WAIT_MS) == osOK)
  {
    *snapshot = s_params;
    (void)osMutexRelease(dataMutexHandle);
  }
}
