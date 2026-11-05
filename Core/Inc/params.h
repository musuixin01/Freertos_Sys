/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    params.h
  * @brief   Unified shared parameter management for the multi-source
  *          parameter setting system (keys / rotary encoder / SSCOM UART).
  *
  *          All tasks MUST access the shared parameters only through this
  *          interface. Range checking, step adjustment, source tagging and
  *          dataMutex protection are all implemented here.
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __PARAMS_H__
#define __PARAMS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Selectable parameter items, cycling order is H -> L -> I -> H. */
typedef enum
{
  PARAM_HIGH = 0,     /* high temperature threshold */
  PARAM_LOW = 1,      /* low temperature threshold  */
  PARAM_INTERVAL = 2  /* record interval            */
} ParamId_t;

/* Origin of the last effective modification. */
typedef enum
{
  SRC_KEY = 0,        /* SW1..SW4 / board keys       */
  SRC_ENC = 1,        /* rotary encoder              */
  SRC_UART = 2        /* SSCOM commands over USART1  */
} ParamSrc_t;

/* Immutable snapshot copied out under the mutex for display / GET command. */
typedef struct
{
  int16_t     high_x2;      /* high threshold, unit 0.5 C (60 = 30.0 C) */
  int16_t     low_x2;       /* low threshold,  unit 0.5 C (20 = 10.0 C) */
  int16_t     interval_s;   /* record interval, unit second  (20 s)    */
  ParamId_t   selected;     /* currently selected item                 */
  uint8_t     editing;      /* 1 = edit mode ('>' shown on OLED)       */
  ParamSrc_t  src;          /* last effective modification source      */
} ParamSnapshot_t;

typedef struct
{
  int16_t high_x2;
  int16_t low_x2;
  int16_t interval_s;
} ParamValues_t;

/* SW1 / encoder key: enter edit mode and cycle H -> L -> I. */
void Params_EnterOrCycle(void);

/* SW4: leave edit mode (does not commit, changes are already effective). */
void Params_ExitEdit(void);

/* One step on the currently selected parameter (threshold 0.5 C / interval
   5 s), clamped at the range boundary. Returns 1 when the value changed. */
uint8_t Params_Step(ParamSrc_t src, int8_t dir);

/* Absolute set used by serial SET commands. Returns 0 when out of range
   (caller replies ERR RANGE), 1 on success. */
uint8_t Params_SetThreshold(ParamId_t id, int16_t value_x2, ParamSrc_t src);
uint8_t Params_SetInterval(int16_t seconds, ParamSrc_t src);
uint8_t Params_SetAll(int16_t high_x2, int16_t low_x2, int16_t interval_s,
                      ParamSrc_t src);
void    Params_ApplyDefaults(void);
uint32_t Params_GetGeneration(void);

/* Read accessors. */
uint8_t Params_IsEditing(void);
void    Params_GetSnapshot(ParamSnapshot_t *snapshot);

/* Formatting helpers shared by OLED and UART responses. */
const char *Params_SrcName(ParamSrc_t src);
const char *Params_ItemName(ParamId_t id);
/* threshold in 0.5 C units -> "-19.5" / "30.0", buffer needs >= 8 bytes */
void        Params_FormatThreshold(char *buffer, int16_t value_x2);

#ifdef __cplusplus
}
#endif

#endif /* __PARAMS_H__ */
