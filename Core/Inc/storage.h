#ifndef __STORAGE_H__
#define __STORAGE_H__

#include <stdint.h>

#define STORAGE_LOG_CAPACITY 16U

typedef enum
{
  LOG_HIGH = 1,
  LOG_LOW = 2,
  LOG_SENSOR_ERROR = 3,
  LOG_VIBRATION = 4
} LogType_t;

typedef struct
{
  uint32_t sequence;
  uint32_t uptime_s;
  int16_t temperature_x10;
  uint8_t type;
  uint8_t acknowledged;
  uint16_t crc;
} AlarmLog_t;

typedef struct
{
  uint32_t magic;
  uint16_t version;
  int16_t high_x2;
  int16_t low_x2;
  uint16_t interval_s;
  uint32_t sequence;
  uint16_t crc;
} PersistParams_t;

/* Returns 1 for CRC/version-valid persisted parameters; 0 for defaults. */
uint8_t Storage_Load(PersistParams_t *params, AlarmLog_t *logs, uint8_t *count);
uint8_t Storage_SaveParams(const PersistParams_t *params);
uint8_t Storage_SaveLog(const AlarmLog_t *log, const AlarmLog_t *logs, uint8_t count);
uint8_t Storage_ClearLogs(void);
uint16_t Storage_Crc16(const void *data, uint16_t length);

#endif
