#include "storage.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stddef.h>

#define PARAM_FLASH_ADDR 0x0800F800UL
#define LOG_FLASH_ADDR   0x0800FC00UL
#define STORAGE_MAGIC    0x53455431UL

uint16_t Storage_Crc16(const void *data, uint16_t length)
{
  const uint8_t *bytes = (const uint8_t *)data;
  uint16_t crc = 0xFFFFU;
  uint16_t i;
  uint8_t bit;
  for (i = 0; i < length; i++)
  {
    crc ^= bytes[i];
    for (bit = 0; bit < 8U; bit++)
    {
      crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0xA001U) : (uint16_t)(crc >> 1);
    }
  }
  return crc;
}

static uint8_t flash_erase_page(uint32_t address)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t error = 0;
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = address;
  erase.NbPages = 1;
  return (HAL_FLASHEx_Erase(&erase, &error) == HAL_OK) ? 1U : 0U;
}

static uint8_t flash_write(const uint8_t *data, uint32_t address, uint16_t length)
{
  uint16_t i;
  uint16_t word;
  for (i = 0; i < length; i += 2U)
  {
    word = data[i];
    if ((i + 1U) < length) { word |= (uint16_t)data[i + 1U] << 8; }
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + i, word) != HAL_OK)
    {
      return 0U;
    }
  }
  return 1U;
}

uint8_t Storage_Load(PersistParams_t *params, AlarmLog_t *logs, uint8_t *count)
{
  const PersistParams_t *stored = (const PersistParams_t *)PARAM_FLASH_ADDR;
  const AlarmLog_t *stored_logs = (const AlarmLog_t *)LOG_FLASH_ADDR;
  uint8_t i;
  uint8_t valid = 0U;

  memset(params, 0, sizeof(*params));
  params->magic = STORAGE_MAGIC;
  params->version = 1U;
  params->high_x2 = 60;
  params->low_x2 = 40;
  params->interval_s = 20U;
  params->sequence = 0U;
  *count = 0U;

  if ((stored->magic == STORAGE_MAGIC) &&
      (stored->version == 1U) &&
      (Storage_Crc16(stored, (uint16_t)offsetof(PersistParams_t, crc)) == stored->crc))
  {
    *params = *stored;
    valid = 1U;
  }
  for (i = 0U; i < STORAGE_LOG_CAPACITY; i++)
  {
    if ((stored_logs[i].sequence != 0xFFFFFFFFUL) &&
        (stored_logs[i].sequence != 0U) &&
        (Storage_Crc16(&stored_logs[i], (uint16_t)offsetof(AlarmLog_t, crc)) ==
         stored_logs[i].crc))
    {
      if (*count < STORAGE_LOG_CAPACITY)
      {
        logs[*count] = stored_logs[i];
        (*count)++;
      }
    }
  }
  return valid;
}

uint8_t Storage_SaveParams(const PersistParams_t *params)
{
  PersistParams_t value = *params;
  value.magic = STORAGE_MAGIC;
  value.version = 1U;
  value.crc = Storage_Crc16(&value, (uint16_t)offsetof(PersistParams_t, crc));
  HAL_FLASH_Unlock();
  if (flash_erase_page(PARAM_FLASH_ADDR) == 0U ||
      flash_write((const uint8_t *)&value, PARAM_FLASH_ADDR, sizeof(value)) == 0U)
  {
    HAL_FLASH_Lock();
    return 0U;
  }
  HAL_FLASH_Lock();
  return 1U;
}

uint8_t Storage_SaveLog(const AlarmLog_t *log, const AlarmLog_t *logs, uint8_t count)
{
  AlarmLog_t ordered[STORAGE_LOG_CAPACITY];
  uint8_t i;
  (void)log; /* logs[] already includes the newest entry, even when full. */
  if (count > STORAGE_LOG_CAPACITY) { return 0U; }
  memset(ordered, 0xFF, sizeof(ordered));
  for (i = 0U; i < count; i++)
  {
    ordered[i] = logs[i];
    ordered[i].crc = Storage_Crc16(&ordered[i],
                                   (uint16_t)offsetof(AlarmLog_t, crc));
  }
  HAL_FLASH_Unlock();
  if (flash_erase_page(LOG_FLASH_ADDR) == 0U ||
      flash_write((const uint8_t *)ordered, LOG_FLASH_ADDR, sizeof(ordered)) == 0U)
  {
    HAL_FLASH_Lock();
    return 0U;
  }
  HAL_FLASH_Lock();
  return 1U;
}

uint8_t Storage_ClearLogs(void)
{
  uint8_t ok;
  HAL_FLASH_Unlock();
  ok = flash_erase_page(LOG_FLASH_ADDR);
  HAL_FLASH_Lock();
  return ok;
}
