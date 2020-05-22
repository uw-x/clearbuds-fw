#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <nrfx.h>

#include "nrf_log.h"

#include "spi.h"
#include "flash_p.h"

uint32_t flashGetId(void) {
  uint8_t bytes[4] = {CMD_RDID, 0x0, 0x0, 0x0};
  spiTransfer(SPI_BUS_1, bytes, 4);
  return bytes[0] << 24 | bytes[1] << 16 | bytes[2] << 8 | bytes[3];
}

void flashInit(void)
{
  uint32_t flashId = flashGetId();
  if (flashId == FLASH_ID) {
    NRF_LOG_RAW_INFO("[flash] initialized\n");
  } else {
    NRF_LOG_RAW_INFO("[flash] error initializing, expected %08x actual %08x\n", FLASH_ID, flashId);
  }
}