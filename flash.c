#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <nrfx.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"

#include "nrf_log.h"
#include "nrf_drv_qspi.h"
#include "nrf_delay.h"
#include "sdk_config.h"

#include "spi.h"
#include "nrf_fstorage_sd.h"
#include "flash.h"
#include "flash_p.h"

static volatile bool internalTransferDone;
static volatile bool externalTransferDone;
static volatile uint32_t internalNextWriteAddress;

static void flashInternalHandler(nrf_fstorage_evt_t * p_evt);

NRF_FSTORAGE_DEF(nrf_fstorage_t flashInternalInstance) =
{
    .evt_handler    = flashInternalHandler,
    .start_addr     = FLASH_INTERNAL_BASE_ADDRESS,
    .end_addr       = FLASH_INTERNAL_BASE_ADDRESS,
};

static void flashInternalHandler(nrf_fstorage_evt_t * p_evt)
{
	if (p_evt->result != NRF_SUCCESS) {
		NRF_LOG_RAW_INFO("[flash] error while writing/reading\n");
		return;
	}

	switch (p_evt->id) {
    case NRF_FSTORAGE_EVT_ERASE_RESULT:
      internalTransferDone = true;
      break;

    case NRF_FSTORAGE_EVT_WRITE_RESULT:
      NRF_LOG_RAW_INFO("[flash] wrote %d bytes at address 0x%x.\n", p_evt->len, p_evt->addr);
      internalNextWriteAddress += p_evt->len;
      internalTransferDone = true;
      break;

    default:
      break;
	}
}

static uint32_t flashInternalGetEndAddress()
{
    uint32_t const bootloader_addr = BOOTLOADER_ADDRESS;
    uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;
    uint32_t const code_sz         = NRF_FICR->CODESIZE;

    return (bootloader_addr != 0xFFFFFFFF ?
            bootloader_addr : (code_sz * page_sz));
}

static void qspiHandler(nrf_drv_qspi_evt_t event, void * p_context)
{
  UNUSED_PARAMETER(event);
  UNUSED_PARAMETER(p_context);
  externalTransferDone = true;
}

void flashExternalEraseAll(void)
{
  externalTransferDone = false;
  APP_ERROR_CHECK(nrf_drv_qspi_erase(QSPI_ERASE_LEN_LEN_All, 0));
  while(!externalTransferDone) { __WFE(); };
  NRF_LOG_RAW_INFO("[flash] erasing...\n");
}

void flashExternalErase(uint32_t address)
{
  externalTransferDone = false;
  APP_ERROR_CHECK(nrf_drv_qspi_erase(NRF_QSPI_ERASE_LEN_64KB, address));
  while(!externalTransferDone) { __WFE(); };
  NRF_LOG_RAW_INFO("[flash] erasing 64kbits at 0x%08x...\n", address);
}

void flashExternalWrite(uint32_t address, uint8_t* data, uint16_t length)
{
  externalTransferDone = false;
  APP_ERROR_CHECK(nrf_drv_qspi_write(data, length, address));
  while(!externalTransferDone) { __WFE(); };
  NRF_LOG_RAW_INFO("[flash] writing %d bytes to 0x%08x...\n", length, address);
}

// length must be divisible by 4
void flashExternalRead(uint32_t address, uint8_t* data, uint16_t length)
{
  externalTransferDone = false;
  APP_ERROR_CHECK(nrf_drv_qspi_read(data, length, address));
  while(!externalTransferDone) { __WFE(); };
  // NRF_LOG_RAW_INFO("[flash] reading from 0x%08X...\n", address);
}

void flashExternalInit()
{
  nrf_drv_qspi_config_t config = NRF_DRV_QSPI_DEFAULT_CONFIG;
  APP_ERROR_CHECK(nrf_drv_qspi_init(&config, qspiHandler, NULL));

  nrf_qspi_cinstr_conf_t cinstr_cfg = {
    .opcode    = QSPI_STD_CMD_RSTEN,
    .length    = NRF_QSPI_CINSTR_LEN_1B,
    .io2_level = true,
    .io3_level = true,
    .wipwait   = true,
    .wren      = true
  };

  // Send reset enable
  APP_ERROR_CHECK(nrf_drv_qspi_cinstr_xfer(&cinstr_cfg, NULL, NULL));

  // Send reset command
  cinstr_cfg.opcode = QSPI_STD_CMD_RST;
  APP_ERROR_CHECK(nrf_drv_qspi_cinstr_xfer(&cinstr_cfg, NULL, NULL));

  // Switch to qspi mode
  uint8_t temporary = 0x40;
  cinstr_cfg.opcode = QSPI_STD_CMD_WRSR;
  cinstr_cfg.length = NRF_QSPI_CINSTR_LEN_2B;
  APP_ERROR_CHECK(nrf_drv_qspi_cinstr_xfer(&cinstr_cfg, &temporary, NULL));

  NRF_LOG_RAW_INFO("[flash] initialized\n");
}

void flashExternalDeInit(void)
{
  nrf_drv_qspi_uninit();
}

bool flashInternalBusy(void)
{
  return (nrf_fstorage_is_busy(&flashInternalInstance));
}

void flashInternalErase(uint32_t pageAddress, uint32_t pagesToErase)
{
  NRF_LOG_RAW_INFO("[flash] erasing %d pages from address 0x%x...\n", pagesToErase, pageAddress);
  internalTransferDone = false;
  nrf_fstorage_erase(&flashInternalInstance, pageAddress, pagesToErase, NULL);
  while(!internalTransferDone) { __WFE(); };
  NRF_LOG_RAW_INFO("[flash] erase complete");
}

void flashInternalWrite(uint32_t address, uint8_t* data, uint32_t length)
{
  if ((address < FLASH_INTERNAL_BASE_ADDRESS) ||
      ((address + length) >= flashInternalGetEndAddress())) {
    NRF_LOG_RAW_INFO("[flash] invalid flash address\n");
  }

  if (internalTransferDone) {
    ret_code_t rc = nrf_fstorage_write(&flashInternalInstance, address, data, length, NULL);
    if (rc != NRF_SUCCESS) { NRF_LOG_RAW_INFO("[flash] error while writing code:%d\n", rc); }
  } else {
    NRF_LOG_RAW_INFO("[flash] internal flash busy, dumping buffer\n");
  }
}

// return next address to read
uint32_t flashInternalRead(uint32_t address, uint8_t* readData, uint32_t length)
{
  nrf_fstorage_read(&flashInternalInstance, address, readData, length);
  return address + length;
}

void flashInternalInit(void)
{
  // nrf_fstorage_sd -> SoftDevice Backend
  flashInternalInstance.end_addr = flashInternalGetEndAddress();
  nrf_fstorage_init(&flashInternalInstance, &nrf_fstorage_sd, NULL);
  internalNextWriteAddress = FLASH_INTERNAL_BASE_ADDRESS;
  internalTransferDone = true;

  NRF_LOG_RAW_INFO("[flash] internal flash initialized\n");
  NRF_LOG_RAW_INFO("[flash] page size: %d bytes\n", NRF_FICR->CODEPAGESIZE);
  NRF_LOG_RAW_INFO("[flash] allocated size: %d bytes\n", flashInternalGetEndAddress() - FLASH_INTERNAL_BASE_ADDRESS);

}

uint32_t flashInternalGetNextWriteAddress(void)
{
  return internalNextWriteAddress;
}

uint32_t flashInternalGetBytesWritten(void)
{
  return (internalNextWriteAddress - FLASH_INTERNAL_BASE_ADDRESS);
}

void flashInternalDeInit(void)
{
  nrf_fstorage_uninit(&flashInternalInstance, NULL);
}

