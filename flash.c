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
#include "flash_p.h"

static bool transferDone = false;

static void qspiHandler(nrf_drv_qspi_evt_t event, void * p_context)
{
  UNUSED_PARAMETER(event);
  UNUSED_PARAMETER(p_context);
  transferDone = true;
}

void flashErase(uint32_t address)
{
  transferDone = false;
  APP_ERROR_CHECK(nrf_drv_qspi_erase(NRF_QSPI_ERASE_LEN_64KB, address));
  while(!transferDone) { __WFE(); };
  NRF_LOG_RAW_INFO("[flash] erasing...\n");
}

void flashWrite(uint32_t address, uint8_t* data, uint16_t length)
{
  transferDone = false;
  APP_ERROR_CHECK(nrf_drv_qspi_write(data, length, address));
  while(!transferDone) { __WFE(); };
  NRF_LOG_RAW_INFO("[flash] writing...\n");
}

void flashRead(uint32_t address, uint8_t* data, uint16_t length)
{
  transferDone = false;
  APP_ERROR_CHECK(nrf_drv_qspi_read(data, length, address));
  while(!transferDone) { __WFE(); };
  NRF_LOG_RAW_INFO("[flash] reading...\n");
}

void flashInit()
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

void flashDeInit(void)
{
  nrf_drv_qspi_uninit();
}