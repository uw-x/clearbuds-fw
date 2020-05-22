// spi
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <nrfx.h>
#include "nrf_delay.h"
#include "nrfx_spim.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_clock.h"
#include "nrf_pwr_mgmt.h"

#include "gpio.h"
#include "spi.h"

static const nrfx_spim_t spiInstance1 = NRFX_SPIM_INSTANCE(SPI_BUS_1);
static const nrfx_spim_t spiInstance3 = NRFX_SPIM_INSTANCE(SPI_BUS_3);

static volatile bool transferDone1;
static volatile bool transferDone3;
static uint8_t spiRxBuffer[10];

nrfx_spim_config_t spiConfig1 = {
  .sck_pin        = FLASH_SCK_PIN,
  .mosi_pin       = FLASH_MOSI_PIN,
  .miso_pin       = FLASH_MISO_PIN,
  .ss_pin         = FLASH_CS_PIN,
  .ss_active_high = false,
  .irq_priority   = NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY,
  .orc            = 0xFF,
  .frequency      = NRF_SPIM_FREQ_8M,
  .mode           = NRF_SPIM_MODE_0,
  .bit_order      = NRF_SPIM_BIT_ORDER_MSB_FIRST,
};

nrfx_spim_config_t spiConfig3 = {
  .sck_pin        = SPI_SCK_PIN,
  .mosi_pin       = SPI_MOSI_PIN,
  .miso_pin       = SPI_MISO_PIN,
  .ss_pin         = SPI_CS_PIN,
  .ss_active_high = false,
  .irq_priority   = NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY,
  .orc            = 0xFF,
  .frequency      = NRF_SPIM_FREQ_8M,
  .mode           = NRF_SPIM_MODE_0,
  .bit_order      = NRF_SPIM_BIT_ORDER_MSB_FIRST,
};

void spiEventHandler1(nrfx_spim_evt_t const* p_event, void* p_context)
{
  transferDone1 = true;
}

void spiEventHandler3(nrfx_spim_evt_t const* p_event, void* p_context)
{
  transferDone3 = true;
}

void spiTransfer(uint8_t spiBus, uint8_t* data, uint8_t length)
{
  nrfx_spim_xfer_desc_t transferDescriptor = \
    NRFX_SPIM_XFER_TRX(data, length, spiRxBuffer, length);

  memset(spiRxBuffer, 0, length);

  switch(spiBus) {
    case SPI_BUS_1:
      transferDone1 = false;
      APP_ERROR_CHECK(nrfx_spim_xfer(&spiInstance1, &transferDescriptor, 0));
      while(!transferDone1) { __WFE(); };
      break;
    case SPI_BUS_3:
      transferDone3 = false;
      APP_ERROR_CHECK(nrfx_spim_xfer(&spiInstance3, &transferDescriptor, 0));
      while(!transferDone3) { __WFE(); };
      break;
    default:
      break;
  }

  memcpy(data, spiRxBuffer, length);
}

void spiInit(void)
{
  APP_ERROR_CHECK(nrfx_spim_init(&spiInstance1, &spiConfig1, spiEventHandler1, NULL));
  APP_ERROR_CHECK(nrfx_spim_init(&spiInstance3, &spiConfig3, spiEventHandler3, NULL));
}