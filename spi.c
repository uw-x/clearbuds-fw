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

static const nrfx_spim_t spiInstance = NRFX_SPIM_INSTANCE(SPI_BUS);

static volatile bool transferDone;
static uint8_t spiRxBuffer[10];

nrfx_spim_config_t spiConfig = {
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

void spiEventHandler(nrfx_spim_evt_t const* p_event, void* p_context)
{
  transferDone = true;
}

void spiTransfer(uint8_t spiBus, uint8_t* data, uint8_t length)
{
  nrfx_spim_xfer_desc_t transferDescriptor = \
    NRFX_SPIM_XFER_TRX(data, length, spiRxBuffer, length);

  memset(spiRxBuffer, 0, length);

  transferDone = false;
  APP_ERROR_CHECK(nrfx_spim_xfer(&spiInstance, &transferDescriptor, 0));
  while(!transferDone) { __WFE(); };
  memcpy(data, spiRxBuffer, length);
}

void spiInit(void)
{
  APP_ERROR_CHECK(nrfx_spim_init(&spiInstance, &spiConfig, spiEventHandler, NULL));
}

void spiDeInit(void)
{
  gpioDisable(SPI_SCK_PIN);
  gpioDisable(SPI_MOSI_PIN);
  gpioDisable(SPI_MISO_PIN);
  gpioDisable(SPI_CS_PIN);
  nrfx_spim_uninit(&spiInstance);
}