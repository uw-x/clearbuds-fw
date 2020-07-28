#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <nrfx.h>

#include "app_util_platform.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "arm_const_structs.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_gpio.h"

#include "nrf_pdm.h"
#include "nrfx_pdm.h"
#include "timers.h"
#include "draw.h"
#include "gpio.h"
#include "event.h"
#include "main.h"
#include "audio.h"

int16_t pdmBuffer[2][PDM_BUFFER_LENGTH] = {0};
int16_t releasedPdmBuffer[PDM_BUFFER_LENGTH] = {0};
static bool fftInputBufferReady = false;
static int pdmBufferIndex = 0;

static void pdmEventHandler(nrfx_pdm_evt_t *event)
{
  nrfx_err_t errorStatus;
  static bool pdmBufferSwitchFlag = false;

  if (event->error != NRFX_PDM_NO_ERROR) {
    NRF_LOG_RAW_INFO("[audio] pdm error\n");
    ASSERT(0);
  }

  if (event->buffer_released) {
    CRITICAL_REGION_ENTER();
    memcpy(releasedPdmBuffer, event->buffer_released, sizeof(int16_t) * PDM_BUFFER_LENGTH);
    eventQueuePush(EVENT_AUDIO_MIC_DATA_READY);
    CRITICAL_REGION_EXIT();
  }

  if (event->buffer_requested) {
    pdmBufferIndex = (pdmBufferIndex == 0) ? 1 : 0;
    errorStatus = nrfx_pdm_buffer_set(pdmBuffer[pdmBufferIndex], PDM_BUFFER_LENGTH);
    ASSERT(errorStatus == NRFX_SUCCESS);
  }

  return;
}

int16_t* audioGetMicData(void)
{
  return releasedPdmBuffer;
}

void audioInit(void)
{
  gpioOutputEnable(MIC_EN_PIN);
  gpioWrite(MIC_EN_PIN, 1);
  delayMs(1);

  NRF_LOG_RAW_INFO("[audio] initialized\n");
}

void audioStart(void)
{
  nrfx_err_t errorStatus;
  nrfx_pdm_config_t pdmConfig = NRFX_PDM_DEFAULT_CONFIG(PDM_CLK_PIN, PDM_DATA_PIN);
  errorStatus = nrfx_pdm_init(&pdmConfig, (nrfx_pdm_event_handler_t) pdmEventHandler);
  ASSERT(errorStatus == NRFX_SUCCESS);

  nrfx_pdm_buffer_set(pdmBuffer[0], PDM_BUFFER_LENGTH);
  ASSERT(errorStatus == NRFX_SUCCESS);

  errorStatus = nrfx_pdm_start();
  ASSERT(errorStatus == NRFX_SUCCESS);
}

void audioDeInit(void)
{
  nrfx_pdm_stop();
  nrfx_pdm_uninit();
}