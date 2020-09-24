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
#include "time_sync.h"
#include "timers.h"
#include "draw.h"
#include "gpio.h"
#include "event.h"
#include "main.h"
#include "audio.h"

int16_t releasedPdmBuffer[PDM_DECIMATION_BUFFER_LENGTH] = {0};
int16_t pdmBuffer[2][PDM_BUFFER_LENGTH] = {0};
static bool fftInputBufferReady         = false;
static int pdmBufferIndex               = 0;
static int64_t samplesSkipped           = 0;
static int64_t ticksAhead               = 0;
static bool streamStarted = false;

static void decimate(int16_t* outputBuffer, int16_t* inputBuffer, uint8_t decimationFactor)
{
  for (int i = 0; i < PDM_DECIMATION_BUFFER_LENGTH; i++) {
    outputBuffer[i] = inputBuffer[i*decimationFactor];
  }
}

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
    gpioWrite(GPIO_3_PIN, 1);
    gpioWrite(GPIO_3_PIN, 0);
    // if ticksAhead > 320, zero order hold a sample
    // if ticksAhead < -320 skip a sample
    decimate(releasedPdmBuffer, event->buffer_released, PDM_DECIMATION_FACTOR);
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

void audioUpdateSamplesSkipped(void)
{
  // With an fs of 50kHz and fpdm of 3.2MHz, after 64 pdm clock edges a sample needs to be skipped
  // (64 clock cycles) / 3.2MHz = 20us
  // 20us on the 16MHz time sync clock is 320 ticks
  // Skip a 50khz sample after 320 ticks have accumulated

  if (!ts_master()) {
    uint32_t peerTimer   = ts_get_peer_timer();
    uint32_t localTimer  = ts_get_local_timer();
    uint32_t timerOffset = ts_get_timer_offset();

    if (localTimer > peerTimer) {
      ticksAhead += (int) timerOffset;
    } else {
      ticksAhead -= (int) timerOffset;
    }

    // NRF_LOG_RAW_INFO("%08d [audio] p:%u l:%u o:%u t:%d\n", systemTimeGetMs(), ts_get_peer_timer(), ts_get_local_timer(), ts_get_timer_offset(), ticksAhead);
  }
}

bool audioStreamStarted(void)
{
  return streamStarted;
}

void audioSetStreamStarted(bool started)
{
  streamStarted = started;
}

void audioInit(void)
{
  nrfx_err_t errorStatus;

  // Enable Mic
  gpioOutputEnable(MIC_EN_PIN);
  gpioWrite(MIC_EN_PIN, 1);
  delayMs(1);

  // Setup PDM
  nrfx_pdm_config_t pdmConfig = {
    .mode               = (nrf_pdm_mode_t)NRFX_PDM_CONFIG_MODE,
    .edge               = (nrf_pdm_edge_t)NRFX_PDM_CONFIG_EDGE,
    .pin_clk            = PDM_CLK_PIN,
    .pin_din            = PDM_DATA_PIN,
    .clock_freq         = (nrf_pdm_freq_t) 0x19000000, // DIV10: 0x19000000 -> CLK: 3.200 MHz -> SR: 50000 Hz
    .gain_l             = 0x3C, // 10dB gain
    .gain_r             = 0x3C, // 10dB gain
    .interrupt_priority = NRFX_PDM_CONFIG_IRQ_PRIORITY
  };

  errorStatus = nrfx_pdm_init(&pdmConfig, (nrfx_pdm_event_handler_t) pdmEventHandler);
  ASSERT(errorStatus == NRFX_SUCCESS);

  nrfx_pdm_buffer_set(pdmBuffer[0], PDM_BUFFER_LENGTH);
  ASSERT(errorStatus == NRFX_SUCCESS);

  NRF_LOG_RAW_INFO("%08d [audio] initialized\n", systemTimeGetMs());
}

void audioStart(void)
{
  if (!streamStarted) {
    nrfx_err_t errorStatus;
    streamStarted = true;
    NRF_LOG_RAW_INFO("%08d [audio] pdm start\n", systemTimeGetMs());

    errorStatus = nrfx_pdm_start();
    ASSERT(errorStatus == NRFX_SUCCESS);
  }
}

void audioStop(void)
{
  streamStarted = false;
  nrfx_pdm_stop();
}

void audioDeInit(void)
{
  nrfx_pdm_uninit();
  gpioWrite(MIC_EN_PIN, 0);
  NRF_LOG_RAW_INFO("%08d [audio] deinitialized\n", systemTimeGetMs());
}

uint32_t audioGetPdmStartTaskAddress(void)
{
  return nrfx_pdm_task_address_get(NRF_PDM_TASK_START);
}