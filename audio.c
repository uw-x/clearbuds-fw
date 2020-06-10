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
#include "audio.h"

int16_t pdmInputBuffer[2][PDM_INPUT_BUFFER_LENGTH] = {0};
int16_t releasedPdmBuffer[PDM_INPUT_BUFFER_LENGTH] = {0};
float32_t fftInputBuffer[FFT_INPUT_BUFFER_LENGTH];
float32_t fftOutputBuffer[FFT_OUTPUT_BUFFER_LENGTH];
static bool fftInputBufferReady = false;
static int pdmInputBufferIndex = 0;

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
    memcpy(releasedPdmBuffer, event->buffer_released, sizeof(int16_t) * PDM_INPUT_BUFFER_LENGTH);
    eventQueuePush(EVENT_AUDIO_MIC_DATA_READY);
    CRITICAL_REGION_EXIT();
  }

  if (event->buffer_requested) {
    pdmInputBufferIndex = (pdmInputBufferIndex == 0) ? 1 : 0;
    errorStatus = nrfx_pdm_buffer_set(pdmInputBuffer[pdmInputBufferIndex], PDM_INPUT_BUFFER_LENGTH);
    ASSERT(errorStatus == NRFX_SUCCESS);
  }

  return;
}

/**
 * @param[in] p_input        Pointer to input data array with complex number samples in time domain.
 * @param[in] p_input_struct Pointer to cfft instance structure describing input data.
 * @param[out] p_output      Pointer to processed data (bins) array in frequency domain.
 * @param[in] output_size    Processed data array size.
 */
void fft_process(float32_t *p_input, const arm_cfft_instance_f32 * p_input_struct, float32_t *p_output, uint16_t output_size)
{
  static uint32_t  m_ifft_flag             = 0;    //!< Flag that selects forward (0) or inverse (1) transform.
  static uint32_t  m_do_bit_reverse        = 1;    //!< Flag that enables (1) or disables (0) bit reversal of output.

  // Use CFFT module to process the data.
  arm_cfft_f32(p_input_struct, p_input, m_ifft_flag, m_do_bit_reverse);
  // Calculate the magnitude at each bin using Complex Magnitude Module function.
  arm_cmplx_mag_f32(p_input, p_output, output_size);
}

#ifdef FPU_INTERRUPT_MODE
/**
 * @brief FPU Interrupt handler. Clearing exception flag at the stack.
 *
 * Function clears exception flag in FPSCR register and at the stack. During interrupt handler
 * execution FPU registers might be copied to the stack (see lazy stacking option) and
 * it is necessary to clear data at the stack which will be recovered in the return from
 * interrupt handling.
 */
void FPU_IRQHandler(void)
{
  // Prepare pointer to stack address with pushed FPSCR register.
  uint32_t * fpscr = (uint32_t * )(FPU->FPCAR + FPU_FPSCR_REG_STACK_OFF);
  // Execute FPU instruction to activate lazy stacking.
  (void)__get_FPSCR();
  // Clear flags in stacked FPSCR register.
  *fpscr = *fpscr & ~(FPU_EXCEPTION_MASK);
}
#endif


void audioService(void)
{
  static uint32_t lastFftTimeMs = 0;

  if (fftInputBufferReady && (systemTimeGetMs() - lastFftTimeMs) > 1000) {
    // It is important to use proper arm_cfft_sR_f32 structure associated with input/output data length.
    //  - 256 numbers in input array (128 complex pairs of samples) -> 128 output bins power data -> &arm_cfft_sR_f32_len128.
    fft_process(fftInputBuffer,
                &arm_cfft_sR_f32_len1024,
                fftOutputBuffer,
                FFT_OUTPUT_BUFFER_LENGTH);

    draw_fft_data(fftOutputBuffer, FFT_OUTPUT_BUFFER_LENGTH/2, GRAPH_WINDOW_HEIGHT); // only draw half the spectrum

    lastFftTimeMs = systemTimeGetMs();
    fftInputBufferReady = false;
    memset(fftInputBuffer, 0, sizeof(float32_t) * FFT_INPUT_BUFFER_LENGTH);

    NRF_LOG_FLUSH();
#ifndef FPU_INTERRUPT_MODE
    __set_FPSCR(__get_FPSCR() & ~(FPU_EXCEPTION_MASK));
    (void) __get_FPSCR();
    NVIC_ClearPendingIRQ(FPU_IRQn);
#endif
  }
}

int16_t* audioGetMicData(void)
{
  return releasedPdmBuffer;
}

void audioInit(void)
{
#ifdef FPU_INTERRUPT_MODE
  // Enable FPU interrupt
  NVIC_SetPriority(FPU_IRQn, APP_IRQ_PRIORITY_LOWEST);
  NVIC_ClearPendingIRQ(FPU_IRQn);
  NVIC_EnableIRQ(FPU_IRQn);
#endif

  gpioOutputEnable(MIC_EN_PIN);
  gpioWrite(MIC_EN_PIN, 1);
  delayMs(1);

  nrfx_err_t errorStatus;
  nrfx_pdm_config_t pdmConfig = NRFX_PDM_DEFAULT_CONFIG(PDM_CLK_PIN, PDM_DATA_PIN);
  errorStatus = nrfx_pdm_init(&pdmConfig, (nrfx_pdm_event_handler_t) pdmEventHandler);
  ASSERT(errorStatus == NRFX_SUCCESS);

  nrfx_pdm_buffer_set(pdmInputBuffer[0], PDM_INPUT_BUFFER_LENGTH);
  ASSERT(errorStatus == NRFX_SUCCESS);

  errorStatus = nrfx_pdm_start();
  ASSERT(errorStatus == NRFX_SUCCESS);

  NRF_LOG_RAW_INFO("[audio] initialized\n");
}

void audioDeInit(void)
{
  nrfx_pdm_stop();
  nrfx_pdm_uninit();
}