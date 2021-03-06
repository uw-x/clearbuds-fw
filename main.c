/*
 * shio
 * maruchi kim
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <nrfx.h>

#include "app_util_platform.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_clock.h"
#include "draw.h"
#include "ble_manager.h"
#include "timers.h"
#include "audio.h"
#include "event.h"
#include "gpio.h"
#include "accel.h"
#include "spi.h"
#include "flash.h"
#include "main.h"

#include "time_sync.h"
#include "nrf_ppi.h"
#include "nrf_timer.h"

APP_TIMER_DEF(resetTimer);

static int16_t micData[PDM_DECIMATION_BUFFER_LENGTH];
static bool bleRetry = false;
static bool bleMicStreamRequested = false;
static uint32_t expectedBufferCount = 0;

static uint8_t metadataIndex = 0;
static uint8_t metadata[180] = { 0 };

accelGenericInterrupt_t accelInterrupt1 = {
  .pin = ACCEL_INT1,
  .source = ACCEL_INT_SOURCE_GENERIC1,
  .xEnable = false,
  .yEnable = false,
  .zEnable = true,
  .activity = true,
  .combSelectIsAnd = false,
  .threshold = 0x3,
  .duration = 0x7,
};

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
  app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void resetTimerCallback(void * p_context)
{
  NVIC_SystemReset();
}

void powerEnterSleepMode(void)
{
  ret_code_t err_code;

  NRF_LOG_RAW_INFO("%08d [power] powering off...\n", systemTimeGetMs());

  err_code = bsp_indication_set(BSP_INDICATE_IDLE);
  APP_ERROR_CHECK(err_code);

  // Drive enable signals low before shutting down
  gpioOutputEnable(MIC_EN_PIN);
  gpioWrite(MIC_EN_PIN, 0);
  gpioOutputEnable(ACCEL_EN_PIN);
  gpioWrite(ACCEL_EN_PIN, 0);
  gpioOutputEnable(FLASH_EN_PIN);
  gpioWrite(FLASH_EN_PIN, 0);

  spiDeInit();
  delayMs(1);

  // Prepare wakeup buttons.
  err_code = bsp_btn_ble_sleep_mode_prepare();
  APP_ERROR_CHECK(err_code);

  // Go to system-off mode (this function will not return; wakeup will cause a reset).
  err_code = sd_power_system_off();
  APP_ERROR_CHECK(err_code);
}

static void bsp_event_handler(bsp_event_t event)
{
  ret_code_t err_code;

  switch (event)
  {
    case BSP_EVENT_SLEEP:
      powerEnterSleepMode();
      break;

    case BSP_EVENT_DISCONNECT:
      err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      if (err_code != NRF_ERROR_INVALID_STATE) { APP_ERROR_CHECK(err_code); }
      break;

    case BSP_EVENT_KEY_0:
      break;

    case BSP_EVENT_KEY_2:
      eventQueuePush(EVENT_TIMESYNC_MASTER_ENABLE);
      break;

    case BSP_EVENT_KEY_3:
      eventQueuePush(EVENT_AUDIO_STREAM_START);
      break;

    default:
      break;
  }
}

static void buttons_leds_init(void)
{
  ret_code_t err_code;

  err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
  APP_ERROR_CHECK(err_code);

  // Configure power off
  bsp_event_to_button_action_assign(USER_BUTTON, BSP_BUTTON_ACTION_RELEASE, BSP_EVENT_SLEEP);
}

static void logInit(void)
{
  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void timeSyncInit(void)
{
  uint32_t       err_code;
  uint8_t        rf_address[5] = {0xDE, 0xAD, 0xBE, 0xEF, 0x19};
  ts_params_t    ts_params;

  nrf_gpiote_task_configure(3, NRF_GPIO_PIN_MAP(1, 14), NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
  nrf_gpiote_task_enable(3);

  nrf_ppi_channel_endpoint_setup(
    NRF_PPI_CHANNEL0,
    (uint32_t) nrf_timer_event_address_get(NRF_TIMER3, NRF_TIMER_EVENT_COMPARE4),
    nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_OUT_3));

  nrf_ppi_channel_enable(NRF_PPI_CHANNEL0);

  nrf_ppi_channel_endpoint_setup(
    NRF_PPI_CHANNEL5,
    (uint32_t) nrf_timer_event_address_get(NRF_TIMER3, NRF_TIMER_EVENT_COMPARE4),
    audioGetPdmStartTaskAddress());

  ts_params.high_freq_timer[0] = NRF_TIMER3;
  ts_params.high_freq_timer[1] = NRF_TIMER2;
  ts_params.rtc             = NRF_RTC1;
  ts_params.egu             = NRF_EGU3;
  ts_params.egu_irq_type    = SWI3_EGU3_IRQn;
  ts_params.ppi_chg         = 0;
  ts_params.ppi_chns[0]     = 1;
  ts_params.ppi_chns[1]     = 2;
  ts_params.ppi_chns[2]     = 3;
  ts_params.ppi_chns[3]     = 4;
  ts_params.rf_chn          = 125; /* For testing purposes */
  memcpy(ts_params.rf_addr, rf_address, sizeof(rf_address));

  err_code = ts_init(&ts_params);
  APP_ERROR_CHECK(err_code);

  err_code = ts_enable();
  APP_ERROR_CHECK(err_code);
}

static void powerInit(void)
{
  ret_code_t err_code;
  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
  sd_power_dcdc_mode_set(true);
}

static void idle(void)
{
  if (NRF_LOG_PROCESS() == false && eventQueueEmpty()) {
    nrf_pwr_mgmt_run();
  }
}

static void shioInit(void)
{
  bool erase_bonds;

  logInit();
  NRF_LOG_RAW_INFO("%08d [shio] booting...\n", systemTimeGetMs());

  timersInit();
  ret_code_t err_code;
  err_code = app_timer_create(&resetTimer, APP_TIMER_MODE_SINGLE_SHOT, resetTimerCallback);
  APP_ERROR_CHECK(err_code);

  gpioInit();

  eventQueueInit();
  buttons_leds_init();

  audioInit();

  spiInit();
  // accelInit();
  // accelGenericInterruptEnable(&accelInterrupt1);

  APP_ERROR_CHECK(nrf_drv_clock_init());
  powerInit();
  gpioOutputEnable(FLASH_EN_PIN);
  gpioWrite(FLASH_EN_PIN, 0);

  bleInit();
  timeSyncInit();
  bleAdvertisingStart();

  NRF_LOG_RAW_INFO("%08d [shio] booted\n", systemTimeGetMs());
}

static void processQueue(void)
{
  if (!eventQueueEmpty()) {
    switch(eventQueueFront()) {
      case EVENT_ACCEL_MOTION:
        NRF_LOG_RAW_INFO("%08d [accel] motion\n", systemTimeGetMs());
        break;

      case EVENT_ACCEL_STATIC:
        break;

      case EVENT_AUDIO_STREAM_START:
      {
        // DEPRECATED: Before there was an explicit characteristic to start the mics synchronously,
        // and then start the BLE notifications separately. After some thought, it would be much
        // simpler to tie the mic startup with the BLE notification startup which will alleviate
        // the need to launch another part of code synchronously.
        // NRF_TIMER3->TASKS_CAPTURE[3] = 1;
        // uint32_t timer3 = NRF_TIMER3->CC[3];
        // NRF_LOG_RAW_INFO("%08d [audio] PPI ENABLE %u %u syncTime:%u\n", systemTimeGetMs(), NRF_TIMER3->CC[0], timer3, ts_timestamp_get_ticks_u64(6));
        // nrf_ppi_channel_enable(NRF_PPI_CHANNEL5);
        break;
      }

      case EVENT_AUDIO_STREAM_STOP:
      {
        NRF_LOG_RAW_INFO("%08d [audio] stream stop\n", systemTimeGetMs());
        bleMicStreamRequested = false;
        audioStop();

        uint32_t timestamp = systemTimeGetMs();
        metadata[metadataIndex++ % 180]  = (timestamp >> 24) & 0xFF;
        metadata[metadataIndex++ % 180] = (timestamp >> 16) & 0xFF;
        metadata[metadataIndex++ % 180] = (timestamp >> 8) & 0xFF;
        metadata[metadataIndex++ % 180] = timestamp & 0xFF;

        metadata[metadataIndex++ % 180] = 0x01;
        metadata[metadataIndex++ % 180] = 0x02;
        metadata[metadataIndex++ % 180] = 0x03;
        metadata[metadataIndex++ % 180] = 0x04;

        bleSendData(metadata, 180);
        break;
      }

      case EVENT_AUDIO_MIC_DATA_READY:
      {
        memcpy(micData, audioGetMicData(), sizeof(int16_t) * PDM_DECIMATION_BUFFER_LENGTH);

        // Sometimes a PDM buffer gets thrown into the ether?
        uint32_t actualBufferCount = audioGetPdmBufferCount();
        if (++expectedBufferCount != actualBufferCount) {
          NRF_LOG_RAW_INFO("%08d [audio] expected(%d) != actual(%d)\n", systemTimeGetMs(), expectedBufferCount, actualBufferCount);
          for (int i = 0; i < (actualBufferCount - expectedBufferCount); i++) { blePushSequenceNumber(); }
          expectedBufferCount = actualBufferCount;
        }

        // PDM started via programmable peripheral interconnect (PPI)
        // Disable PPI so that PDM doesn't restart, and set audioStreamStarted to true
        if (!audioStreamStarted()) {
          nrf_ppi_channel_disable(NRF_PPI_CHANNEL5);
          audioSetStreamStarted(true);
        }

        if (audioStreamStarted() && bleMicStreamRequested) {
          if (bleBufferHasSpace(sizeof(int16_t) * PDM_DECIMATION_BUFFER_LENGTH)) {
            bleSendData((uint8_t *) micData, sizeof(int16_t) * PDM_DECIMATION_BUFFER_LENGTH);
          } else {
            // No space, drop this data
            NRF_LOG_RAW_INFO("%08d [ble] dropped 270 samples\n", systemTimeGetMs());
            blePushSequenceNumber();
          }
        }

        break;
      }

      case EVENT_BLE_DATA_STREAM_START:
        NRF_LOG_RAW_INFO("%08d [ble] stream start\n", systemTimeGetMs());

        NRF_TIMER3->TASKS_CAPTURE[3] = 1;
        uint32_t timer3 = NRF_TIMER3->CC[3];
        nrf_ppi_channel_enable(NRF_PPI_CHANNEL5);
        NRF_LOG_RAW_INFO("%08d [main] PPI ENABLE %u\n", systemTimeGetMs(), timer3);

        metadata[0] = (timer3 >> 24) & 0xFF;
        metadata[1] = (timer3 >> 16) & 0xFF;
        metadata[2] = (timer3 >> 8) & 0xFF;
        metadata[3] = timer3 & 0xFF;

        bleSendData(metadata, 180);

        bleMicStreamRequested = true;
        gpioWrite(GPIO_1_PIN, 1);
        break;

      case EVENT_BLE_DATA_STREAM_STOP:
      {
        NVIC_SystemReset();
        // app_timer_start(resetTimer, APP_TIMER_TICKS(2000), NULL);
        break;
      }

      case EVENT_BLE_RADIO_START:
        // Event that fires whenever the radio starts up
        break;

      case EVENT_BLE_SEND_DATA_DONE:
        // BLE just finished, attempt to fill in more data
        send();
        break;

      case EVENT_BLE_IDLE:
        powerEnterSleepMode();
        break;

      case EVENT_BLE_DISCONNECTED:
        NVIC_SystemReset();
        break;

      case EVENT_TIMESYNC_MASTER_ENABLE:
        NRF_LOG_RAW_INFO("%08d [main] time sync master enabled\n", systemTimeGetMs());
        ts_tx_start(200);
        break;

      case EVENT_TIMESYNC_SLAVE_ENABLE:
        NRF_LOG_RAW_INFO("%08d [main] time sync slave enabled\n", systemTimeGetMs());
        ts_tx_stop();
        break;

      case EVENT_TIMESYNC_PACKET_RECEIVED:
        audioUpdateTicksAhead();
        break;

      case EVENT_TIMERS_ONE_SECOND_ELAPSED:
        break;

      case EVENT_METADATA_SAVE_TIMESTAMP:
      {
        uint32_t timestamp = systemTimeGetMs();
        metadata[metadataIndex++ % 180]  = (timestamp >> 24) & 0xFF;
        metadata[metadataIndex++ % 180] = (timestamp >> 16) & 0xFF;
        metadata[metadataIndex++ % 180] = (timestamp >> 8) & 0xFF;
        metadata[metadataIndex++ % 180] = timestamp & 0xFF;
        break;
      }

      default:
        NRF_LOG_RAW_INFO("%08d [main] unhandled event:%d\n", systemTimeGetMs(), eventQueueFront());
        break;
    }

    eventQueuePop();
  }
}

int main(void)
{
  shioInit();

  for (;;)
  {
    idle();
    processQueue();
  }
}
