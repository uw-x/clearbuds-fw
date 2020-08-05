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

#define SECONDS_TO_RECORD 3
#define MIC_TO_BLE
// #define MIC_TO_FLASH

static uint8_t flashReadBuffer[FLASH_READ_BUFFER_SIZE] = {0};
static int16_t micData[PDM_BUFFER_LENGTH];
static bool bleRetry = false;

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

void sleep_mode_enter(void)
{
  ret_code_t err_code;

  err_code = bsp_indication_set(BSP_INDICATE_IDLE);
  APP_ERROR_CHECK(err_code);

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
      sleep_mode_enter();
      break; // BSP_EVENT_SLEEP

    case BSP_EVENT_DISCONNECT:
      err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      if (err_code != NRF_ERROR_INVALID_STATE)
      {
        APP_ERROR_CHECK(err_code);
      }
      break; // BSP_EVENT_DISCONNECT

    case BSP_EVENT_WHITELIST_OFF:
      if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
      {
        err_code = ble_advertising_restart_without_whitelist(&m_advertising);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
          APP_ERROR_CHECK(err_code);
        }
      }
      break; // BSP_EVENT_KEY_0

    default:
      break;
  }
}

static void buttons_leds_init(bool * p_erase_bonds)
{
  ret_code_t err_code;
  bsp_event_t startup_event;

  err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
  APP_ERROR_CHECK(err_code);

  err_code = bsp_btn_ble_init(NULL, &startup_event);
  APP_ERROR_CHECK(err_code);

  *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

static void logInit(void)
{
  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void powerInit(void)
{
  ret_code_t err_code;
  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
}

static void idle(void)
{
  if (NRF_LOG_PROCESS() == false && eventQueueIsEmpty()) {
    nrf_pwr_mgmt_run();
  }
}

static void shioInit(void)
{
  bool erase_bonds;

  logInit();
  NRF_LOG_RAW_INFO("[shio] booting...\n");
  timersInit();
  gpioInit();
  gpioOutputEnable(GPIO_1_PIN);
  gpioWrite(GPIO_1_PIN, 0); // booting
  eventQueueInit();
  buttons_leds_init(&erase_bonds);
  flashInternalInit();

#ifdef MIC_TO_FLASH
  flashInternalErase(FLASH_INTERNAL_BASE_ADDRESS, (SECONDS_TO_RECORD*100000) / 4000); // Erase 125 4kB pages
#endif

  audioInit();
  spiInit();
  accelInit();
  accelGenericInterruptEnable(&accelInterrupt1);
  APP_ERROR_CHECK(nrf_drv_clock_init());
  powerInit();

#ifdef MIC_TO_BLE
  bleInit();
  bleAdvertisingStart();
#endif

  gpioWrite(GPIO_1_PIN, 1); // finished booting
  NRF_LOG_RAW_INFO("[shio] booted\n");
}

static void processQueue(void)
{
#ifdef MIC_TO_BLE
  static bool streamStarted = false;
#endif

  if (!eventQueueIsEmpty()) {
    switch(eventQueueFront()) {
      case EVENT_ACCEL_MOTION:
        NRF_LOG_RAW_INFO("%08d [accel] motion\n", systemTimeGetMs());
        break;

      case EVENT_ACCEL_STATIC:
        break;

      case EVENT_AUDIO_MIC_DATA_READY:
        memcpy(micData, audioGetMicData(), sizeof(int16_t) * PDM_BUFFER_LENGTH);

#ifdef MIC_TO_BLE
        if (streamStarted) {
          if (bleCanTransmit() && !bleRetry) {
            bleSendData((uint8_t *) micData, sizeof(int16_t) * PDM_BUFFER_LENGTH);
          } else {
            if (!bleRetry) {
              bleRetry = true;
            } else {
              NRF_LOG_RAW_INFO("%08d [ble] dropped packet\n", systemTimeGetMs());
            }
          }
        }
#endif

#ifdef MIC_TO_FLASH
        flashInternalWrite(
          (flashInternalGetNextWriteAddress()),
          (uint8_t*) micData,
          (2*PDM_BUFFER_LENGTH));

        if (flashInternalGetBytesWritten() > SECONDS_TO_RECORD*100000) {
          uint32_t readAddress = FLASH_INTERNAL_BASE_ADDRESS;
          audioDeInit();

          // Read 512 bytes, 1000 times = 512000KB dump
          for (int i = 0; i < 1000; i++) {
            readAddress = flashInternalRead(readAddress, flashReadBuffer, FLASH_READ_BUFFER_SIZE);
            for (int j = 0; j < 512; j+=2) {
              NRF_LOG_RAW_INFO("%d\n", (int16_t) (flashReadBuffer[j+1] << 8 | flashReadBuffer[j]));
            }
          }

          while(1) {};
        }
#endif
        break;

      case EVENT_BLE_DATA_STREAM_START:
#ifdef MIC_TO_BLE
        streamStarted = true;
        audioStart();
#endif
        NRF_LOG_RAW_INFO("%08d [ble] stream start\n", systemTimeGetMs());
        break;

      case EVENT_BLE_DATA_STREAM_STOP:
#ifdef MIC_TO_BLE
        streamStarted = false;
#endif
        NRF_LOG_RAW_INFO("%08d [ble] stream stop\n", systemTimeGetMs());
        break;

      case EVENT_BLE_RADIO_START:
        // Event that fires whenever the radio starts up
        break;

      case EVENT_BLE_SEND_DATA_DONE:
        if (bleRetry && bleCanTransmit()) {
          bleRetry = false;
          bleSendData((uint8_t *) micData, sizeof(int16_t) * PDM_BUFFER_LENGTH);
        }
        break;

      default:
        NRF_LOG_RAW_INFO("unhandled event\n");
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
