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

#include "nrf_fstorage_sd.h"

void fstorage_callback(nrf_fstorage_evt_t * p_evt);

#define FSTORAGE_BASE_ADDR 0x60000
#define FSTORAGE_END_ADDR 0x100000

static volatile uint32_t flash_addr = FSTORAGE_BASE_ADDR;
static volatile uint8_t fstorage_write_done = 1;

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage_instance) =
{
    .evt_handler    = fstorage_callback,
    .start_addr     = FSTORAGE_BASE_ADDR,
    .end_addr       = FSTORAGE_END_ADDR,
};

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

int16_t* micData;

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
  ret_code_t ret;
  bool erase_bonds;

  logInit();
  NRF_LOG_RAW_INFO("[shio] booting...\n");
  timersInit();
  gpioInit();
  eventQueueInit();
  buttons_leds_init(&erase_bonds);

  nrf_fstorage_init(
		&fstorage_instance, /* You fstorage instance, previously defined. */
		&nrf_fstorage_sd, /* Name of the backend. */
		NULL                /* Optional parameter, backend-dependant. */
	);

  nrf_fstorage_erase(&fstorage_instance, FSTORAGE_BASE_ADDR, 100, NULL);

  // flashInit();
  // for (int i = 0; i < 4; i++) {
  //   flashErase(i*0x10000);
  // }
  audioInit();
  // spiInit();
  // accelInit();
  // accelGenericInterruptEnable(&accelInterrupt1);

  ret = nrf_drv_clock_init();
  APP_ERROR_CHECK(ret);

  powerInit();
  // bleInit();
  // advertising_start(erase_bonds);

  NRF_LOG_RAW_INFO("[shio] booted\n");
}

uint32_t bytesWritten = 0;
uint8_t flashReadBuffer[1024] = {0};
int16_t* checkData;

void fstorage_callback(nrf_fstorage_evt_t * p_evt) {
	if (p_evt->result != NRF_SUCCESS) {
		NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
		return;
	}

	switch (p_evt->id) {
	case NRF_FSTORAGE_EVT_WRITE_RESULT:
		NRF_LOG_INFO("%d | --> Event received: wrote %d bytes at address 0x%x.", systemTimeGetMs(), p_evt->len, p_evt->addr);
		flash_addr += PDM_INPUT_BUFFER_LENGTH*2;
		fstorage_write_done = 1;
		break;

	default:
		break;
	}
}

static void processQueue(void)
{
  if (!eventQueueIsEmpty()) {
    switch(eventQueueFront()) {
      case EVENT_ACCEL_MOTION:
        NRF_LOG_RAW_INFO("%08d | motion\n", systemTimeGetMs());
        break;
      case EVENT_ACCEL_STATIC:
        break;
      case EVENT_AUDIO_MIC_DATA_READY:
        micData = audioGetMicData();

        // do we have space to write ?
        if (flash_addr + PDM_INPUT_BUFFER_LENGTH*2 < FSTORAGE_END_ADDR) {
          if (fstorage_write_done /*nrf_fstorage_is_busy(&fstorage_instance)*/) {
            // write to flash
            fstorage_write_done = 0;
            ret_code_t rc = nrf_fstorage_write(
              &fstorage_instance,     /* The instance to use. */
              flash_addr,             /* The address in flash where to store the data. */
              micData, /* A pointer to the data. */
              PDM_INPUT_BUFFER_LENGTH*2, /* Lenght of the data, in bytes. */
              NULL                    /* Optional parameter, backend-dependent. */
            );

            if (rc != NRF_SUCCESS)
              NRF_LOG_ERROR("nrf_fstorage_write() = %d", rc);
          } else NRF_LOG_ERROR("fstorage busy -> dumping buffer");
        } else {
          audioDeInit();
          flash_addr = FSTORAGE_BASE_ADDR;
          for (int i = 0; i < 250; i++) {
            nrf_fstorage_read(&fstorage_instance, flash_addr, flashReadBuffer, 1024);
            flash_addr += 1024;
            for (int j = 0; j < 1024; j+=2) {
              NRF_LOG_RAW_INFO("%d\n", (int16_t) (flashReadBuffer[j+1] << 8 | flashReadBuffer[j]));
            }
          }
          while (1) {};
        }



        // flashWrite(bytesWritten, (uint8_t *) micData, sizeof(micData[0])*PDM_INPUT_BUFFER_LENGTH);
        // bytesWritten += sizeof(micData[0])*PDM_INPUT_BUFFER_LENGTH;

        // if (bytesWritten > 300000) {
        //   audioDeInit();
        //   for (int i = 0; i < 300; i++) {
        //     flashRead(1024*i, flashReadBuffer, 1024);
        //     for (int j = 0; j < 1024; j+=2) {
        //       NRF_LOG_RAW_INFO("%d\n", (int16_t) (flashReadBuffer[j+1] << 8 | flashReadBuffer[j]));
        //     }
        //   }
        //   while(1) {};
        // }

        break;
      default:
        NRF_LOG_RAW_INFO("unhandled event");
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
    // audioService();
    processQueue();
  }
}