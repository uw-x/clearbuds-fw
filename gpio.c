#include <stdbool.h>

#include "main.h"
#include "event.h"
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "boards.h"
#include "gpio.h"

void gpioInit(void) {
  ret_code_t err_code;

  err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);

  // nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

  // err_code = nrf_drv_gpiote_out_init(PIN_OUT, &out_config);
  // APP_ERROR_CHECK(err_code);
}