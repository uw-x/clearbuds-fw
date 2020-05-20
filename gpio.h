// bsp
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"

// PDM
#define PDM_CLK_PIN                      NRF_GPIO_PIN_MAP(1,8)
#define PDM_DATA_PIN                     NRF_GPIO_PIN_MAP(1,9)

// SPI NEW
#define SPI_SCK_PIN                      NRF_GPIO_PIN_MAP(0,4)
#define SPI_MOSI_PIN                     NRF_GPIO_PIN_MAP(0,26)
#define SPI_MISO_PIN                     NRF_GPIO_PIN_MAP(0,27)
#define SPI_SS_PIN                       NRF_GPIO_PIN_MAP(0,28)

// ACCEL
#define ACCEL_INT1_PIN                   NRF_GPIO_PIN_MAP(1, 5)
#define ACCEL_INT2_PIN                   NRF_GPIO_PIN_MAP(1, 6)

// Wrapper
#define GPIO_INTERRUPT_CONFIG_RISING  GPIOTE_CONFIG_IN_SENSE_LOTOHI(true)
#define GPIO_INTERRUPT_CONFIG_FALLING GPIOTE_CONFIG_IN_SENSE_HITOLO(true)
#define GPIO_INTERRUPT_CONFIG_TOGGLE  GPIOTE_CONFIG_IN_SENSE_TOGGLE(true)

#define gpioPin_t                            nrfx_gpiote_pin_t
#define gpioOutput_t                         nrf_drv_gpiote_out_config_t
#define gpioInput_t                          nrf_drv_gpiote_in_config_t
#define gpioOutputEnable(pin, config)          nrf_drv_gpiote_out_init(pin, config)
#define gpioInputEnable(pin, config, handler)  nrf_drv_gpiote_in_init(pin, config, handler)
#define gpioInterruptEnable(pin)             nrf_drv_gpiote_in_event_enable(pin, true)
#define gpioInterruptDisable(pin)            nrf_drv_gpiote_in_event_disable(pin)
#define gpioRead(pin)                        nrf_gpio_pin_read(pin)

void gpioInit(void);