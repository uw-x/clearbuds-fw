#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "gpio.h"

#define ACCEL_INT1 0
#define ACCEL_INT2 1

#define ACCEL_INT_SOURCE_WAKE_UP            (1 << 0)
#define ACCEL_INT_SOURCE_GENERIC1           (1 << 2)
#define ACCEL_INT_SOURCE_GENERIC2           (1 << 3)

typedef struct {
  uint8_t pin;
  uint8_t source;
  bool xEnable;
  bool yEnable;
  bool zEnable;
  bool activity;
  bool combSelectIsAnd;
  uint8_t threshold;
  uint16_t duration;
} accelGenericInterrupt_t;

void accelInit(void);
uint16_t accelGetX(void);
uint16_t accelGetY(void);
uint16_t accelGetZ(void);
void accelGenericInterruptEnable(accelGenericInterrupt_t*);
void accelScratchpad();
void accelDumpRegisters(uint8_t start, uint8_t end);