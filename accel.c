#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "gpio.h"
#include "spi.h"
#include "accel.h"
#include "accel_p.h"
#include "event.h"

accelStatus_t accelStatus;

static void accelWrite(uint8_t reg, uint8_t data)
{
  uint8_t bytes[2] = {reg, data};
  spiTransfer(SPI_BUS_3, bytes, 2);
}

static uint8_t accelRead(uint8_t reg)
{
  uint8_t bytes[3] = {0x80 | reg, 0x0, 0x0};
  spiTransfer(SPI_BUS_3, bytes, 3);
  return bytes[2];
}

static void accelInterruptHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  bool motion = gpioRead(ACCEL_INT1_PIN);
  if (motion) { eventQueuePush(EVENT_ACCEL_MOTION); }
}

// int1vs2, interrupt source, x,y,z, activity/inactivity, or/and, threshold, duration
void accelGenericInterruptEnable(accelGenericInterrupt_t* accelGenericInterrupt)
{
  // map source to pin
  uint8_t interruptPinReg = (accelGenericInterrupt->pin == ACCEL_INT1) ? INT1_MAP_REG : INT2_MAP_REG;
  accelWrite(interruptPinReg, accelGenericInterrupt->source);

  // unlatch interrupts
  accelWrite(INT_CONFIG1_REG, INT_CONFIG1_UNLATCHED);

  // active high
  accelWrite(INT12_IO_CTRL_REG, INT12_IO_CTRL_INT1_ACTIVE_HIGH | INT12_IO_CTRL_INT2_ACTIVE_HIGH);

  // configure axes, refu, hysteresis, activity/inactivity, or/and
  uint8_t genericInterruptBaseReg = (accelGenericInterrupt->source == ACCEL_INT_SOURCE_GENERIC1) ? GEN1INT_CONFIG0_REG : GEN2INT_CONFIG0_REG;

  accelWrite(genericInterruptBaseReg,
            (accelGenericInterrupt->xEnable ? GEN1INT_CONFIG0_ACT_X_EN : 0) |
            (accelGenericInterrupt->yEnable ? GEN1INT_CONFIG0_ACT_Y_EN : 0) |
            (accelGenericInterrupt->zEnable ? GEN1INT_CONFIG0_ACT_Z_EN : 0) |
            (GEN1INT_CONFIG0_DATA_SRC_ACC_FILT2) |
            (GEN1INT_CONFIG0_ACT_REFU_ONETIME) |
            (GEN1INT_CONFIG0_ACT_HYST_NONE));

  accelWrite(genericInterruptBaseReg + 1,
            (accelGenericInterrupt->activity ? GEN1INT_CONFIG1_ACTIVITY_DETECT : GEN1INT_CONFIG1_INACTIVITY_DETECT) |
            (accelGenericInterrupt->combSelectIsAnd ? GEN1INT_CONFIG1_COMB_SEL_AND : GEN1INT_CONFIG1_COMB_SEL_OR));

  // threshold
  accelWrite(genericInterruptBaseReg + 2, accelGenericInterrupt->threshold);

  // duration
  accelWrite(genericInterruptBaseReg + 3, (accelGenericInterrupt->duration & 0xFF00) >> 8);
  accelWrite(genericInterruptBaseReg + 4, (accelGenericInterrupt->duration & 0xFF));

  // enable
  uint8_t sourcesEnabled = accelRead(INT_CONFIG0_REG);
  sourcesEnabled |= accelGenericInterrupt->source;
  accelWrite(INT_CONFIG0_REG, sourcesEnabled);
}

void accelScratchpad(void)
{
  accelWrite(INT1_MAP_REG, INT1_MAP_GEN1); // Generic Interrupt 1 on INT1
  accelWrite(INT_CONFIG1_REG, INT_CONFIG1_UNLATCHED); // Unlatch interrupt
  accelWrite(INT12_IO_CTRL_REG, INT12_IO_CTRL_INT1_ACTIVE_HIGH); // INT1 Active High

  accelWrite(GEN1INT_CONFIG0_REG,
    GEN1INT_CONFIG0_ACT_Z_EN |
    GEN1INT_CONFIG0_DATA_SRC_ACC_FILT2 |
    GEN1INT_CONFIG0_ACT_REFU_ONETIME |
    GEN1INT_CONFIG0_ACT_HYST_NONE);

  accelWrite(GEN1INT_CONFIG1_REG, GEN1INT_CONFIG1_ACTIVITY_DETECT | GEN1INT_CONFIG1_COMB_SEL_OR);

  accelWrite(GEN1INT_CONFIG2_REG, 0x02);  // threshold in 8mg/lsb
  accelWrite(GEN1INT_CONFIG3_REG, 0x00);  // msb duration in samples (100hz)
  accelWrite(GEN1INT_CONFIG31_REG, 0x07); // lsb duration in samples (100hz)
  accelWrite(INT_CONFIG0_REG, INT_CONFIG0_GEN1_ENABLE);
}

void accelDumpRegisters(uint8_t start, uint8_t end)
{
  for (uint8_t i = start; i < end; i++) {
    NRF_LOG_RAW_INFO("[accel] %02X: %02X\n", i, accelRead(i));
  }
}

uint16_t accelGetX(void)
{
  return (accelRead(ACC_X_MSB_REG) << 8) | accelRead(ACC_X_LSB_REG);
}

uint16_t accelGetY(void)
{
  return (accelRead(ACC_Y_MSB_REG) << 8) | accelRead(ACC_Y_LSB_REG);
}

uint16_t accelGetZ(void)
{
  return (accelRead(ACC_Z_MSB_REG) << 8) | accelRead(ACC_Z_LSB_REG);
}

void accelInit(void)
{
  uint8_t version = accelRead(CHIP_ID_REG);
  if (version == CHIP_ID)
  {
    // Normal power mode
    accelWrite(ACC_CONFIG0_REG, ACC_CONFIG0_POWER_MODE_NORMAL);
    accelStatus.bits = accelRead(STATUS_REG);
    NRF_LOG_RAW_INFO("[accel] powerMode:%d\n", accelStatus.powerMode);
    NRF_LOG_RAW_INFO("[accel] commandReady:%d\n", accelStatus.commandReady);
    NRF_LOG_RAW_INFO("[accel] dataReady:%d\n", accelStatus.dataReady);

    // Configure interrupts
    gpioInput_t gpioInput = GPIO_INTERRUPT_CONFIG_RISING;
    gpioInput.pull = NRF_GPIO_PIN_NOPULL;
    gpioInputEnable(ACCEL_INT1_PIN, &gpioInput, accelInterruptHandler);
    gpioInterruptEnable(ACCEL_INT1_PIN);

    NRF_LOG_RAW_INFO("[accel] initialized\n");
  } else {
    NRF_LOG_RAW_INFO("[accel] error initializing\n");
  }
}
