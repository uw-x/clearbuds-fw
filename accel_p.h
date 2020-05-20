#define CHIP_ID_REG                          0x00
#define CHIP_ID                              0x90

#define ERROR_REG                            0x02
#define STATUS_REG                           0x03
  typedef union {
    struct {
      unsigned interruptActive: 1;
      unsigned powerMode: 2; // 0: sleep, 1: low power, 2: normal, 3: unused
      unsigned reserved2: 1;
      unsigned commandReady: 1;
      unsigned reserved: 2;
      unsigned dataReady: 1;
    };
    uint8_t bits;
  } accelStatus_t;

#define ACC_X_LSB_REG                         0x04
#define ACC_X_MSB_REG                         0x05
#define ACC_Y_LSB_REG                         0x06
#define ACC_Y_MSB_REG                         0x07
#define ACC_Z_LSB_REG                         0x08
#define ACC_Z_MSB_REG                         0x09
#define EVENT_REG                             0x0D
#define INT_STAT0_REG                         0x0E
#define INT_STAT1_REG                         0x0F
#define INT_STAT2_REG                         0x10
#define ACC_CONFIG0_REG                       0x19
#define ACC_CONFIG0_POWER_MODE_SLEEP          (0 << 0)
#define ACC_CONFIG0_POWER_MODE_LOW_POWER      (1 << 0)
#define ACC_CONFIG0_POWER_MODE_NORMAL         (2 << 0)

#define ACC_CONFIG1_REG                       0x1A
#define ACC_CONFIG2_REG                       0x1B
#define INT_CONFIG0_REG                       0x1F
#define INT_CONFIG0_GEN1_ENABLE               (1 << 2)

#define INT_CONFIG1_REG                       0x20
#define INT_CONFIG1_LATCHED                   (1 << 0)
#define INT_CONFIG1_UNLATCHED                 (0 << 0)

#define INT1_MAP_REG                          0x21
#define INT2_MAP_REG                          0x22
#define INT1_MAP_GEN1                         (1 << 2)

#define INT2_MAP_REG                          0x22
#define INT12_MAP_REG                         0x23
#define INT12_IO_CTRL_REG                     0x24
#define INT12_IO_CTRL_INT1_ACTIVE_LOW         (1 << 0)
#define INT12_IO_CTRL_INT1_ACTIVE_HIGH        (1 << 1)
#define INT12_IO_CTRL_INT2_ACTIVE_LOW         (1 << 5)
#define INT12_IO_CTRL_INT2_ACTIVE_HIGH        (1 << 5)

#define AUTOLOWPOW_0_REG                      0x2A
#define AUTOLOWPOW_1_REG                      0x2B
#define AUTOWAKEUP_0_REG                      0x2C
#define AUTOWAKEUP_1_REG                      0x2D
#define WKUP_INT_CONFIG0_REG                  0x2F
#define WKUP_INT_CONFIG1_REG                  0x30
#define WKUP_INT_CONFIG2_REG                  0x31
#define WKUP_INT_CONFIG3_REG                  0x32
#define WKUP_INT_CONFIG4_REG                  0x33

#define GEN1INT_CONFIG0_REG                   0x3F
#define GEN1INT_CONFIG0_ACT_Z_EN              (1 << 7)
#define GEN1INT_CONFIG0_ACT_Y_EN              (1 << 6)
#define GEN1INT_CONFIG0_ACT_X_EN              (1 << 5)
#define GEN1INT_CONFIG0_DATA_SRC_ACC_FILT2    (1 << 4) // 0: acc_filt1, 1:acc_filt2 (need filt2 for interrupts)
#define GEN1INT_CONFIG0_DATA_SRC_ACC_FILT1    (0 << 4) // 0: acc_filt1, 1:acc_filt2 (need filt2 for interrupts)
#define GEN1INT_CONFIG0_ACT_REFU_EVERYTIME_LP (3 << 2)
#define GEN1INT_CONFIG0_ACT_REFU_EVERYTIME    (2 << 2)
#define GEN1INT_CONFIG0_ACT_REFU_ONETIME      (1 << 2)
#define GEN1INT_CONFIG0_ACT_REFU_MANUAL       (0 << 2)
#define GEN1INT_CONFIG0_ACT_HYST_96MG         (3 << 0)
#define GEN1INT_CONFIG0_ACT_HYST_48MG         (2 << 0)
#define GEN1INT_CONFIG0_ACT_HYST_24MG         (1 << 0)
#define GEN1INT_CONFIG0_ACT_HYST_NONE         (0 << 0)

#define GEN1INT_CONFIG1_REG                   0x40
#define GEN1INT_CONFIG1_ACTIVITY_DETECT       (1 << 1)
#define GEN1INT_CONFIG1_INACTIVITY_DETECT     (0 << 1)
#define GEN1INT_CONFIG1_COMB_SEL_AND          (1 << 0)
#define GEN1INT_CONFIG1_COMB_SEL_OR           (0 << 0)

#define GEN1INT_CONFIG2_REG                   0x41
#define GEN1INT_CONFIG3_REG                   0x42
#define GEN1INT_CONFIG31_REG                  0x43
#define GEN1INT_CONFIG4_REG                   0x44
#define GEN1INT_CONFIG5_REG                   0x45
#define GEN1INT_CONFIG6_REG                   0x46
#define GEN1INT_CONFIG7_REG                   0x47
#define GEN1INT_CONFIG8_REG                   0x48
#define GEN1INT_CONFIG9_REG                   0x49
#define GEN2INT_CONFIG0_REG                   0x4A
#define ACTIVITY_CHANGE_CONFIG0_REG           0x55
#define ACTIVITY_CHANGE_CONFIG1_REG           0x56