// PDM
// FS: 50000 Hz

#define PDM_BUFFER_LENGTH          (512)                                  // This is the minimum size

// FPU
#define FPU_EXCEPTION_MASK               0x0000009F                             //!< FPU exception mask used to clear exceptions in FPSCR register.
#define FPU_FPSCR_REG_STACK_OFF          0x40                                   //!< Offset of FPSCR register stacked during interrupt handling in FPU part stack.

void audioInit(void);
void audioDeInit(void);
int16_t* audioGetMicData(void);