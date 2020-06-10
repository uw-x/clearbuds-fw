// PDM
#define PDM_INPUT_BUFFER_LENGTH          (512)                                  // This is the minimum size
#define PDM_OUTPUT_BUFFER_LENGTH         (1024)                                 // Millisonic requires 1024 samples
#define FFT_INPUT_BUFFER_LENGTH          (4)                                    // (2*PDM_OUTPUT_BUFFER_LENGTH), Need to interleave zeros
#define FFT_OUTPUT_BUFFER_LENGTH         (FFT_INPUT_BUFFER_LENGTH / 2)

// FPU
#define FPU_EXCEPTION_MASK               0x0000009F                             //!< FPU exception mask used to clear exceptions in FPSCR register.
#define FPU_FPSCR_REG_STACK_OFF          0x40                                   //!< Offset of FPSCR register stacked during interrupt handling in FPU part stack.

void audioInit(void);
void audioDeInit(void);
int16_t* audioGetMicData(void);
void audioService(void);