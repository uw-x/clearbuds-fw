// audio.h

void audioInit(void);
void audioStart(void);
void audioDeInit(void);
int16_t* audioGetMicData(void);
void decimate(int16_t *buffer, uint8_t dec_factor);