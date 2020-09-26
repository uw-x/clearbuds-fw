// audio.h

void audioInit(void);
void audioDeInit(void);
void audioStart(void);
void audioStop(void);
int16_t* audioGetMicData(void);
void audioUpdateTicksAhead(void);
bool audioStreamStarted(void);
void audioSetStreamStarted(bool);
uint32_t audioGetPdmStartTaskAddress(void);