#include "spi.h"

void flashErase(uint32_t address);
void flashWrite(uint32_t address, uint8_t* data, uint16_t length);
void flashRead(uint32_t address, uint8_t* data, uint16_t length);

void flashInit(void);
void flashDeInit(void);