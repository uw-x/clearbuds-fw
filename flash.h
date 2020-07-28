#include "spi.h"

#define FLASH_INTERNAL_BASE_ADDRESS 0x50000

void flashExternalEraseAll(void);
void flashExternalErase(uint32_t address);
void flashExternalWrite(uint32_t address, uint8_t* data, uint16_t length);
void flashExternalRead(uint32_t address, uint8_t* data, uint16_t length);

void flashExternalInit(void);
void flashExternalDeInit(void);

void flashInternalErase(uint32_t pageAddress, uint32_t pagesToErase);
void flashInternalWrite(uint32_t address, uint8_t* data, uint32_t length);
uint32_t flashInternalRead(uint32_t address, uint8_t* readData, uint32_t length);
void flashInternalInit(void);
uint32_t flashInternalGetNextWriteAddress(void);
uint32_t flashInternalGetBytesWritten(void);