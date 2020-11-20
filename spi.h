#define SPI_BUS  3

void spiTransfer(uint8_t spiBus, uint8_t* data, uint8_t length);
void spiInit(void);
void spiDeInit(void);
