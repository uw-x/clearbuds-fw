#define SPI_BUS_1  1
#define SPI_BUS_3  3

void spiTransfer(uint8_t spiBus, uint8_t* data, uint8_t length);
void spiInit(void);
