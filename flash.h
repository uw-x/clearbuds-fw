#include "spi.h"

#define flashTransfer(data, length) spiTransfer(data, length);

void flashInit(void);