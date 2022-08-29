#include "stm32f3xx_hal.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"

//CS port and pin for SPI2 is also used and defined in fatfs.c and fatfs_sd.c
#define SDCARD_CS_GPIO_PORT                                             GPIOB
#define SDCARD_CS_GPIO_PIN                                              GPIO_PIN_12
#define driverHWSPI2DefaultTimeout										100

void driverHWSPI2Init(void);
bool driverHWSPI2Write(uint8_t *writeBuffer, uint8_t noOfBytesToWrite,GPIO_TypeDef* GPIOCSPort, uint16_t GPIO_CSPin);
bool driverHWSPI2WriteRead(uint8_t *writeBuffer, uint8_t noOfBytesToWrite, uint8_t *readBuffer, uint8_t noOfBytesToRead,GPIO_TypeDef* GPIOCSPort, uint16_t GPIO_CSPin);