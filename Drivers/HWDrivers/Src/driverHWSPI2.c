#include "driverHWSPI2.h"

SPI_HandleTypeDef driverHWSPI2Handle;

void driverHWSPI2Init(void) {
    /* SPI2 parameter configuration*/
    driverHWSPI2Handle.Instance = SPI2;
    driverHWSPI2Handle.Init.Mode = SPI_MODE_MASTER;
    driverHWSPI2Handle.Init.Direction = SPI_DIRECTION_2LINES;
    driverHWSPI2Handle.Init.DataSize = SPI_DATASIZE_8BIT;
    driverHWSPI2Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
    driverHWSPI2Handle.Init.CLKPhase = SPI_PHASE_1EDGE;
    driverHWSPI2Handle.Init.NSS = SPI_NSS_SOFT;
    driverHWSPI2Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    driverHWSPI2Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    driverHWSPI2Handle.Init.TIMode = SPI_TIMODE_DISABLE;
    driverHWSPI2Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    driverHWSPI2Handle.Init.CRCPolynomial = 7;
    driverHWSPI2Handle.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    driverHWSPI2Handle.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&driverHWSPI2Handle) != HAL_OK)
    {
        while(true);
    }
  	
	/*Init CS GPIO config */
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(SDCARD_CS_GPIO_PORT,SDCARD_CS_GPIO_PIN,GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = SDCARD_CS_GPIO_PIN;
  	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  	GPIO_InitStruct.Pull = GPIO_NOPULL;
  	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  	HAL_GPIO_Init(SDCARD_CS_GPIO_PORT, &GPIO_InitStruct);

};

bool driverHWSPI2Write(uint8_t *writeBuffer, uint8_t noOfBytesToWrite, GPIO_TypeDef* GPIOCSPort, uint16_t GPIO_CSPin) {
	uint8_t *readBuffer;																																					// Make fake buffer holder
	HAL_StatusTypeDef halReturnStatus;																														// Make holder for HAL state
	readBuffer = malloc(noOfBytesToWrite);																												// Make fake buffer for
	
	HAL_GPIO_WritePin(GPIOCSPort,GPIO_CSPin,GPIO_PIN_RESET);																      // Make CS low
	halReturnStatus = HAL_SPI_TransmitReceive(&driverHWSPI2Handle,writeBuffer,readBuffer,noOfBytesToWrite,driverHWSPI2DefaultTimeout);	// Write desired data to slave and store the received data in readBuffer
	while( driverHWSPI2Handle.State == HAL_SPI_STATE_BUSY );  																		// Wait until transmission is complete
	HAL_GPIO_WritePin(GPIOCSPort,GPIO_CSPin,GPIO_PIN_SET);																	      // Make CS High
	
	free(readBuffer);																																							// Dump de fake buffer
	
	return (halReturnStatus == HAL_OK);																														// Return true if all went OK
};

bool driverHWSPI2WriteRead(uint8_t *writeBuffer, uint8_t noOfBytesToWrite, uint8_t *readBuffer, uint8_t noOfBytesToRead, GPIO_TypeDef* GPIOCSPort, uint16_t GPIO_CSPin) {
	uint8_t *writeArray, *readArray;
	HAL_StatusTypeDef halReturnStatus;																														// Make holder for HAL state
	
	writeArray = malloc(sizeof(uint8_t)*(noOfBytesToWrite+noOfBytesToRead));
	readArray = malloc(sizeof(uint8_t)*(noOfBytesToWrite+noOfBytesToRead));	
	
	memset(writeArray,0xFF,noOfBytesToWrite+noOfBytesToRead);
	memcpy(writeArray,writeBuffer,noOfBytesToWrite);
	
	HAL_GPIO_WritePin(GPIOCSPort,GPIO_CSPin,GPIO_PIN_RESET);
	halReturnStatus = HAL_SPI_TransmitReceive(&driverHWSPI2Handle,writeArray,readArray,noOfBytesToWrite+noOfBytesToRead,driverHWSPI2DefaultTimeout);
	while( driverHWSPI2Handle.State == HAL_SPI_STATE_BUSY );  // wait xmission complete
	HAL_GPIO_WritePin(GPIOCSPort,GPIO_CSPin,GPIO_PIN_SET);
	
	memcpy(readBuffer,readArray+noOfBytesToWrite,noOfBytesToRead);
		
	free(writeArray);
	free(readArray);
	
	return (halReturnStatus == HAL_OK);																														// Return true if all went OK
};




