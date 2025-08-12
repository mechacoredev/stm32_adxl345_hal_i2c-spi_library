/*
 * adxl345.c
 *
 *  Created on: Aug 12, 2025
 *      Author: Enes
 */

#include "adxl345.h"

static ADXL345_RETURN_STAT _adxl345readregister(ADXL345_t *dev,
		uint16_t MemAddress, uint8_t *pData, uint16_t Size) {
	if (HAL_I2C_Mem_Read(dev->i2c_handler, dev->i2c_add, MemAddress, 1, pData,
			Size, HAL_MAX_DELAY) == HAL_OK) {
		return ADXL345_OK;
	}
	return REG_READ_FAIL;
}

static ADXL345_RETURN_STAT _adxl345readregister_DMA(ADXL345_t *dev,
		uint16_t MemAddress, uint8_t *pData, uint16_t Size){
	if(HAL_I2C_Mem_Read_DMA(dev->i2c_handler, dev->i2c_add, MemAddress, 1, pData, Size)==HAL_OK){
		return ADXL345_OK;
	}
	return REG_READ_FAIL;
}

static ADXL345_RETURN_STAT _adxl345writeregister(ADXL345_t *dev,
		uint16_t MemAddress, uint8_t *pData, uint16_t Size) {
	if (HAL_I2C_Mem_Write(dev->i2c_handler, dev->i2c_add, MemAddress, 1, pData,
			Size, HAL_MAX_DELAY) == HAL_OK) {
		return ADXL345_OK;
	}
	return REG_READ_FAIL;
}

uint8_t ADXL345_AutoDetect(ADXL345_t *dev) {
	for(uint8_t add=100; add<256; add++){
		if(HAL_I2C_IsDeviceReady(dev->i2c_handler, add, 1, 1500)==HAL_OK){
			return add;
		}
	}
	return 0;
}

ADXL345_RETURN_STAT ADXL345_Init(ADXL345_t *dev, uint8_t addr) {
	if (HAL_I2C_IsDeviceReady(dev->i2c_handler, addr, 1, HAL_MAX_DELAY)
			!= HAL_OK) {
		return DEVICE_NOT_FOUND;
	}
	dev->i2c_add = addr;

	if (_adxl345readregister(dev, ADXL345_DEVID_ADDR, &(dev->who_am_i), 1)
			!= ADXL345_OK) {
		return REG_READ_FAIL;
	}
	if (dev->who_am_i != 0xE5) {
		return WHO_IS_THIS;
	}
	return ADXL345_OK;
}

ADXL345_RETURN_STAT ADXL345_Configuration(ADXL345_t *dev,
		ADXL345_CONFIG_t *config) {
	uint8_t databuffer;

	databuffer = 0x3F & ((config->ADXL345_POWERCNTRL.LINK << 5)
			| (config->ADXL345_POWERCNTRL.AUTO_SLEEP << 4)
			| (config->ADXL345_POWERCNTRL.MEASURE << 3)
			| (config->ADXL345_POWERCNTRL.SLEEP << 2)
			| (config->ADXL345_POWERCNTRL.WAKE_UP << 1));

	if (_adxl345writeregister(dev, ADXL345_POWER_CTL_ADDR, &databuffer, 1)
			!= ADXL345_OK) {
		return REG_WRITE_FAIL;
	}

	databuffer = 0xEF & ((config->ADXL345_DATAFRMT.SELF_TEST << 7)
			| (config->ADXL345_DATAFRMT.SPI << 6)
			| (config->ADXL345_DATAFRMT.INT_INVERT << 5)
			| (config->ADXL345_DATAFRMT.FULL_RES << 3)
			| (config->ADXL345_DATAFRMT.JUSTIFY << 2)
			| (config->ADXL345_DATAFRMT.RANGE));

	if (_adxl345writeregister(dev, ADXL345_DATA_FORMAT_ADDR, &databuffer, 1)
			!= ADXL345_OK) {
		return REG_WRITE_FAIL;
	}

	databuffer = 0x1F & ((config->ADXL345_BWRATE.LOW_PWR << 4)
			| (config->ADXL345_BWRATE.RATE));

	if (_adxl345writeregister(dev, ADXL345_BW_RATE_ADDR, &databuffer, 1)
			!= ADXL345_OK) {
		return REG_WRITE_FAIL;
	}

	databuffer = 124;

	if (_adxl345writeregister(dev, ADXL345_INT_MAP_ADDR, &databuffer, 1)
			!= ADXL345_OK) {
		return REG_WRITE_FAIL;
	}

	databuffer = (config->ADXL345_INT_ENABLE.DATA_READY<<7)|(config->ADXL345_INT_ENABLE.SINGLE_TAP<<6)|(config->ADXL345_INT_ENABLE.DOUBLE_TAP<<5)|(config->ADXL345_INT_ENABLE.ACTIVITY<<4)|(config->ADXL345_INT_ENABLE.INACTIVITY<<3)|(config->ADXL345_INT_ENABLE.FREE_FALL<<2)|(config->ADXL345_INT_ENABLE.WATERMARK<<1)|(config->ADXL345_INT_ENABLE.OVERRUN);

	if (_adxl345writeregister(dev, ADXL345_INT_ENABLE_ADDR, &databuffer, 1)
			!= ADXL345_OK) {
		return REG_WRITE_FAIL;
	}

	return ADXL345_OK;
}

void inline ADXL345_GetValue(ADXL345_t *dev) {
	dev->dataX =  (dev->rawdata[1] << 8) | (dev->rawdata[0]);
	dev->dataY =  (dev->rawdata[3] << 8) | (dev->rawdata[2]);
	dev->dataZ =  (dev->rawdata[5] << 8) | (dev->rawdata[4]);
}

ADXL345_RETURN_STAT ADXL345_Read_DMA(ADXL345_t *dev) {
	if (_adxl345readregister_DMA(dev, ADXL345_DATAX0, dev->rawdata, 6)
			== ADXL345_OK) {
		return ADXL345_OK;
	}
	return REG_READ_FAIL;
}

ADXL345_RETURN_STAT ADXL345_Read_Interrupt(ADXL345_t *dev) {
	if (_adxl345readregister(dev, ADXL345_INT_SOURCE_ADDR, &(dev->interruptstats), 1)
			== ADXL345_OK) {
		return ADXL345_OK;
	}
	return REG_READ_FAIL;
}

ADXL345_RETURN_STAT ADXL345_Read_Polling(ADXL345_t *dev) {
	if (_adxl345readregister(dev, ADXL345_DATAX0, dev->rawdata, 6)
			== ADXL345_OK) {
		return ADXL345_OK;
	}
	return REG_READ_FAIL;
}
