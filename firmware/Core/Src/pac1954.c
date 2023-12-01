/*
 * pac1954.c
 *
 *  Created on: Nov 25, 2023
 *      Author: Michiel
 */

/* Includes ------------------------------------------------------------------*/
#include "pac1954.h"

uint8_t address=0x00;
uint8_t state=0x00;

/**
 * Try to find the address of the PAC based on a restricted range
 * return The found address or 0x00 if not found
 */
uint8_t PAC1954_findAddress(){
	uint8_t addr;
	for( addr=0x10;addr<0x20;addr++){
		if( I2C1_PokeDevice(addr) == I2C_OK ){
			address=addr;
			return addr;
		}
	}
	return 0x00;
}
/**
 * Read the current value of the voltage and current registers
 * address - the address of the PAC
 * lastVoltCur - the struct that holds the data
 * return The result of the I2C operation
 */
uint8_t PAC1954_readVoltageCurrent( uint8_t address, voltcur *lastVoltCur ){
	uint16_t recBuffer[9];
	uint8_t result = I2C1_Read16bitData( address, PAC1954_VBUSN_REG, 8,recBuffer);
	if( result == I2C_OK ){
		// The order of the sense pins doesn't match the sequence on the board...
		lastVoltCur->out3_voltage = recBuffer[0];
		lastVoltCur->out4_voltage = recBuffer[1];
		lastVoltCur->out1_voltage = recBuffer[2];
		lastVoltCur->out2_voltage = recBuffer[3];

		lastVoltCur->out3_current = recBuffer[4];
		lastVoltCur->out4_current = recBuffer[5];
		lastVoltCur->out1_current = recBuffer[6];
		lastVoltCur->out2_current = recBuffer[7];
	}
	return result;
}
/**
 * Read the content of the Accummulator count of a certain pac
 * address The address of the pac
 * returns The count or 0xFFFFFFFF if there was a failure
 */
uint32_t PAC1954_readAccCount( uint8_t address ){
	uint16_t recBuffer[2];
	if( I2C1_Read16bitData( address, PAC1954_ACC_CNT_REG , 2,recBuffer) == I2C_OK ){
		state=1;
		uint32_t count = recBuffer[0];
		count *= 0x10000;
		count += recBuffer[1];
		return count;
	}
	state=0x00;
	return 0xFFFFFFFF;
}
/**
 * Issue the refreshV command
 * address  the address of the pac
 */
uint8_t PAC1954_doRefreshV( uint8_t address){
	return I2C1_transmitByte(address,PAC1954_REFRESH_V);
}
/**
 * Request the current state of the PAC connection
 * return 0x00=bad or 0x01=good
 */
uint8_t PAC1954_checkState(){
	return state;
}
/* ********************************* L I M I T S ****************************************** */
uint8_t PAC1954_setOVLimit( uint8_t addr, uint8_t chn, uint16_t limit ){
	uint8_t buffer[4];
	buffer[0] = PAC1954_OV_LMTN_1_REG+(chn-1); // Address of the register
	buffer[1] = limit/0xFF;
	buffer[2] = limit%0x100;

	uint32_t alerts = PAC1954_read24bitRegister(address,PAC1954_ALERT_EN_REG); // Read alerts

	// Set the limit
	state=I2C1_transmitData(addr, 3, buffer); // Works
	// Enable the alert for it

	if( alerts == 0xFFFFFFFF ){
		return 0; // Failed
	}
	alerts |= (PAC_OV_ALERT_EN_CH1>>(chn-1)); // Alter it

	buffer[0] = PAC1954_ALERT_EN_REG;
	buffer[3]=alerts % 0x100;
	alerts/=0xFF;
	buffer[2]=alerts % 0x100;
	alerts/=0xFF;
	buffer[1]=alerts % 0x100;
	state=I2C1_transmitData(addr, 4, buffer); // Send it back?

	return state;
}
/**
 * Reads the content of an OV limit register, ch determines with one
 */
uint16_t PAC1954_readOVlimit( uint8_t address, uint8_t ch ){
	uint16_t recBuffer[2];
	state=0x00;
	if( I2C1_Read16bitData( address, PAC1954_OV_LMTN_1_REG+(ch-1) , 1,recBuffer) == I2C_OK ){
		state=0x01;
		return recBuffer[0];
	}

	return 0x00;
}

/* ********************************* A L E R T ******************************************** */
/**
 * Reads the current value of the alert status register return 0xFFFFFFFF if failed
 */
uint32_t PAC1954_readAlertStatus(uint8_t address){
	return PAC1954_read24bitRegister(address,PAC1954_ALERT_STS_REG);
}
/**
 * Reads the current value of the alert enable register return 0xFFFFFFFF if failed
 */
uint32_t PAC1954_readAlertEnable(uint8_t address){
	return PAC1954_read24bitRegister(address,PAC1954_ALERT_EN_REG);
}
/* ************************ U T I L I T Y ************************************************ */
/**
 * Method to read the content of a 24bit register of the pac, this writes to a 32bit uint.
 * return The result of the read or 0xFFFFFFFF (32bit FS)
 */
uint32_t PAC1954_read24bitRegister(uint8_t address,uint8_t reg){
	uint8_t recBuffer[3];
	state=0;
	uint8_t res = I2C1_Read8bitData( address, reg ,3,recBuffer);
	if( res == I2C_OK ){
		state=1;
		uint32_t status = recBuffer[0];
		status *= 0x10000;
		status += recBuffer[1];
		status *= 0x10000;
		status += recBuffer[2];
		return status;
	}
	return 0xFFFFFFFF;
}
/* **************************************************************************************** */
