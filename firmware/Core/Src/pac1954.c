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
	return 0x00;
}
uint8_t PAC1954_doRefreshV( uint8_t address){
	return I2C1_SendSingleByte(address,PAC1954_REFRESH_V);
}
uint8_t PAC1954_checkState(){
	return state;
}
