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
uint8_t PAC1954_readVoltageCurrent( uint8_t address, VoltageCurrent *lastVoltCur ){
	uint16_t recBuffer[PAC_CHANNELS*2];
	uint8_t result = I2C1_Read16bitData( address, PAC1954_VBUSN_REG, PAC_CHANNELS*2,recBuffer);
	if( result == I2C_OK ){
		// The order of the sense pins doesn't match the sequence on the board...
		// This will be fixed client side.
		for( uint8_t a=0;a<4;a++){
			lastVoltCur[a].voltage = recBuffer[a];
			lastVoltCur[a].current = recBuffer[a+4];
		}
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
	return PAC1954_setLimit(addr,chn,limit,PAC1954_OV_LMTN_1_REG,PAC_OV_ALERT_EN_CH1);
}
uint8_t PAC1954_setUVLimit( uint8_t addr, uint8_t chn, uint16_t limit ){
	return PAC1954_setLimit(addr,chn,limit,PAC1954_UV_LMTN_1_REG,PAC_UV_ALERT_EN_CH1);
}
uint8_t PAC1954_setOCLimit( uint8_t addr, uint8_t chn, uint16_t limit ){
	return PAC1954_setLimit(addr,chn,limit,PAC1954_OC_LMTN_1_REG,PAC_OC_ALERT_EN_CH1);
}
uint8_t PAC1954_setUCLimit( uint8_t addr, uint8_t chn, uint16_t limit ){
	return PAC1954_setLimit(addr,chn,limit,PAC1954_UC_LMTN_1_REG,PAC_UC_ALERT_EN_CH1);
}
uint8_t PAC1954_setOPLimit( uint8_t addr, uint8_t chn, uint16_t limit ){
	return PAC1954_setLimit(addr,chn,limit,PAC1954_OP_LMTN_1_REG,PAC_OP_ALERT_EN_CH1);
}
uint8_t PAC1954_setLimit( uint8_t addr, uint8_t chn, uint16_t limit,uint8_t limitReg, uint32_t enableVal ){
	uint8_t buffer[4];
	buffer[0] = limitReg+(chn-1); // Address of the register
	buffer[1] = limit/0xFF;
	buffer[2] = limit%0x100;

	// Set the limit
	return I2C1_transmitData(addr, 3, buffer);
}
/**
 * Reads the content of an OV limit register, ch determines with one
 */
uint16_t PAC1954_readOVlimit( uint8_t address, uint8_t ch ){
	return PAC1954_readLimitReg(address,ch,PAC1954_OV_LMTN_1_REG);
}
uint16_t PAC1954_readUVlimit( uint8_t address, uint8_t ch ){
	return PAC1954_readLimitReg(address,ch,PAC1954_UV_LMTN_1_REG);
}
uint16_t PAC1954_readUClimit( uint8_t address, uint8_t ch ){
	return PAC1954_readLimitReg(address,ch,PAC1954_UC_LMTN_1_REG);
}
uint16_t PAC1954_readOClimit( uint8_t address, uint8_t ch ){
	return PAC1954_readLimitReg(address,ch,PAC1954_OC_LMTN_1_REG);
}
uint16_t PAC1954_readOPlimit( uint8_t address, uint8_t ch ){
	return PAC1954_readLimitReg(address,ch,PAC1954_OP_LMTN_1_REG);
}
uint16_t PAC1954_readLimitReg( uint8_t address, uint8_t ch, uint8_t reg ){
	uint16_t recBuffer[2];
	state=0x00;
	if( I2C1_Read16bitData( address, reg+(ch-1) , 1,recBuffer) == I2C_OK ){
		state=0x01;
		return recBuffer[0];
	}
	return 0x00;
}
/* ********************************* S E T T I N G S ****************************************** */
void PAC1954_applySettings( PacSettings * settings, PacChannel * channels, uint8_t cnt){
	PAC1954_clearAlertEnable(address); // First disable all alerts
	if( settings->alerts_enable == 0x00 ){ // Nothing enabled, so just return
		return;
	}
	/* Now set all alerts that are enabled, so set the limits */
	for( int a=0;a<cnt;a++ ){
		if( settings->alerts_enable & (PAC_OV_ALERT_EN_CH1 >> a) )
			PAC1954_setOVLimit( address,a+1,channels[a].OV_lim );
		if( settings->alerts_enable & (PAC_OV_ALERT_EN_CH1 >> a) )
			PAC1954_setUVLimit( address,a+1,channels[a].UV_lim );
		if( settings->alerts_enable & (PAC_OV_ALERT_EN_CH1 >> a) )
			PAC1954_setOCLimit( address,a+1,channels[a].OC_lim );
		if( settings->alerts_enable & (PAC_OV_ALERT_EN_CH1 >> a) )
			PAC1954_setUCLimit( address,a+1,channels[a].UC_lim );
		if( settings->alerts_enable & (PAC_OV_ALERT_EN_CH1 >> a) )
			PAC1954_setOPLimit( address,a+1,channels[a].OP_lim );
	}

	/* Actually enable the alerts on the PAC */
	PAC1954_enableAlerts(address,settings->alerts_enable);
}
/* ********************************* A L E R T S ******************************************** */
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
uint8_t PAC1954_clearAlertEnable(uint8_t addr){
	uint8_t buffer[4];
	buffer[0] = PAC1954_ALERT_EN_REG;
	buffer[1] = 0x00;
	buffer[2] = 0x00;
	buffer[3] = 0x00;
	return I2C1_transmitData(addr, 4, buffer);
}
uint8_t PAC1954_enableAlerts( uint8_t addr,uint32_t enable_macro ){
	uint8_t buffer[4];
	uint32_t alerts = PAC1954_read24bitRegister(addr,PAC1954_ALERT_EN_REG); // Read alerts
	if( alerts == 0xFFFFFFFF ){
		return 0; // Failed
	}
	alerts |= enable_macro;

	buffer[0] = PAC1954_ALERT_EN_REG; // First byte is the register address
	buffer[3]=alerts % 0x100;
	alerts /= 0x100;
	buffer[2]=alerts % 0x100;
	alerts /= 0x100;
	buffer[1]=alerts % 0x100;
	return I2C1_transmitData(addr, 4, buffer); // Send it back?
}
uint8_t PAC1954_disableAlerts( uint8_t addr,uint32_t enable_macro ){
	uint8_t buffer[4];
	uint32_t alerts = PAC1954_read24bitRegister(addr,PAC1954_ALERT_EN_REG); // Read alerts
	if( alerts == 0xFFFFFFFF ){
		return 0; // Failed
	}
	alerts &= ~enable_macro;

	buffer[0] = PAC1954_ALERT_EN_REG; // First byte is the register address
	buffer[3]=alerts % 0x100;
	alerts /= 0x100;
	buffer[2]=alerts % 0x100;
	alerts /= 0x100;
	buffer[1]=alerts % 0x100;
	return I2C1_transmitData(addr, 4, buffer); // Send it back?
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
		status *= 0x100;
		status += recBuffer[1];
		status *= 0x100;
		status += recBuffer[2];
		return status;
	}
	return 0xFFFFFFFF;
}
/* **************************************************************************************** */
