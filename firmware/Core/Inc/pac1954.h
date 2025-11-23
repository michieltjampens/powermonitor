/*
 * pac1954.h
 *
 *  Created on: Nov 25, 2023
 *      Author: Michiel
 */

#ifndef INC_PAC1954_H_
#define INC_PAC1954_H_

#include "i2c1.h"

	#define PAC_CHANNELS	0x04
	/* Exported types ------------------------------------------------------------*/
	/* Structure to store the settings in nv memory */
	/* For EEPROM in STM it needs to be a multiple of 4 byte to store the whole struct*/
	typedef struct {

		/* Other 8byte*/
		uint16_t overcurrentTime;
		uint32_t alerts_enable;
		uint8_t  pacAddress;
		uint8_t  spare; // Because the amount of bytes needs to be a multiple of 4

		// Total: 8byte
	} PacSettings;

	typedef struct {
		/* Chn settings 12byte */
		uint16_t UV_lim;
		uint16_t OV_lim;
		uint16_t UC_lim;
		uint16_t OC_lim;
		uint16_t OP_lim;
		uint8_t pacAddress;
		uint8_t spare;
	} PacChannel;

	typedef struct {
		uint16_t voltage;
		uint16_t current;
	} VoltageCurrent;

	uint8_t PAC1954_findAddress(void);
	uint8_t PAC1954_setControl( uint8_t addr, uint16_t value );
	uint8_t PAC1954_readVoltageCurrent( uint8_t address, VoltageCurrent *lastVoltCur );
	uint8_t PAC1954_readAvgVoltageCurrent( uint8_t address, VoltageCurrent *lastVoltCur );
	uint32_t PAC1954_readAccCount( uint8_t address ); // Read the content of the accumulator counter register
	uint8_t PAC1954_doRefreshV( uint8_t address);
	uint8_t PAC1954_doRefresh( uint8_t address);
	uint8_t PAC1954_checkState(void);

	// Alerts
	uint8_t PAC1954_setAlert1( uint8_t addr, uint32_t value );
	uint8_t PAC1954_setAlert2( uint8_t addr, uint32_t value );
	uint32_t PAC1954_readAlertStatus(uint8_t address);
	uint32_t PAC1954_readAlertEnable(uint8_t address);
	uint8_t PAC1954_clearAlertEnable(uint8_t addr);
	uint8_t PAC1954_enableAlerts( uint8_t addr,uint32_t enable_macro );
	uint8_t PAC1954_disableAlerts( uint8_t addr,uint32_t enable_macro );

	// Set Limits
	uint8_t PAC1954_setOVLimit( uint8_t addr, uint8_t chn, uint16_t limit );
	uint8_t PAC1954_setUVLimit( uint8_t addr, uint8_t chn, uint16_t limit );
	uint8_t PAC1954_setOCLimit( uint8_t addr, uint8_t chn, uint16_t limit );
	uint8_t PAC1954_setUCLimit( uint8_t addr, uint8_t chn, uint16_t limit );
	uint8_t PAC1954_setOPLimit( uint8_t addr, uint8_t chn, uint16_t limit );
	uint8_t PAC1954_setLimit( uint8_t addr, uint8_t chn, uint16_t limit,uint8_t limitReg, uint32_t enableVal );

	// Read limits
	uint16_t PAC1954_readOVlimit( uint8_t address, uint8_t ch );
	uint16_t PAC1954_readUVlimit( uint8_t address, uint8_t ch );
	uint16_t PAC1954_readOClimit( uint8_t address, uint8_t ch );
	uint16_t PAC1954_readUClimit( uint8_t address, uint8_t ch );
	uint16_t PAC1954_readOPlimit( uint8_t address, uint8_t ch );
	uint16_t PAC1954_readLimitReg( uint8_t address, uint8_t ch, uint8_t reg );
	uint32_t PAC1954_read24bitRegister(uint8_t address,uint8_t reg);
	uint16_t PAC1954_read16bitRegister(uint8_t address,uint8_t reg);

	void PAC1954_applySettings( PacSettings * settings, PacChannel * channels, uint8_t cnt);

	#define PAC_OK						0x01
	#define PAC_BAD						0x00

	/* Refresh cmds */
	#define PAC1954_REFRESH				0x00
	#define PAC1954_REFRESH_V			0x1F
	/* Registers */
	#define PAC1954_CTRL_REG			0x01
	#define PAC1954_ACC_CNT_REG			0x02	// 4 bytes (32bit)
	#define PAC1954_VACCN_1_REG			0x03	// 7 bytes (56bit)
	#define PAC1954_VACCN_2_REG			0x04	// 7 bytes (56bit)
	#define PAC1954_VACCN_3_REG			0x05	// 7 bytes (56bit)
	#define PAC1954_VACCN_4_REG			0x06	// 7 bytes (56bit)

	#define PAC1954_VBUSN_REG			0x07	// 4x2byte (64bit)
	#define PAC1954_SENSEN_REG			0x0B	// 4x2byte (64bit)
	#define PAC1954_VBUSN_AVG_REG		0x0F	// 4x2byte (64bit)
	#define PAC1954_VSENSEN_AVG_REG		0x13	// 4x2byte (64bit)
	#define PAC1954_VPOWERN_REG			0x17	// 4x4byte (128bit)

	#define PAC1954_ACCUM_CFG_REG		0x25	// 1 byte (8bit)
	#define PAC1954_ALERT_STS_REG		0x26	// 3 byte (24bit)
	#define PAC1954_SLOW_ALERT1_REG		0x27	// 3 byte (24bit)
	#define PAC1954_GPIO_ALERT2_REG		0x28	// 3 byte (24bit)
	/* Limit reg */
	#define PAC1954_OC_LMTN_1_REG		0x30	// 2 byte (16bit)
	#define PAC1954_OC_LMTN_2_REG		0x31	// 2 byte (16bit)
	#define PAC1954_OC_LMTN_3_REG		0x32	// 2 byte (16bit)
	#define PAC1954_OC_LMTN_4_REG		0x33	// 2 byte (16bit)
	#define PAC1954_UC_LMTN_1_REG		0x34	// 2 byte (16bit)
	#define PAC1954_UC_LMTN_2_REG		0x35	// 2 byte (16bit)
	#define PAC1954_UC_LMTN_3_REG		0x36	// 2 byte (16bit)
	#define PAC1954_UC_LMTN_4_REG		0x37	// 2 byte (16bit)
	#define PAC1954_OP_LMTN_1_REG		0x38	// 2 byte (16bit)
	#define PAC1954_OP_LMTN_2_REG		0x39	// 2 byte (16bit)
	#define PAC1954_OP_LMTN_3_REG		0x3A	// 2 byte (16bit)
	#define PAC1954_OP_LMTN_4_REG		0x3B	// 2 byte (16bit)
	#define PAC1954_OV_LMTN_1_REG		0x3C	// 2 byte (16bit)
	#define PAC1954_OV_LMTN_2_REG		0x3D	// 2 byte (16bit)
	#define PAC1954_OV_LMTN_3_REG		0x3E	// 2 byte (16bit)
	#define PAC1954_OV_LMTN_4_REG		0x3F	// 2 byte (16bit)
	#define PAC1954_UV_LMTN_1_REG		0x40	// 2 byte (16bit)
	#define PAC1954_UV_LMTN_2_REG		0x41	// 2 byte (16bit)
	#define PAC1954_UV_LMTN_3_REG		0x42	// 2 byte (16bit)
	#define PAC1954_UV_LMTN_4_REG		0x43	// 2 byte (16bit)
	/* Limit trigger samples */
	#define PAC1954_OC_LMTN_S_REG		0x44	// 1 byte (8bit)
	#define PAC1954_UC_LMTN_S_REG		0x45	// 1 byte (8bit)
	#define PAC1954_OP_LMTN_S_REG		0x46	// 1 byte (8bit)
	#define PAC1954_OV_LMTN_S_REG		0x47	// 1 byte (8bit)
	#define PAC1954_UV_LMTN_S_REG		0x48	// 1 byte (8bit)
	/* Other */
	#define PAC1954_ALERT_EN_REG		0x49	// 3 byte (24bit)
	#define PAC1954_ACC_CFG_ACT_REG		0x4A	// 1 byte (8bit)
	#define PAC1954_ACC_CFG_LAT_REG		0x4B	// 1 byte (8bit)
	#define PAC1954_PRODUCT_ID			0xFD	// 1 byte (8bit)
	#define PAC1954_MANUF_ID_REG		0xFE	// 1 byte (8bit)
	#define PAC1954_REVISION_ID_REG		0xFF	// 1 byte (8bit)


	/* ALERT ENABLE MACRO'S */
	#define PAC_OC_ALERT_EN_CH1		0x00800000  // bit 23 set
	#define PAC_OC_ALERT_EN_CH2		0x00400000  // bit 22 set
	#define PAC_OC_ALERT_EN_CH3		0x00200000  // bit 21 set
	#define PAC_OC_ALERT_EN_CH4		0x00100000  // bit 20 set

	#define PAC_UC_ALERT_EN_CH1		0x00080000  // bit 19 set
	#define PAC_UC_ALERT_EN_CH2		0x00040000  // bit 18 set
	#define PAC_UC_ALERT_EN_CH3		0x00020000  // bit 17 set
	#define PAC_UC_ALERT_EN_CH4		0x00010000  // bit 16 set

	#define PAC_OV_ALERT_EN_CH1		0x00008000  // bit 15 set
	#define PAC_OV_ALERT_EN_CH2		0x00004000  // bit 14 set
	#define PAC_OV_ALERT_EN_CH3		0x00002000  // bit 13 set
	#define PAC_OV_ALERT_EN_CH4		0x00001000  // bit 12 set

	#define PAC_UV_ALERT_EN_CH1		0x00000800  // bit 11 set
	#define PAC_UV_ALERT_EN_CH2		0x00000400  // bit 10 set
	#define PAC_UV_ALERT_EN_CH3		0x00000200  // bit  9 set
	#define PAC_UV_ALERT_EN_CH4		0x00000100  // bit  8 set

	#define PAC_OP_ALERT_EN_CH1		0x00000080  // bit 7 set
	#define PAC_OP_ALERT_EN_CH2		0x00000040  // bit 6 set
	#define PAC_OP_ALERT_EN_CH3		0x00000020  // bit 5 set
	#define PAC_OP_ALERT_EN_CH4		0x00000010  // bit 4 set

#endif /* INC_PAC1954_H_ */
