/*
 * pac1954.h
 *
 *  Created on: Nov 25, 2023
 *      Author: Michiel
 */

#ifndef INC_PAC1954_H_
#define INC_PAC1954_H_

#include "i2c1.h"


	/* Exported types ------------------------------------------------------------*/

	typedef struct {
		uint16_t out1_voltage;
		uint16_t out2_current;
		uint16_t out3_voltage;
		uint16_t out4_voltage;

		uint16_t out1_current;
		uint16_t out2_voltage;
		uint16_t out3_current;
		uint16_t out4_current;
	} voltcur;

	uint8_t PAC1954_findAddress(void);
	uint8_t PAC1954_readVoltageCurrent( uint8_t address, voltcur *lastVoltCur );
	uint32_t PAC1954_readAccCount( uint8_t address ); // Read the content of the accumulator counter register
	uint8_t PAC1954_doRefreshV( uint8_t address);
	/* Sense to position translation */

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

#endif /* INC_PAC1954_H_ */
