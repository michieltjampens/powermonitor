/*
 * hardware.h

 */

#ifndef INC_HARDWARE_H_
	#define INC_HARDWARE_H_

	/* Includes ------------------------------------------------------------------*/

	/* Private includes ----------------------------------------------------------*/

	/* Exported types ------------------------------------------------------------*/
	typedef struct {
		/* OUT 1 settings 64bit */
		uint16_t out1_lowvlim;
		uint16_t out1_highvlim;
		uint16_t out1_curlim;
		uint8_t out1_defState;
		uint8_t out1_spare;

		/* OUT 2 settings 64bit */
		uint16_t out2_lowvlim;
		uint16_t out2_highvlim;
		uint16_t out2_curlim;
		uint8_t out2_defState;
		uint8_t out2_spare;

		/* OUT 3 settings 64bit */
		uint16_t out3_lowvlim;
		uint16_t out3_highvlim;
		uint16_t out3_curlim;
		uint8_t out3_defState;
		uint8_t out3_spare;

		/* OUT 4 settings 64bit */
		uint16_t out4_lowvlim;
		uint16_t out4_highvlim;
		uint16_t out4_curlim;
		uint8_t out4_defState;
		uint8_t out4_spare;

		/* Other 32bit*/
		uint16_t overcurrentTime;
		uint8_t  pacAddress;
		uint8_t spare;
	} settings_t;

	extern settings_t *GLOBAL_settings_ptr;
	/* Exported constants --------------------------------------------------------*/

	/* Exported macro ------------------------------------------------------------*/

	/* Exported functions prototypes ---------------------------------------------*/
	/* Private defines -----------------------------------------------------------*/

	/* Hearbeat */
	#define HEART_ON  GPIOB->ODR |= GPIO_ODR_OD5
	#define HEART_OFF GPIOB->ODR &= ~(GPIO_ODR_OD5)


#endif /* INC_HARDWARE_H_ */
