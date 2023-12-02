/*
 * hardware.h

 */

#ifndef INC_HARDWARE_H_
	#define INC_HARDWARE_H_

	/* Includes ------------------------------------------------------------------*/

	/* Private includes ----------------------------------------------------------*/

	/* Exported types ------------------------------------------------------------*/
	typedef struct {
		/* OUT 1 settings 48bit */
		uint16_t out1_lowvlim;
		uint16_t out1_highvlim;
		uint16_t out1_curlim;

		/* OUT 2 settings 48bit */
		uint16_t out2_lowvlim;
		uint16_t out2_highvlim;
		uint16_t out2_curlim;

		/* OUT 3 settings 48bit */
		uint16_t out3_lowvlim;
		uint16_t out3_highvlim;
		uint16_t out3_curlim;

		/* OUT 4 settings 48bit */
		uint16_t out4_lowvlim;
		uint16_t out4_highvlim;
		uint16_t out4_curlim;

		/* Other 24bit*/
		uint16_t overcurrentTime;
		uint8_t  pacAddress;
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
