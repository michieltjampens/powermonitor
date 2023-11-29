#ifndef INC_LPUART1_H_
#define INC_LPUART1_H_


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "./stm32l0xx.h"

/* Private includes ----------------------------------------------------------*/
/* Shared Variables  ---------------------------------------------------------*/
extern __IO uint32_t Tick;
extern __IO uint16_t error;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/
void LPUART1_Configure(void);
uint8_t LPUART1_Buffer_Free(void);
void LPUART1_Configure_Setup(void);
void LPUART1_Configure_GPIO(void);

void LPUART1_writeText( const char *buffer );
void LPUART1_writeNullEndedArray( uint8_t *buffer );
void LPUART1_writeByteArray( uint8_t *buffer, uint8_t length );
void LPUART1_writeDec( uint16_t nr );
void LPUART1_writeHexWord( uint16_t number );
void LPUART1_writeHexQuad( uint32_t nr );
void LPUART1_writeHexByteNoPrefix( uint8_t nr );
void LPUART1_writeHexByteArrayNoPrefix( uint8_t *nrs, uint8_t length );
void LPUART1_writeHexWordNoPrefix( uint16_t nr );
void LPUART1_writeByte( uint8_t data );
void LPUART1_Transfer_Buffer( void );
uint8_t LPUART1_hasCmd(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ERROR_USART_TRANSMIT 0x20
#define ERROR_USART_ISR_TC 	 0x21
#define ERROR_USART_TIMEOUT  0x22

#define CIRCULAR 120

#define IDLE 	0x01
#define BUSY 	0x02
#define WAITING 0x04

extern uint32_t Tock;
#ifdef __cplusplus
}
#endif

#endif /* INC_LPUART1_H_ */
