/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "../inc/lpuart1.h"
#include "../inc/main.h"

/* Shared variables ----------------------------------------------------------*/
__IO uint8_t cmdReady;
uint8_t freeSpace_USART1 = CIRCULAR-1;
uint8_t irqStatus=IDLE;

/* Private variables ---------------------------------------------------------*/

/* Circular buffer for UART OUT*/
uint8_t outputBuffer_LPUART1[CIRCULAR];
uint8_t *outTemp_LPUART1;
uint8_t *dmaStart_LPUART1;
uint8_t *outEnd_LPUART1;
uint8_t *outHead_LPUART1;
uint8_t *outTail_LPUART1;

/* Circular buffer for UART IN*/
uint8_t inputBuffer_LPUART1[64];
uint8_t *inputTemp_LPUART1;
uint8_t *inputStart_LPUART1;
uint8_t *inputEnd_LPUART1;
uint8_t *inputHead_LPUART1;
uint8_t *inputTail_LPUART1;

uint16_t lpuart_todo=0;

void LPUART1_Configure(){
    /* Initialise the circular buffers */
    inputStart_LPUART1 = &inputBuffer_LPUART1[0];
    inputEnd_LPUART1 =   &inputBuffer_LPUART1[0];
    inputHead_LPUART1 =  &inputBuffer_LPUART1[0];
    inputTail_LPUART1 =  &inputBuffer_LPUART1[64-1];

    dmaStart_LPUART1 =  &outputBuffer_LPUART1[0];
    outEnd_LPUART1   =  &outputBuffer_LPUART1[0];
    outHead_LPUART1  =  &outputBuffer_LPUART1[0];
    outTail_LPUART1  =  &outputBuffer_LPUART1[CIRCULAR-1];

    LPUART1_Configure_GPIO();
    LPUART1_Configure_Setup();

    LPUART1_DMA_Init();

    cmdReady=0;
}
void LPUART1_Configure_GPIO(void){
  /* Enable the peripheral clock of GPIOA */
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

  /* GPIO configuration for USART1 signals */
  /* (1) Select Alternate Function (AF) mode (b10) on PA0 and PA1 */
  /* Moder LSB nibble: 1111 -> 1010 */
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE0|GPIO_MODER_MODE1))\
                 | (GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1);
  /* (2) AF4 for LPUSART1 signals, the line is explained in detail below it */
  GPIOA->AFR[0] = (GPIOA->AFR[0] &~ (0x00000FF0)) | (6 << (0 * 4)) | (6 << (1 * 4));

  /* Extra info                                                                         */
  /* For the alternate functions, check the datasheet 'Alternate functions'             */
  /* AFR[0]=pin 0 to 7, AFR[1]=pin 8 to 15, each pin has 4 bits for selection           */
  /* So AFR[0] &~(0x00000FF0) = AFR[0] & 0xFFFFF00F -> Reset the bits for PA0 & PA1     */
  /* 4 << (0 * 4) -> Value 4 shifted one nibble to get to PA0 position  -> 0x00000004   */
  /* 4 << (1 * 4) -> Value 4 shifted two nibbles to get to PA1 position -> 0x00000040   */
}
void LPUART1_Configure_Setup(void){
	uint32_t tickstart;

    /* Enable the peripheral clock USART1 */
    RCC->APB1ENR |= RCC_APB1ENR_LPUART1EN;
    /* Configure USART1 */
    /* System clock is 16MHz (1MHz oversampling by 16), 19200 baud (both already divided by 100) */
    LPUART1->BRR = (256*160000)/192;//19200??

    LPUART1->CR2 |= USART_CR2_SWAP; /* Swap RX and TX this needs to be set before CR1_UE is set */

    /* 8 data bit, 1 start bit, 1 stop bit, no parity */
    LPUART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    /* Extra info                                 */
    /* USART2->CR1 = USART2 Control register      */
    /* 8/1/1 no parity is default                 */
    /* USART_CR1_RE = Receiver enable             */
    /* USART_CR1_TE = Transmitter Enable          */
    /* USART_CR1_UE = USART Enable                */

    /* polling idle frame Transmission */
    tickstart = Tick;
    while((LPUART1->ISR & USART_ISR_TC) != USART_ISR_TC){
    	if ((Tick - tickstart ) > I2C_TIMEOUT_VALUE){
			error = ERROR_USART_TIMEOUT;
			return;
		}
    }
	LPUART1->ICR |= USART_ICR_TCCF;  /* Clear TC flag  (no bit for receive) */
	LPUART1->CR1 |= USART_CR1_RXNEIE; /* Enable Transmission Complete and receive interrupt */
	LPUART1->CR3 |= USART_CR3_DMAT;

	/* Configure Interrupt */
	/* Set priority for USART1_IRQn */
	NVIC_SetPriority(LPUART1_IRQn, 0);
	/* Enable USART1_IRQn */
	NVIC_EnableIRQ(LPUART1_IRQn);
}
/* *************************************************************************************** */
/* ******************************* D M A ************************************************* */
/* *************************************************************************************** */

void LPUART1_DMA_Init(){
    // Init DMA for I2C->UART
    SET_BIT(RCC->AHBENR,RCC_AHBENR_DMA1EN);				// Enable the clock

    // Need to use channel 2 because channel 1 only services ADC
    DMA1_CSELR->CSELR = 0x00000050;		// Select LPUART_TX according to page 218 of RM

    DMA1_Channel2->CPAR = (uint32_t)&(LPUART1->TDR);		// Set the destination (works)
    DMA1_Channel2->CMAR = (uint32_t)dmaStart_LPUART1;    // Set the start point of the buffer
    DMA1_Channel2->CCR = DMA_CCR_MINC 		// Memory increment
    					| DMA_CCR_DIR 		// Read from memory
						| DMA_CCR_TCIE 		// Transfer complete interrupt enabled
						| DMA_CCR_TEIE;     // Transfer error interrupt enabled
    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);		// Enable the irq
}
void DMA1_Channel2_3_IRQHandler(void) {  // now it does nothing only clears the flag
	DMA1 -> IFCR |= DMA_IFCR_CHTIF2;
	if(DMA1 -> ISR & (DMA_ISR_TCIF2)) {  // Transfer complete for channel 1
        DMA1 -> IFCR |= DMA_IFCR_CTCIF2; // Clear the transfer complete flag
        CLEAR_BIT(DMA1_Channel2->CCR,DMA_CCR_EN); // Finished, so disable it
    	// Update the dma start position
    	DMA1_Channel2->CMAR = (uint32_t)dmaStart_LPUART1;
    }
}
void LPUART1_check_dma(){
	// If DMA is active, nothing to do so return
	if( DMA1_Channel2->CCR & DMA_CCR_EN )
		return;

	// If there isn't any new data in the buffer, nothing do so so return
	if( dmaStart_LPUART1 == outEnd_LPUART1 )
		return;

	/* If we get here, it means the DMA isn't active but there's at least one byte in the buffer */
	uint16_t todo = lpuart_todo; // Make a backup because isr might overwrite this

	// Find the adres of the last byte, which will be the next start
	uint8_t *newStart_LPUART1 = dmaStart_LPUART1 + todo;

	// Check if we go beyond or on the tail
	if( newStart_LPUART1 >= outTail_LPUART1 ){ // We go beyond, so only start the part till the tail
		todo = outTail_LPUART1-dmaStart_LPUART1; // Update the start to where the dma will end
		newStart_LPUART1 = outHead_LPUART1;    // Update the start to beginning of buffer
	}
	DMA1_Channel2->CNDTR = todo;
	dmaStart_LPUART1 = newStart_LPUART1; // Update the start to where the dma will end
	lpuart_todo -= todo; 			 // subtract the portion from the total

	SET_BIT(DMA1_Channel2->CCR,DMA_CCR_EN); // Start it again
}
/* *************************************************************************************** */
void LPUART1_writeByte( uint8_t bt ){
	*outEnd_LPUART1++ = bt;
	lpuart_todo++;
	if (outEnd_LPUART1 == outTail_LPUART1) // So never write on the tail!
		outEnd_LPUART1 = outHead_LPUART1;  // End reached, back to head
}
/**
 * Brief Send elements from an array in the send buffer till a 0x00 is found
 * Param
 * 		buffer -> Pointer to the array
 * RetVal
 * 		None
 */
void LPUART1_writeText( const char *buffer ){
	uint8_t cnt=0;

    while( *buffer != 0x00){
    	*outEnd_LPUART1++ = *buffer++;
    	cnt++;
		if (outEnd_LPUART1 == outTail_LPUART1) // So never write on the tail!
			outEnd_LPUART1 = outHead_LPUART1;  // End reached, back to head
    }
    lpuart_todo += cnt;
}
void LPUART1_writeNullEndedArray( uint8_t *buffer ){
	uint8_t cnt=0;

    while( *buffer != 0x00 ){
    	*outEnd_LPUART1++ = *buffer++;
    	cnt++;
		if (outEnd_LPUART1 == outTail_LPUART1) // So never write on the tail!
			outEnd_LPUART1 = outHead_LPUART1;  // End reached, back to head
    }
    lpuart_todo += cnt;
}
void LPUART1_writeHexByteArrayNoPrefix( uint8_t *nrs, uint8_t length ){
	uint8_t tmp;
	uint8_t added=length;

	while( length != 0x00){
		tmp = *nrs/16;
		if(tmp>9){
			*outEnd_LPUART1++ = tmp+55;
		}else{
			*outEnd_LPUART1++ = tmp + '0';
		}
		if (outEnd_LPUART1 == outTail_LPUART1) // So never write on the tail!
			outEnd_LPUART1 = outHead_LPUART1;  // End reached, back to head
		tmp = *nrs%16;
		if(tmp>9){
			*outEnd_LPUART1++ = tmp+55;
		}else{
			*outEnd_LPUART1++ = tmp + '0';
		}
		if (outEnd_LPUART1 == outTail_LPUART1) // So never write on the tail!
			outEnd_LPUART1 = outHead_LPUART1;  // End reached, back to head
        nrs++;
        length--;
    }
	lpuart_todo+=added;
}
/**
 * Brief Convert a 16bit number to hex ascii and send it
 * Param
 * 		nr -> The number to send
 */
void LPUART1_writeHexWord( uint16_t nr ){
	uint8_t data[7]={'0','x','0','0',0,0,0};
	uint8_t tmp;
	uint8_t index=5;
	if( nr <= 0xFF ){
		index=3;
	}
	while( index > 1 ){
		tmp = nr%16;
		if(tmp>9){
			data[index]= tmp+55;
		}else{
			data[index] = tmp + '0';
		}
		nr -= tmp;
		nr /= 16;
		index --;
	}
	LPUART1_writeNullEndedArray(data);
}
void LPUART1_writeHexQuad( uint32_t nr ){
	uint8_t data[11]={'0','x','0','0','0','0','0','0','0','0',0};
	uint8_t tmp;
	uint8_t index=9;
	while( index > 1 ){
		tmp = nr%16;
		if(tmp>9){
			data[index]= tmp+55;
		}else{
			data[index] = tmp + '0';
		}
		nr -= tmp;
		nr /= 16;
		index --;
	}
	LPUART1_writeNullEndedArray(data);
}

void LPUART1_Transfer_Buffer( void ){
    uint8_t rec[16];
    uint8_t a;

    if( inputStart_LPUART1 == inputEnd_LPUART1) // This shouldn't happen, but somehow does
    	return;

    /* Clean the buffer that will be used */
    for( a=0;a<16;a++)
        rec[a]=0x00;

    a = 0x00;
    /* Move the data from the circular buffer to the local one */
    inputTemp_LPUART1 = inputEnd_LPUART1;                             // Remember the current endstop for the circular buffer,because other might use it in ISR
    if (inputStart_LPUART1 > inputTemp_LPUART1) {                     // If the 'end' is closer to the beginning of the buffer than the 'start'
        do{
            rec[a++] = *inputStart_LPUART1++;
        }while( inputStart_LPUART1 != inputTail_LPUART1 && a < CIRCULAR+5);  // Repeat till the 'start' has reached the end of the buffer
        inputStart_LPUART1 = inputHead_LPUART1;                       // Because the end was reached, continue from the start of the buffer
    }
    do{
        rec[a++] = *inputStart_LPUART1++;
    }while( inputStart_LPUART1 < inputTemp_LPUART1 && a < CIRCULAR+5);      // Repeat the 'start' is the same as the 'end'

    executeCommand( rec );
}
void LPUART1_IRQHandler(void){
    uint8_t recChar = 0;
    uint8_t ok = 0x01;

    if( LPUART1->ISR & USART_ISR_TC ){
    	LPUART1->ICR = USART_ICR_TCCF; /* Clear transfer complete flag */
    }
    if((LPUART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE){ // ISR for received data
        recChar = (uint8_t)(LPUART1->RDR); /* Receive data, clear flag */
        if( recChar >= 0x20 && recChar <= 0x7F){
            *inputEnd_LPUART1++ = recChar;
            if (inputEnd_LPUART1 == inputTail_LPUART1) { // So never write on the tail!
                inputEnd_LPUART1 = inputHead_LPUART1;
            }
        }else if(recChar==0x00){
        	ok=0x00;
        }else if( recChar == '\n'){
        }else if( recChar == '\r'){
        	cmdReady ++;
        }
        // Can't echo inside the ISR! because the processor doesn't process fast enough?
        //LPUART1_SendByte(recChar); // This gives issues
    }
    if( ok==0x00 ){ // Meaning ISR was for unknown reason
        error = ERROR_USART_TRANSMIT; /* Report an error */
        NVIC_DisableIRQ(LPUART1_IRQn); /* Disable USART2_IRQn */
    }
}
uint8_t LPUART1_hasCmd(){
	if( cmdReady != 0){
		cmdReady=0;
		return 1;
	}
	return 0;
}
