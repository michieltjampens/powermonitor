/**
 * Class with methods to use the I2C1 peripheral on the STM32L0x0 device
 */
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "i2c1.h"

/* Private variables ---------------------------------------------------------*/
uint8_t * i2c_txBuffer;

uint8_t * i2c_rxBuffer_8bit; // Pointer to 8bit receiving buffer
uint8_t * i2c_rxBuffer_8bit_END;  // Points to the end of the 8bit buffer
uint16_t * i2c_rxBuffer_16bit; // Pointer to 16bit receiving buffer
uint16_t * i2c_rxBuffer_16bit_END; // Points to the end of the 16bit buffer

uint8_t bits=0x00;
uint8_t toReceive=0x00;
uint8_t toSend=0x00;
/* *********************************** S E T U P *************************************************** */
/**
 * Brief Init the I2C1 peripheral and the used GPIO's
 */
__INLINE void I2C1_Configure_Master(void){

  /* GPIO Setup, PA9=SCL and PA10=SDA */
  SET_BIT(RCC->IOPENR,RCC_IOPENR_GPIOAEN);  // Enable the peripheral clock of GPIOA
  MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE9|GPIO_MODER_MODE10, GPIO_MODER_MODE9_1|GPIO_MODER_MODE10_1); //Use AF
  MODIFY_REG( GPIOA->AFR[1], GPIO_AFRH_AFSEL9|GPIO_AFRH_AFSEL10, (1 << GPIO_AFRH_AFSEL9_Pos) | (1 << GPIO_AFRH_AFSEL10_Pos) );// Pick AF1 (I2C1)

  SET_BIT( GPIOA->OTYPER, GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_10 ); // Open drain

  SET_BIT(RCC->APB1ENR,RCC_APB1ENR_I2C1EN); // Enable the peripheral clock I2C1 do this before GPIO's

  /* Configure I2C1 as master */
  I2C1->TIMINGR = (uint32_t)0x00503D5A; // Timing setup: 100kHz mode, PCLK=16MHz, rise=100ns,fall=10ns (STM32CubeMX calculated)
  SET_BIT(I2C1->CR1,I2C_CR1_RXIE | I2C_CR1_TXIE); // Enable Receive and transmit interrupt

  /* Configure Interrupts */
  NVIC_SetPriority(I2C1_IRQn, 0);
  NVIC_EnableIRQ(I2C1_IRQn);

  SET_BIT(I2C1->CR1,I2C_CR1_PE); 		// Enable I2C peripheral
  while((I2C1->CR1 & I2C_CR1_PE) == 0); // Wait for it to be enabled
}
/* *********************************** S E N D I N G ************************************************* */
/**
 * Brief	Send data to a device on the bus
 * Param
 *   address  - 7bit address (so omit the r/w bit)
 *   length - the amount of bytes to send
 *   data  - pointer to array containing length of bytes
 * Retval
 * 		0 -> No device
 * 		1 -> Device replied
 * 		x -> Error
 */
uint8_t I2C1_transmitData( uint8_t address, uint8_t length, uint8_t *data){
	uint32_t tickstart;

	toSend=length;

	i2c_txBuffer = data;	// Do this before start otherwise might be to late...

	I2C1->CR2 = I2C_CR2_AUTOEND | (length << I2C_CR2_NBYTES_Pos) | (address << 1);

	if( I2C1_start() != I2C_OK ) // If Start failed, give up
		return ERROR_I2C_NACK;

	if( length == 0 ){
		return I2C_OK;
	}

	tickstart = Tick;
	while( toSend != 0 ){
		if ((Tick - tickstart ) > I2C_TIMEOUT_VALUE){
			return ERROR_I2C_NO_TXE_EMTPY;
		}
	}

	return I2C_OK;
}
uint8_t I2C1_start(){
	uint32_t tickstart;
	// Enable auto-end, set amount of bytes to send and the address shifted for R/W bit
	SET_BIT(I2C1->ICR, I2C_ICR_NACKCF); // Make sure this is cleared

	I2C1->CR2 |= I2C_CR2_START;

	tickstart = Tick;
	while ((I2C1->CR2 & I2C_CR2_START)==0) { // Wait for start
		if ((Tick - tickstart ) > I2C_TIMEOUT_VALUE){
			return ERROR_I2C_NO_TXE_EMTPY;
		}
	}
	tickstart = Tick;
	while ((I2C1->CR2 & I2C_CR2_START)==I2C_CR2_START) { // Wait for start to go down
		if ((Tick - tickstart ) > I2C_TIMEOUT_VALUE){
			return ERROR_I2C_NO_TXE_EMTPY;
		}
	}
	if( I2C1->ISR & I2C_ISR_NACKF ){ // Check if NACKF is raised, if so that's not good
		SET_BIT(I2C1->ICR, I2C_ICR_NACKCF); // Clear it
		SET_BIT(I2C1->ICR, I2C_ICR_STOPCF); // Clear it
		return ERROR_I2C_NACK;
	}else{
		return I2C_OK;
	}
}
/**
 * Brief	Transmit a single byte to the given address
 * Param
 *   i2c_Address  - 7bit address (so omit the r/w bit)
 *   data  - pointer to array containing length of bytes
 * Retval
 * 		0 -> No device
 *		1 -> Device replied
 * 		x -> Error
 */
uint8_t I2C1_transmitByte( uint8_t i2c_Address, uint8_t data){
	if( i2c_Address==0x00)
		return 0;
	uint8_t d[1]={data};
	return I2C1_transmitData( i2c_Address,1,d);
}
/* ******************************** R E C E I V I N G ************************************************* */
/**
 * Block read, starting at a certain register and write to uint16_t buffer
 * Param
 * 	i2c_Address The 7bit address of the device
 * 	reg 		The address of the register to start at
 * 	length 		The amount of bytes to read
 * 	data 		Pointer to the buffer to write the received data to
 * RetVal
 *  I2C_OK(0x01) if no issues were encountered
 *  Other value if so (see i2c.h for options)
 */
uint8_t I2C1_Read16bitData( uint8_t i2c_Address, uint8_t reg, uint8_t length, uint16_t *data){
	i2c_rxBuffer_16bit = data;
	i2c_rxBuffer_16bit_END = data+length;
	length *=2; // Because the bus reads a byte at a time
	bits=16; // Set this to 16 so the IRQ knows if it writes to uint8_t or uint16_t
	return I2C1_ReadData( i2c_Address, reg, length);
}
/**
 * Block read, starting at a certain register and write to uint8_t buffer
 * Param
 * 	i2c_Address The 7bit address of the device
 * 	reg 		The address of the register to start at
 * 	length 		The amount of bytes to read
 * 	data 		Pointer to the buffer to write the received data to
 * RetVal
 *  I2C_OK(0x01) if no issues were encountered
 *  Other value if so (see i2c.h for options)
 */
uint8_t I2C1_Read8bitData( uint8_t i2c_Address, uint8_t reg, uint8_t length, uint8_t *data){
	i2c_rxBuffer_8bit = data;
	i2c_rxBuffer_8bit_END = data+length;
	bits=8; // Set this to 8 so the IRQ knows if it writes to uint8_t or uint16_t
	return I2C1_ReadData( i2c_Address, reg, length);
}
/**
 * Block read starting at a certain register
 * Param
 * 	i2c_Address 	The 7bit address of the device
 * 	reg 			The address of the register to start at
 * 	length 		The amount of bytes to read
 * RetVal
 *  I2C_OK(0x01) if no issues were encountered
 *  Other value if so (see i2c.h for options)
 */
uint8_t I2C1_ReadData( uint8_t i2c_Address, uint8_t reg, uint8_t length){
	uint32_t tickstart;

	toReceive = length; // Store the expected length, will be decreased in the irq

	tickstart = Tick;
	while((I2C1->ISR & I2C_ISR_TXE) == 0){  // Wait till transfer buffer is empty
		if ((Tick - tickstart ) > I2C_TIMEOUT_VALUE){ // If timeout passed
			return ERROR_I2C_NO_TXE_EMTPY;
		}
	}
	I2C1->TXDR = reg;  		// Load the first byte

	// No auto-end, set amount of bytes to send and the address shifted for R/W bit
	I2C1->CR2 = (1 << I2C_CR2_NBYTES_Pos) | (i2c_Address << 1);

	if( I2C1_start() != I2C_OK ) // If Start failed, give up
		return ERROR_I2C_NACK;

	///Then check when TC flag gets set (TC detected), 10ms max
	tickstart = Tick;
	while( (I2C1->ISR & I2C_ISR_TC)==0 ){
		if ((Tick - tickstart ) > I2C_TIMEOUT_VALUE){
			return ERROR_I2C_NO_TC_DETECT;
		}
	}
	// Now the register is set, request the data

	// Enable auto-end, set amount of bytes to receive and the address shifted for R/W bit
	I2C1->CR2 = I2C_CR2_AUTOEND | (length<<16) | I2C_CR2_RD_WRN |(i2c_Address<<1);
	I2C1->CR2 |= I2C_CR2_START; // Send start condition

	// Now wait till all the data was received...
	// This probably should be doable with ISR but for some reason STOPF is unreliable
	tickstart = Tick;
	while(toReceive != 0){ //wait for receiving the data?
		if ((Tick - tickstart ) > I2C_TIMEOUT_VALUE){
			return ERROR_I2C_DATA_REC_DELAY;
		}
	}
	return I2C_OK;
}

/* *********************************** U T I L I T Y ************************************************* */
/**
 * Brief	Check if there's a device on the given address
 * Param
 *   	i2c_Address  - 7bit address (so omit the r/w bit)
 * Retval
 * 		0 -> No device
 * 		1 -> Device replied
 * 		x -> Error
 */
uint8_t I2C1_PokeDevice( uint8_t i2c_Address ){
	return I2C1_transmitData( i2c_Address,0x00,0x00);
}
/* *************************************** I R Q **************************************************** */
void I2C1_IRQHandler(void){

  if(I2C1->ISR & I2C_ISR_RXNE){ // Receive buffer not empty
	  if( bits == 8 ){
		  if( i2c_rxBuffer_8bit <= i2c_rxBuffer_8bit_END ){
			  *i2c_rxBuffer_8bit++ = I2C1->RXDR;
			  toReceive--;
		  }else{
			  error = ERROR_I2C_OVERFLOW;
		  }
	  }else if( bits == 16 ){
		  if( i2c_rxBuffer_16bit <= i2c_rxBuffer_16bit_END ){
			  if( toReceive%2==0 ){
				  *i2c_rxBuffer_16bit = I2C1->RXDR; /* Read receive register, will clear RXNE flag */
				  *i2c_rxBuffer_16bit = *i2c_rxBuffer_16bit <<8; //Receiving MSB first, so shift it to msb
			  }else{
				  *i2c_rxBuffer_16bit += I2C1->RXDR; // Add the LSB
				  i2c_rxBuffer_16bit++; // Increment the pointer
			  }
			  toReceive--;
		  }else{
			  error = ERROR_I2C_OVERFLOW;
		  }
	  }
  }else if(I2C1->ISR & I2C_ISR_TXIS){ /*  Ready to send the next byte */
	  I2C1 ->TXDR = *i2c_txBuffer++; // Put the next byte
	  toSend--;
  }else if(I2C1->ISR & I2C_ISR_NACKF){ /* NACK Received*/
	  SET_BIT(I2C1->ICR, I2C_ICR_NACKCF); // Clear flag
  }else if(I2C1->ISR & I2C_ISR_STOPF){
	  SET_BIT(I2C1->ICR, I2C_ICR_STOPCF); // Clear flag
  }else if(I2C1->ISR & I2C_ISR_TC){ // Transfer complete?

  }else if(I2C1->ISR & I2C_ISR_BERR ){ // misplaced Start or STOP condition
	  SET_BIT(I2C1->ICR, I2C_ICR_BERRCF);
	  error=ERROR_I2C_BERR;
  }else if(I2C1->ISR & I2C_ISR_ARLO ){ // Arbitration lost
	  SET_BIT(I2C1->ICR, I2C_ICR_ARLOCF);
	  error=ERROR_I2C_ARLO;
  }else if(I2C1->ISR & I2C_ISR_PECERR){ // PEC Error in reception
	  SET_BIT(I2C1->ICR, I2C_ICR_PECCF);
	  error=ERROR_I2C_PECERR;
  }else if(I2C1->ISR & I2C_ISR_TIMEOUT){  // Timeout or tLOW detection flag
	  SET_BIT(I2C1->ICR, I2C_ICR_TIMOUTCF);
	  error=ERROR_I2C_TIMEOUT;
  }else{
    error = ERROR_I2C; /* Report an error */
    NVIC_DisableIRQ(I2C1_IRQn); /* Disable I2C2_IRQn */
  }
}
