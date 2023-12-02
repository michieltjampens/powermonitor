/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "main.h"

/* Shared Variables  ---------------------------------------------------------*/
__IO uint32_t Tick;
__IO uint16_t error;

/* Private variables ---------------------------------------------------------*/
uint8_t delay;
uint8_t pacAddress=0x00;
uint16_t heartBeats=0;
uint16_t recBuffer[12];

uint16_t check=0;
uint8_t pacState=0;

static settings_t _settings_in_ram;
settings_t *GLOBAL_settings_ptr = &_settings_in_ram;

static voltcur lastVoltCur;
voltcur *GLOBAL_voltcur_ptr = &lastVoltCur;

int main(void){
	uint8_t result;
    /*!< At this stage the microcontroller clock setting is already configured,
         this is done through SystemInit() function which is called from startup
         file (startup_stm32l0xx.s) before to branch to application main.
         To reconfigure the default setting of SystemInit() function, refer to
         system_stm32l0xx.c file
    */
	
    SysTick_Config(2000); /* 1ms config */
    SystemClock_Config();
    SysTick_Config(16000); /* 1ms config */

	init();
	PAC1954_clearAlertEnable(pacAddress);
	error = PAC1954_setOVLimit(pacAddress,1,0x1000);
	error = PAC1954_setOVLimit(pacAddress,2,0x1100);
	error = PAC1954_setOVLimit(pacAddress,3,0x0110);
	error = PAC1954_setOVLimit(pacAddress,4,0x1111);

	error = PAC1954_setUVLimit(pacAddress,1,0x1000);
	error = PAC1954_setUVLimit(pacAddress,2,0x1100);
	error = PAC1954_setUVLimit(pacAddress,3,0x4110);
	error = PAC1954_setUVLimit(pacAddress,4,0x5111);

	if(  error == I2C_OK)
		error = 0x00;

	// To test RAM '.data' section initialisation:
	//static int dont_panic = 42;
	while (1) {
		if( error!=0x00){
			LPUART1_writeText("Error Occurred -> ");
			LPUART1_writeHexWord(error);
			LPUART1_writeText("\r\n");
			error=0;
		}
		if( check>=220 && pacState==PAC_REFRESHED){ // Minimum of 10ms between refresh and readout
			readAll();
			check=0; // Reset counter
		}
		if( check>=218 && pacState==PAC_FOUND){
			result = PAC1954_doRefreshV(pacAddress);
			if(  result == I2C_OK){
				pacState=PAC_REFRESHED;
			}else{
				LPUART1_writeText("I2C:Refresh Error -> ");
				LPUART1_writeHexWord(result);
				LPUART1_writeText("\r\n");
			}
		}

		if( LPUART1_hasCmd() ){
			LPUART1_Transfer_Buffer();
		}
	}
}

/**
  * Brief   This function configures the system clock @16MHz and voltage scale 1
  *         assuming the registers have their reset value before the call.
  *         POWER SCALE   = RANGE 1
  *         SYSTEM CLOCK  = PLL MUL8 DIV2
  *         PLL SOURCE    = HSI/4
  *         FLASH LATENCY = 0
  * Param   None
  * Retval  None
  */
__INLINE void SystemClock_Config(void){

  uint32_t tickstart;
  
  /* (1) Enable power interface clock */
  RCC->APB1ENR |= (RCC_APB1ENR_PWREN);
  
  /* (2) Select voltage scale 1 (1.65V - 1.95V) i.e. (01)  for VOS bits in PWR_CR */
  PWR->CR = (PWR->CR & ~(PWR_CR_VOS)) | PWR_CR_VOS_0;
  
  /* (3) Enable HSI divided by 4 in RCC->CR */
  RCC->CR |= RCC_CR_HSION | RCC_CR_HSIDIVEN;
  
  tickstart = Tick;
    
  /* (4) Wait for HSI ready flag and HSIDIV flag */
   while ((RCC->CR & (RCC_CR_HSIRDY |RCC_CR_HSIDIVF)) != (RCC_CR_HSIRDY |RCC_CR_HSIDIVF)){
    if ((Tick - tickstart ) > HSI_TIMEOUT_VALUE){
      error = ERROR_HSI_TIMEOUT; /* Report an error */
      return;
    }      
  } 
  /* (5) Set PLL on HSI, multiply by 8 and divided by 2 */
  RCC->CFGR |= RCC_CFGR_PLLSRC_HSI | RCC_CFGR_PLLMUL8 | RCC_CFGR_PLLDIV2;
  
  /* (6) Enable the PLL in RCC_CR register */
  RCC->CR |= RCC_CR_PLLON; 
  
  tickstart = Tick;
  
  /* (7) Wait for PLL ready flag */
  while ((RCC->CR & RCC_CR_PLLRDY)  == 0){
    if ((Tick - tickstart ) > PLL_TIMEOUT_VALUE){
      error = ERROR_PLL_TIMEOUT; /* Report an error */
      return;
    }      
  }
  
  /* (8) Select PLL as system clock */
  RCC->CFGR |= RCC_CFGR_SW_PLL; /* (8) */
  tickstart = Tick;
  
  /* (9) Wait for clock switched on PLL */
  while ((RCC->CFGR & RCC_CFGR_SWS_PLL)  == 0){
    if ((Tick - tickstart ) > CLOCKSWITCH_TIMEOUT_VALUE){
      error = ERROR_CLKSWITCH_TIMEOUT; /* Report an error */
      return;
    }      
  }
}

/* ********************************************************************************************************** */
/* *********************************** A P P L I C A T I O N  C O D E *************************************** */
/* ********************************************************************************************************** */
void init(void){
	uint8_t temp=0;

	I2C1_Configure_Master();
	LPUART1_Configure();

    configure_IO();

    LPUART1_writeText("I>Booting...\r\n");
    delay = 10; // wait a bit
    while(delay!=0); // 10ms delay

    /* Read the settings from the internal E²PROM */
    if( sizeof(settings_t) % 4 ==0){ // Needs to be a multiple of 4 because e²prom has 32bit pages
    	// Get the settings from e²prom
    	memcpy(GLOBAL_settings_ptr, (uint32_t*)DATA_E2_ADDR, sizeof(settings_t));

    	//if( _settings_in_ram.pacAddress==0x00){ // nothing in there yet
    	//	resetSettings();
    	//}
    }
    pacAddress = 0x1D;//_settings_in_ram.pacAddress; // Get the read address

    // Check if I2C hardware is found
    LPUART1_writeText("I>PAC1954:");
    temp=1;//I2C1_PokeDevice(pacAddress);
    if( temp==1){
    	LPUART1_writeText("OK\r\n");
    	pacState=PAC_FOUND;
    }else if( temp == ERROR_I2C_TIMEOUT ){
    	LPUART1_writeText("TimeOut\r\n");
    }else{
    	LPUART1_writeText("NOK\r\n");
    }

	delay = 10; // wait a bit
	while(delay!=0); // 10ms delay
	LPUART1_writeText("I>Init done.\r\n");
}

void configure_IO(void){
  
  /* Enable the peripheral clock of GPIOA (inputs) and GPIOB (heartbeat) */
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;


  /* INPUTS & Interrupts*/
  /* Enable the SYStemConfiguration peripheral clock, this handles interrupts */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  /* 
    A6 = U2 alert1
    A7 = U2 alert2
  */  
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7)); // Make PA6 & PA7 inputs

  SYSCFG->EXTICR[1] &= ~(SYSCFG_EXTICR2_EXTI6_Msk|SYSCFG_EXTICR2_EXTI7_Msk); // First clear the bits that define the port of for pin 6 and 7 (inverse means all other bits remain as they were)
  SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI6_PA|SYSCFG_EXTICR2_EXTI7_PA); // Then set the pins 6 and 7 to select port A

  /* Next up is enabling the interrupt */
  EXTI->IMR |= (EXTI_IMR_IM4|EXTI_IMR_IM6|EXTI_IMR_IM7); // Enable interrupt for both 6 and 7, no need to clear anything first

  /* Next select the kind of edge to trigger on */
  EXTI->RTSR &= ~(EXTI_RTSR_RT4_Msk|EXTI_RTSR_RT6_Msk|EXTI_RTSR_RT7_Msk);	// Clear rising edge
  EXTI->FTSR |= (EXTI_FTSR_FT4 | EXTI_FTSR_FT6 | EXTI_FTSR_FT7);			// Set Falling edge

  /* Finally enable the interrupt */
  NVIC_SetPriority(EXTI4_15_IRQn, 0x03); // This is for pins between 4 and 15 which 6 and 7 belong to, set to lowest priority
  NVIC_EnableIRQ(EXTI4_15_IRQn); // Actually enable it

  /* OUTPUTS */
  /* Select output mode (01) on GPIOB pin 5 */
  /*
    PB5 = Heart beat led
  */
  GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE5))| (GPIO_MODER_MODE5_0);

}

/* ******************************************** I 2 C ******************************************************** */

void I2C1_FindDevices(){

	uint8_t addr;
	uint8_t tmp;
	LPUART1_writeText("I2C>");
	/* Clear the address and bytes to send spot, then fill them in*/
	for( addr=0x01;addr<127;addr++){
		if( addr%16==0)
			LPUART1_writeText("\r\nI2C>");
		//
		switch( I2C1_PokeDevice(addr) ){
			case 0x00:
				LPUART1_writeByte('.'); // Show some progress?
				break;
			case 0x01:
				LPUART1_writeHexWord(addr);
				tmp++;
				break;
			case ERROR_I2C_TIMEOUT:
				LPUART1_writeText(":Time Out");
				addr=0x8F;//Stop searching
				break;
		}

		delay = 30; // wait a bit
		while(delay!=0); // 30ms delay
	}
	LPUART1_writeText("\r\nI2C>Found ");
	LPUART1_writeHexWord(tmp);
	LPUART1_writeText(" devices\r\n");
}
/* ***************************************** C L I ******************************************************* */
void executeCommand( uint8_t * cmd ){
    uint8_t ok = 0xFF;

    switch( cmd[0]){
      case 'E': // E²PROM
    	  if( cmd[1]=='C'){ // Reset the e²prom
    		  resetSettings();
    	  }else if( cmd[1]=='R'){ // Read from the e²prom
    		  memcpy(GLOBAL_settings_ptr, (uint32_t*)DATA_E2_ADDR, sizeof(settings_t));
    	  }else if( cmd[1]=='W'){ // Write to the e²prom
    		  settings_write();
    	  }
      case 's': // Read data
    	  /*if( I2C1_SendSingleByte(pacAddress,PAC1954_REFRESH_N==0x01)){
    		  ok=0xFF;
    	  }*/
    	  break;
      case 'p': // Send last voltage and sense reading for all channels
    	  readVCdata();
    	  break;
      case 'u':
    	  switch(cmd[1]){
    	  	  case 'v': readUVLimits(); break;
    	  	  case 'c': readUCLimits(); break;
    	  	  default: ok=0x00;
    	  }
		  break;
      case 'o':
    	  switch(cmd[1]){
    	  	  case 'v': readOVLimits(); break;
    	  	  case 'c': readOCLimits(); break;
    	  	  case 'p': readOPLimits(); break;
    	  	  default: ok=0x00;
    	  }
		  break;
      case 'a':
    	  switch(cmd[1]){
    	  	  case 's': readAlertStatus(); break;
    	  	  case 'e': readAlertEnable(); break;
    	  	  default: ok=0x00;
    	  }
    	  break;
      default : ok=0x00; break;

    }
    LPUART1_writeNullEndedArray(cmd);
    if( ok != 0xFF ){
    	LPUART1_writeText(":NOK\r\n");
    }else{
    	LPUART1_writeText(":OK\r\n");
    }
}
/* ***************************************** P A C 1 9 5 4  ******************************************************* */
void readAll(){
	readVCdata(); // Read Vbus, Vsense
	//readAccumulator(3);
	//readAccumulator(4);
	//readAccumulator(1);
	//readAccumulator(2);
	//readAlertStatus();
	//readAlertEnable();
}
void readVCdata(){
	uint8_t res = PAC1954_readVoltageCurrent( pacAddress, &lastVoltCur );
	if( res==I2C_OK ){
			LPUART1_writeText("VC:");
			LPUART1_writeHexWord(pacAddress);
			LPUART1_writeByte(';');
			LPUART1_writeHexWord(lastVoltCur.out1_voltage);
			LPUART1_writeByte(';');
			LPUART1_writeHexWord(lastVoltCur.out1_current);
			LPUART1_writeByte(';');
			LPUART1_writeHexWord(lastVoltCur.out2_voltage);
			LPUART1_writeByte(';');
			LPUART1_writeHexWord(lastVoltCur.out2_current);
			LPUART1_writeByte(';');
			LPUART1_writeHexWord(lastVoltCur.out3_voltage);
			LPUART1_writeByte(';');
			LPUART1_writeHexWord(lastVoltCur.out3_current);
			LPUART1_writeByte(';');
			LPUART1_writeHexWord(lastVoltCur.out4_voltage);
			LPUART1_writeByte(';');
			LPUART1_writeHexWord(lastVoltCur.out4_current);
			LPUART1_writeText("\r\n");
			pacState=PAC_FOUND;
	}else{
		LPUART1_writeText("I2C:VC Error->");
		printI2Cerror(res);
		LPUART1_writeText("\r\n");
	}
}

void printI2Cerror(uint8_t error){
	switch(error){
		case ERROR_I2C_NO_TXE_EMTPY:    LPUART1_writeText("Transfer Busy"); break;
		case ERROR_I2C_NO_BUSY_FLAG :   LPUART1_writeText("Busy Flag"); break;
		case ERROR_I2C_NO_TC_DETECT :   LPUART1_writeText("No TC");     break;
		case ERROR_I2C_NO_BUSY_FLAG2:   LPUART1_writeText("No Busy 2"); break;
		case ERROR_I2C_NO_STOP_DT:	    LPUART1_writeText("STOP DT");   break;
		case ERROR_I2C_DATA_REC_DELAY:  LPUART1_writeText("REC Delay"); break;
		default: LPUART1_writeText("Other"); break;
	}
}
void readAccumulator( uint8_t acc ){
	uint8_t buffer[8];

	// Read the accumulator data
	uint8_t res = I2C1_Read8bitData( pacAddress, 0x02+acc , 7,buffer);
	if( res==I2C_OK ){
		LPUART1_writeText("AC:");
		LPUART1_writeHexWord(pacAddress);
		switch(acc){
			case 1:LPUART1_writeText(";3;"); break;
			case 2:LPUART1_writeText(";4;"); break;
			case 3:LPUART1_writeText(";1;"); break;
			case 4:LPUART1_writeText(";2;"); break;
		}
		LPUART1_writeHexWord(buffer[0]);
		LPUART1_writeHexByteArrayNoPrefix(&buffer[1],6);
		LPUART1_writeByte(';');

		// Read the accumulator count
		uint32_t cnt = PAC1954_readAccCount(pacAddress);
		if( cnt!=0 ){
			LPUART1_writeHexQuad(cnt);
		}else{
			LPUART1_writeText("0x0");
		}
		LPUART1_writeText("\r\n");
	}else{
		LPUART1_writeText("I2C:AC Error->");
		printI2Cerror(res);
		LPUART1_writeText("\r\n");
	}
}
void readAlertStatus(){
	uint32_t status = PAC1954_readAlertStatus(pacAddress);
	LPUART1_writeText("AS:"); // Header
	LPUART1_writeHexWord(pacAddress); // address of PAC
	LPUART1_writeByte(';');
	if( PAC1954_checkState() == 1){
		LPUART1_writeHexQuad(status);
	}else{
		LPUART1_writeText("-1");
	}
	LPUART1_writeText("\r\n");
}
void readAlertEnable(){
	uint32_t status = PAC1954_readAlertEnable(pacAddress);
	LPUART1_writeText("AE:"); // Header
	LPUART1_writeHexWord(pacAddress); // address of PAC
	LPUART1_writeByte(';');
	if( PAC1954_checkState() == 1){
		LPUART1_writeHexQuad(status);
	}else{
		LPUART1_writeText("-1");
	}
	LPUART1_writeText("\r\n");
}
void readOVLimits(){
	LPUART1_writeText("OVL:");
	LPUART1_writeHexWord(pacAddress);
	for( uint8_t a=1;a<5;a++ ){
		LPUART1_writeByte(';');
		LPUART1_writeHexWord( PAC1954_readOVlimit( pacAddress, a ) );
	}
	LPUART1_writeText("\r\n");
}
void readUVLimits(){
	LPUART1_writeText("UVL:");
	LPUART1_writeHexWord(pacAddress);
	for( uint8_t a=1;a<5;a++ ){
		LPUART1_writeByte(';');
		LPUART1_writeHexWord( PAC1954_readUVlimit( pacAddress, a ) );
	}
	LPUART1_writeText("\r\n");
}
void readUCLimits(){
	LPUART1_writeText("UCL:");
	LPUART1_writeHexWord(pacAddress);
	for( uint8_t a=1;a<5;a++ ){
		LPUART1_writeByte(';');
		LPUART1_writeHexWord( PAC1954_readUClimit( pacAddress, a ) );
	}
	LPUART1_writeText("\r\n");
}
void readOCLimits(){
	LPUART1_writeText("OCL:");
	LPUART1_writeHexWord(pacAddress);
	for( uint8_t a=1;a<5;a++ ){
		LPUART1_writeByte(';');
		LPUART1_writeHexWord( PAC1954_readOClimit( pacAddress, a ) );
	}
	LPUART1_writeText("\r\n");
}
void readOPLimits(){
	LPUART1_writeText("OPL:");
	LPUART1_writeHexWord(pacAddress);
	for( uint8_t a=1;a<5;a++ ){
		LPUART1_writeByte(';');
		LPUART1_writeHexWord( PAC1954_readOPlimit( pacAddress, a ) );
	}
	LPUART1_writeText("\r\n");
}
/* ************************************* E E P R O M ************************************************************** */
void resetSettings(){
	// Find pac
	_settings_in_ram.pacAddress = PAC1954_findAddress();
	_settings_in_ram.out1_lowvlim=10000;
	_settings_in_ram.out1_highvlim=16000;
	_settings_in_ram.out1_curlim=2000;

	_settings_in_ram.out2_lowvlim=10000;
	_settings_in_ram.out2_highvlim=16000;
	_settings_in_ram.out2_curlim=2000;

	_settings_in_ram.out3_lowvlim=10000;
	_settings_in_ram.out3_highvlim=16000;
	_settings_in_ram.out3_curlim=2000;

	_settings_in_ram.out4_lowvlim=10000;
	_settings_in_ram.out4_highvlim=16000;
	_settings_in_ram.out4_curlim=2000;

	_settings_in_ram.overcurrentTime=10;

	settings_write();
}
uint32_t settings_write(void){

	UnlockPELOCK();

    uint32_t *src = (uint32_t*)GLOBAL_settings_ptr;
    uint32_t *dst = (uint32_t*)DATA_E2_ADDR;

    //write settings word (uint32_t) at a time
    FLASH->PECR |= FLASH_PECR_DATA; /* (1) */
    for (uint32_t i = 0; i < sizeof(settings_t)/sizeof(uint32_t); i++){
        if (*dst != *src){ //write only if value has been modified
        	*(__IO uint32_t *)dst = *src;
        	__WFI(); /* (3) */
        }
        src++;
        dst++;
    }
    FLASH->PECR &= ~(FLASH_PECR_DATA); /* (4) */

    LockNVM();
    return 0;
}
/******************************************************************************/
/*            Cortex-M0 Plus Processor Interrupt Handlers                    */
/******************************************************************************/

void EXTI4_15_IRQHandler(void){
	if (EXTI->PR & (EXTI_PR_PIF4)) {
	    // Clear the EXTI status flag.
	    EXTI->PR |= (EXTI_PR_PIF4); // Writing a 1 clears it
	    HEART_ON;
	}
	if (EXTI->PR & (EXTI_PR_PIF6)) {
	    // Clear the EXTI status flag.
	    EXTI->PR |= (EXTI_PR_PIF6); // Writing a 1 clears it
	    HEART_ON;
	}
	if (EXTI->PR & (EXTI_PR_PIF7)) {
	    // Clear the EXTI status flag.
	    EXTI->PR |= (EXTI_PR_PIF7); // Writing a 1 clears it
	    HEART_ON;
	}
}
/******************************************************************************/
/*            Cortex-M0 Plus Processor Exceptions Handlers                    */
/******************************************************************************/
/* This function handles SysTick Handler.  */
void SysTick_Handler(void){
    Tick++;

	if( Tick % 10 == 0){
		if( pacState >= PAC_FOUND){
			check++;
		}
	}

    if( delay != 0 ){
    	delay--;
    	if( delay == 40 ){
    		delay --;
    	}
    }

    heartBeats++;
    if( heartBeats == 1000 ){ // Every second
    	HEART_ON;
    	heartBeats=0;
    }else if( heartBeats == 50){ // Stay on for 50ms
    	HEART_OFF;
    }
}
#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
