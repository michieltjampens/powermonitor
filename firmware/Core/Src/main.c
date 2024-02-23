/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "main.h"

/* Shared Variables  ---------------------------------------------------------*/
__IO uint32_t Tick;
__IO uint16_t error;

/* Private variables ---------------------------------------------------------*/

uint8_t pacAddress=0x00;
uint16_t heartBeats=0;

uint8_t delay;
uint16_t check=0;
uint8_t pacState=0;

static PacSettings pacSettings;
PacSettings *GLOBAL_settings_ptr = &pacSettings;

static VoltageCurrent lastVoltCur[PAC_CHANNELS];

PacChannel chSettings[PAC_CHANNELS];

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
			readVCdata();
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
    //loadSettings();
    pacAddress = 0x1D;//pacSettings.pacAddress; // Get the read address

    // Check if I2C hardware is found
    LPUART1_writeText("I>PAC1954:");
    temp=I2C1_PokeDevice(pacAddress);
    if( temp==1){
    	LPUART1_writeText("OK\r\n");
    	//PAC1954_clearAlertEnable(pacAddress);  // Make sure alerts are cleared
    	//PAC1954_applySettings( &pacSettings, chSettings,PAC_CHANNELS);
    	//PAC1954_doRefreshV(pacAddress);
    	delay = 5; // wait a bit
    	while(delay!=0); // 5ms delay
    	readVCdata(pacAddress);
    	pacState=PAC_FOUND;
    	check=0;
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
    PB0 = PAC1954 PWRDN
  */
  GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE5))| (GPIO_MODER_MODE5_0);
  GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE0))| (GPIO_MODER_MODE0_0);

}

/* ******************************************** I 2 C ******************************************************** */

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
/* ***************************************** C L I ******************************************************* */
void executeCommand( uint8_t * cmd ){
    uint8_t ok = 0xFF;
      /*
       * EEPROM
       * er -> Reset settings to their defaults (doesn't store)
       * el -> Load settings from eeprom, overwriting current ones
       * es -> Store settings to eeprom
       * SET LIMIT AND ENABLE THE ALERT of channel x with (decimal) value y
       * - if y is 0 disables it instead
       * - This doesn't store it to eeprom, so do 'es' when ready.
       * sovx:yyyy -> Overvoltage
       * socx:yyyy -> Overcurrent
       * sopx:yyyy -> Overpower
       * suvx:yyyy -> Undervoltage
       * sucx:yyyy -> Undercurrent
       * READ STUFF
	   * uv -> read Under Voltage limits
	   * uc -> read Under Current Limits
	   * ov -> Read Over voltage limits
	   * oc -> Read Over current limits
	   * op -> read Over Power limits
	   * as -> Read alert status
       * ae -> Read alert enable
       * acx -> Read accumulator for channel x
	   */
    switch( cmd[0]){
      case 'e': // E²PROM
    	  switch(cmd[1]){
    	  	  case 'r': resetSettings(); break;
    	  	  case 'l': loadSettings();  break;
    	  	  case 'w': storeSettings(); break;
    	  }
      case 's': // Set data
    	  if( cmd[4]!=':'){
    		  ok=0x00;
    		  break;
    	  }
    	  switch(cmd[1]){
    	  	  case 'o':
    	  		  switch(cmd[2]){
    	  		  	  case 'v': applyLimit( OV,cmd[3]-'0',cmd+5 ); break;
    	  		  	  case 'c': applyLimit( OC,cmd[3]-'0',cmd+5 ); break;
    	  		  	  case 'p': applyLimit( OP,cmd[3]-'0',cmd+5 ); break;
    	  		  	  default: ok=0x00; break;
    	  		  }
    	  		  break;
    	  	  case 'u':
    	  		  switch(cmd[2]){
	  		  	  	  case 'v': applyLimit( UV,cmd[3]-'0',cmd+5 ); break;
	  		  	  	  case 'c': applyLimit( UC,cmd[3]-'0',cmd+5 ); break;
    	  		  	  default: ok=0x00; break;
    	  		  }
    	  		  break;
    	  	 default: ok=0x00; break;
    	  }
    	  break;
      case 'p': // Send last voltage and sense reading for all channels
    	  readVCdata(pacAddress);
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
    	  	  case 'c':
    	  		  cmd[2]=cmd[2]-'0';
    	  		  if( cmd[2] <= PAC_CHANNELS)
    	  		  	  readAccumulator(cmd[2]);
    	  	  	  break;
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
void findI2CDevices(){

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
uint8_t applyLimit( uint8_t reg, uint8_t chn, uint8_t * lmt){
	uint16_t value=0x00;
	uint8_t  ok=0x01;

	// First convert lmt to the value...
	uint8_t maxdigits=6; // To limit the out of bounds possibility
	while( *lmt != 0x00 && maxdigits>0 ){
		value += *lmt - '0';
		lmt++;
		if( *lmt != 0x00 ) // If this wasn't the last digit, shift it
			value *= 10;
		maxdigits--;
	}
	if( maxdigits==0 )
		return 0x22;

    if( chn <= PAC_CHANNELS){
	   switch(reg){
			case UV: chSettings[chn-1].UV_lim = value; break;
			case OV: chSettings[chn-1].OV_lim = value; break;
			case UC: chSettings[chn-1].UC_lim = value; break;
			case OC: chSettings[chn-1].OC_lim = value; break;
			case OP: chSettings[chn-1].OP_lim = value; break;
		}
    }else{
    	return 0x23; // return that the given ch was incorrect
    }

	uint32_t macro;
	switch(reg){
		case UV:
			ok=PAC1954_setUVLimit( pacAddress,chn,value );
			macro = PAC_UV_ALERT_EN_CH1 >> (chn-1);
			break;
		case OV:
			ok=PAC1954_setOVLimit( pacAddress,chn,value );
			macro = PAC_OV_ALERT_EN_CH1 >> (chn-1);
			break;
		case UC:
			ok=PAC1954_setUCLimit( pacAddress,chn,value );
			macro = PAC_UC_ALERT_EN_CH1 >> (chn-1);
			break;
		case OC:
			ok=PAC1954_setOCLimit( pacAddress,chn,value );
			macro = PAC_OC_ALERT_EN_CH1 >> (chn-1);
			break;
		case OP:
			ok=PAC1954_setOPLimit( pacAddress,chn,value );
			macro = PAC_OP_ALERT_EN_CH1 >> (chn-1);
			break;
	}
	if( ok == I2C_OK){
		if( value==0){
			PAC1954_disableAlerts( pacAddress,macro );
		}else{
			PAC1954_enableAlerts( pacAddress,macro );
		}
	}
	return ok;
}
/* ***************************************** P A C 1 9 5 X  ******************************************************* */
void readVCdata(uint8_t address){
	uint8_t res = PAC1954_readAvgVoltageCurrent( address, lastVoltCur );
	if( res==I2C_OK ){
			LPUART1_writeText("VC:");
			LPUART1_writeHexWord(address);
			for( uint8_t a=0;a<PAC_CHANNELS;a++ ){
				LPUART1_writeByte(';');
				LPUART1_writeHexWord(lastVoltCur[a].voltage);
				LPUART1_writeByte(';');
				LPUART1_writeHexWord(lastVoltCur[a].current);
			}
			LPUART1_writeText("\r\n");
			pacState=PAC_FOUND;
	}else{
		if( res == ERROR_I2C_NACK){
			pacState=PAC_NAK;
			if( I2C1_PokeDevice(pacAddress) == I2C_OK ){
				pacState=PAC_FOUND;
			}
		}
		pacState=PAC_FOUND;
		LPUART1_writeText("I2C:VC Error->");
		printI2Cerror(res);
		LPUART1_writeText("\r\n");
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
			case 1:LPUART1_writeText(";1;"); break;
			case 2:LPUART1_writeText(";2;"); break;
			case 3:LPUART1_writeText(";3;"); break;
			case 4:LPUART1_writeText(";4;"); break;
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
	}else{
		LPUART1_writeText("I2C:AC Error->");
		printI2Cerror(res);
	}
	LPUART1_writeText("\r\n");
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
	pacSettings.pacAddress = PAC1954_findAddress();

	for( uint8_t a=0;a<PAC_CHANNELS;a++){
		chSettings[a].OV_lim = 0x00;
		chSettings[a].UV_lim = 0x00;
		chSettings[a].OC_lim = 0x00;
		chSettings[a].UC_lim = 0x00;
		chSettings[a].OP_lim = 0x00;
	}

	pacSettings.overcurrentTime=10;
	pacSettings.alerts_enable=0x00;

	storeSettings();
}
uint8_t loadSettings(void){
	for( uint8_t a=0;a<PAC_CHANNELS;a++){
		if( sizeof(chSettings[a]) % 4 ==0){ // Needs to be a multiple of 4 because e²prom has 32bit pages
			// Get the settings from e²prom
			memcpy(GLOBAL_settings_ptr, (uint32_t*)DATA_E2_ADDR, sizeof(chSettings[a]));

			if( a==1 && chSettings[a].pacAddress==0x00){ // nothing in there yet
				resetSettings();
				return 0x02;
			}
			return 0x01;
		}
	}
	return 0x00;
}
uint8_t storeSettings(void){

	UnlockPELOCK();

    uint32_t *dst = (uint32_t*)DATA_E2_ADDR;

    //write settings word (uint32_t) at a time
    FLASH->PECR |= FLASH_PECR_DATA; /* (1) */
    for( uint8_t a=0;a<PAC_CHANNELS;a++ ){
    	uint32_t *src = (uint32_t*)&chSettings[a];
		for (uint32_t i = 0; i < sizeof(chSettings[a])/sizeof(uint32_t); i++){
			if (*dst != *src){ //write only if value has been modified
				*(__IO uint32_t *)dst = *src;
				__WFI(); /* (3) */
			}
			src++;
			dst++;
		}
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
