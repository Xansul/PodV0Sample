//******************************* POD V0 *********************************
//TYLER CALL 2023/2024

//*********** EXTERNAL IO PIN USAGE **************
//PC1: SPI MOSI
//PA10: SPI SS
//PA8: SPI RST
//PB10: SPI SCK
//PA12: SPI D/C
//PA0: ADC1 IN0
//PA4: DAC OUT1

//includes
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include <math.h> // eventually replace usage with lookup table

//********************************* GLOBAL VARIABLES *****************************

//********* RTOS *******

//display message queue
osMessageQueueId_t timeQHours;
osMessageQueueId_t timeQMinutes;

//update displayed time flag
osEventFlagsId_t eventFlagID;

//thread IDs
osThreadId_t updateTimeThreadID;
osThreadId_t writeTimeThreadID;
osThreadId_t getADC1ThreadID;
osThreadId_t handleAlarmThreadID;

//******** DMA ********

//DMA handles
DMA_HandleTypeDef hdma_spi2;
DMA_HandleTypeDef hdma_dac;

//******** RTC **********

//RTC handle
RTC_HandleTypeDef hrtc;

//array for currently displayed time - hours, minutes
uint8_t timeDisplay[2];

//********* TIMERS **********

//timer handles
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim2;

//time trackers
uint8_t hourPassed;
uint8_t minPassed;
uint8_t secPassed;

//alarm variables
uint8_t alarmHour;
uint8_t alarmMin;
uint8_t alarmSec;
uint8_t alarmLength;
uint8_t alarmTriggered;
uint8_t tempSec;

//alarm tracker
uint8_t alarmTriggeredSec;

//array for set time (set by user) - hours, minutes
uint8_t timeInput[2];

//********* ADC **********

//ADC handles
ADC_HandleTypeDef hadc1;

//********* DAC *********

//DAC handle
DAC_HandleTypeDef hdac;

//DAC input buffer
uint16_t inputDAC[DAC_SAMPLES];

//DAC temp value
uint16_t valueDAC;

//********* USART ********

//USART2 handle
UART_HandleTypeDef huart2;

//******* UTILITY ********

//define custom heap for FreeRTOS - is assigned in custom linker script
uint8_t  __attribute__((section(".ucHeap"))) ucHeap[ configTOTAL_HEAP_SIZE ];

//******* HAL DEFAULTS **********

//default task
osThreadId_t defaultTaskHandle;

//default task attributes
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

//********************************** FUNCTION PROTOTYPES *****************************

//******** RTOS ***********

void updateTimeThread(void *argument);
void writeTimeThread(void *argument);
void getADC1Thread(void *argument);
void handleAlarmThread(void *argument);

//******** RTC ***********

void RTC_Init();
void RTC_SetTime();

//******** DMA **********

void DMA_Init();
void DMA_SPI_Config();

//******* DISPLAY/SPI *******

//additional GPIO config for display/SPI
void displayConfigIO();
//SPI config
void displayConfigSPI();
//display write byte
void displayWrite8(uint8_t);
//display write 2 bytes
void displayWrite16(uint16_t);
//display send command
void displayWriteCommand(uint8_t data);
//display config
void displayInit();

//********* TIMERS **********

void TIM1_Config();
void TIM1_Start_IT();
void TIM3_Config();
void TIM3_Start_IT();
void TIM2_Config();
void TIM2_Start_IT();

//********** ADC ************

void ADC1_Config();

//********** DAC *********

void DAC_Config();
void DAC_Start();

//********** USART ***********
void USART2_Config();

//********* UTILITY *******

//overwrite newlib malloc/free to use FreeRTOS implementation - ensures thread safe use
void *malloc (size_t size);
void free (void *ptr);

//general IO config
void IO_GeneralConfig();

//********* HAL DEFAULTS ********

//system clock config
void SystemClock_Config(void);

//default task
void StartDefaultTask(void *argument);

//********************************** BEGIN MAIN *****************************

int main(void)
{
  //HAL default function calls
  HAL_Init();
  SystemClock_Config();

  //config function calls
  IO_GeneralConfig();
  USART2_Config();
  DMA_Init();
  displayConfigIO();
  displayConfigSPI();
  displayInit();
  TIM1_Config();
  TIM3_Config();
  TIM2_Config();
  ADC1_Config();
  DAC_Config();

  //start timers
  TIM1_Start_IT();
  TIM3_Start_IT();
  TIM2_Start_IT();

  //initialize variables
  //time trackers
  secPassed = 0U;
  minPassed = 0U;
  hourPassed = 0U;
  //time input
  timeInput[0] = 0U;
  timeInput[1] = 0U;
  //alarm variables - currently set at minute 1 for testing
  alarmHour = 0;
  alarmMin = 1;
  alarmSec = 0;
  //alarm trackers
  alarmLength = 10;
  alarmTriggered = 0;
  alarmTriggeredSec = 0;


  //scheduler init
  osKernelInitialize();

  //event flag creation
  eventFlagID = osEventFlagsNew(NULL);

  //********* THREAD CREATION

  //HAL default task thread
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  //update time thread *********** NOT USED
  //updateTimeThreadID = osThreadNew(updateTimeThread, NULL, &defaultTask_attributes);
  //osThreadSetPriority(updateTimeThreadID, osPriorityLow);

  //write time thread
  writeTimeThreadID = osThreadNew(writeTimeThread, NULL, &defaultTask_attributes);
  osThreadSetPriority(writeTimeThreadID, osPriorityLow);

  //poll ADC2 thread
  getADC1ThreadID = osThreadNew(getADC1Thread, NULL, &defaultTask_attributes);
  osThreadSetPriority(getADC1ThreadID, osPriorityLow);

  //handle alarm thread
  handleAlarmThreadID = osThreadNew(handleAlarmThread, NULL, &defaultTask_attributes);
  osThreadSetPriority(handleAlarmThreadID, osPriorityLow);

  //start scheduler
  osKernelStart();

  while (1)
  {
    //loop - should never reach here as control is taken by scheduler
  }

}

//********************************** FUNCTION DEFINITIONS *****************************

//********** RTOS ***********

//update time thread ****************** NOT USED
void updateTimeThread(void *argument)
{
	//loop
	while (1)
	{
		//time handle
		RTC_TimeTypeDef htime;

		//get time
		HAL_RTC_GetTime(&hrtc, &htime, RTC_FORMAT_BIN);

		//get date (to unlock/sync registers)
		RTC_DateTypeDef hdate;
		HAL_RTC_GetDate(&hrtc, &hdate, RTC_FORMAT_BIN);

		//check if RTC time is different than displayed time
		if ((htime.Hours != timeDisplay[0]) || (htime.Minutes != timeDisplay[1]))
		{
			//add to display queue - needs rework
			osMessageQueuePut(timeQHours, &htime.Hours, 0, 0);
			osMessageQueuePut(timeQMinutes, &htime.Minutes, 0, 0);

			//yield control
			osThreadYield();
		}
	}

	//precaution
	osThreadTerminate(osThreadGetId());
}

//write time thread - currently implemented through event flags
void writeTimeThread(void *argument)
{
	//loop
	while (1)
	{
		osEventFlagsWait(eventFlagID, TIME_UPDATE_READY, osFlagsWaitAll, osWaitForever);

		//write to display (currently USART for debug)
		char timeStr[20];
		sprintf(timeStr, "\r\n%d : %d : %d\r\n", hourPassed, minPassed, secPassed);
		HAL_UART_Transmit(&huart2, (uint8_t*)timeStr, strlen(timeStr), 10);

		//yield when complete
		osThreadYield();

		//message queue system **************** NOT USED
//		if ((osMessageQueueGetCapacity(timeQMinutesID) != 0) || (osMessageQueueGetCapacity(timeQHours) != 0))
//		{
//			//create time variables
//			uint8_t hours = 0;
//			uint8_t minutes = 0;
//
//			//pull time from queue
//			osMessageQueueGet(timeQHours, &hours, 0, 0);
//			osMessageQueueGet(timeQMinutes, &minutes, 0, 0);
//
//			//write to display (currently USART for debug)
//			char timeStr[10];
//			sprintf(timeStr, "\r\n%d : %d\r\n", hours, minutes);
//			HAL_UART_Transmit(&huart2, (uint8_t*)timeStr, strlen(timeStr), 10);
//
//			//update displayed time array
//			timeDisplay[0] = hours;
//			timeDisplay[1] = minutes;
//
//			//yield when complete
//			osThreadYield();
//		}
	}

	//precaution
	osThreadTerminate(osThreadGetId());
}

//thread to get ADC1 reading
void getADC1Thread(void *argument)
{
	//loop
	while (1)
	{
		//wait for flag
		osEventFlagsWait(eventFlagID, ADC1_UPDATE_READY, osFlagsWaitAll, osWaitForever);

		//get ADC1 value - currently reading ambient light level (higher = brighter)
		uint16_t rawVal = HAL_ADC_GetValue(&hadc1);

		//********** ALTERNATE FUNCTION - read internal temp (see datasheet) - set ADC1 channel to ADC_CHANNEL_TEMPSENSOR
		//uint16_t temp = ((float)rawVal / 4095) * 3300;
		//temp = ((temp - 760) / 2.5) +25;

		//write to display (currently USART for debug)
		char dataStr[50];
		sprintf(dataStr, "\r\nADC1: %d\r\n", rawVal);
		HAL_UART_Transmit(&huart2, (uint8_t*)dataStr, strlen(dataStr), 10);

		//yield when complete
		osThreadYield();
	}

	//precaution
	osThreadTerminate(osThreadGetId());
}

//thread to handle alarm
void handleAlarmThread(void *argument)
{
	//loop
	while (1)
	{
		//wait for flag - don't clear flag yet
		osEventFlagsWait(eventFlagID, ALARM_TRIGGER, osFlagsWaitAll | osFlagsNoClear, osWaitForever);

		//only start DAC if alarm hasn't been triggered yet
		if (alarmTriggered == 0)
		{
			//start DAC
			DAC_Start();

			//update alarm triggered time
			alarmTriggeredSec = secPassed;

			//set temp variable
			tempSec = alarmTriggeredSec + alarmLength;

			//check for overflow
			tempSec = tempSec > 59 ? (tempSec - 59U) : tempSec;

			//set alarm triggered variable
			alarmTriggered = 1;
		}

		//turn off alarm if alarm length has passed
		if (secPassed == tempSec)
		{
			//stop DAC
			HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);

			//reset alarm triggered variable
			alarmTriggered = 0;

			//clear flag
			osEventFlagsClear(eventFlagID, ALARM_TRIGGER);
		}

		//yield when complete
		osThreadYield();
	}

	//precaution
	osThreadTerminate(osThreadGetId());
}

//********* RTC *************

//RTC config **************** NOT USED
void RTC_Init()
{
	//RTC config
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_12;
	//set prescalers according to use of 32kHz LSI as clock source
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 249;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_PUSHPULL;
	HAL_RTC_Init(&hrtc);
}

//RTC set time ***************** NOT USED
void RTC_SetTime()
{
	//time handle config
	RTC_TimeTypeDef htime;

	//set init variables to global time array values
	htime.Hours = timeInput[0];
	htime.Minutes = timeInput[1];
	htime.TimeFormat = RTC_HOURFORMAT_12;

	//set time
	HAL_RTC_SetTime(&hrtc, &htime, RTC_FORMAT_BIN);

	//add to display queue
	osMessageQueuePut(timeQHours, &htime.Hours, 0, 100);
	osMessageQueuePut(timeQMinutes, &htime.Minutes, 0, 100);
}

//********* DMA *************

//DMA base config
void DMA_Init()
{
	//enable clock
	__HAL_RCC_DMA1_CLK_ENABLE();

	//******* DMA SPI2 config

	//DMA config
	hdma_spi2.Instance = DMA1_Stream4;
	hdma_spi2.Init.Channel = DMA_CHANNEL_0;
	hdma_spi2.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_spi2.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_spi2.Init.MemInc = DMA_MINC_ENABLE;
	hdma_spi2.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_spi2.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_spi2.Init.Mode = DMA_NORMAL;
	hdma_spi2.Init.Priority = DMA_PRIORITY_LOW;
	hdma_spi2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init(&hdma_spi2);

	//interrupt config
	HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

	//******* DMA DAC config

	//DMA config
	hdma_dac.Instance = DMA1_Stream5;
	hdma_dac.Init.Channel = DMA_CHANNEL_7;
	hdma_dac.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_dac.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_dac.Init.MemInc = DMA_MINC_ENABLE;
	hdma_dac.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	hdma_dac.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	hdma_dac.Init.Mode = DMA_CIRCULAR;
	hdma_dac.Init.Priority = DMA_PRIORITY_LOW;
	hdma_dac.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	HAL_DMA_Init(&hdma_dac);

	//interrupt config
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

//DMA SPI setup - currently unused/debugging
void DMA_SPI_Config()
{
	//******* SPI Setup


	//******* DMA Setup

	//reset CR and disable stream
	DMA1_Stream4->CR = 0x00;

	//reset FCR
	DMA1_Stream4->FCR = 0x21;

	//wait until actually disabled
	while(DMA1_Stream4->CR & DMA_SxCR_EN) {}

	//clear flags
	DMA1->LISR = 0x00;
	DMA1->HISR = 0x00;

	//setup temp register
	uint32_t tempReg = 0U;
	tempReg = DMA1_Stream4->CR;

	//write to temp register - all default except for MINC and transfer complete interrupt
	tempReg |= DMA_SxCR_MINC | DMA_SxCR_TCIE;
	DMA1_Stream4->CR = tempReg;

	//set destination address
	DMA1_Stream4->PAR = SPI2->DR;

}

//********* DISPLAY/SPI *************

//SPI GPIO config for display - currently only TX, will add RX at later point
void displayConfigIO()
{
	GPIO_InitTypeDef IOInit = {0};

	//enable clocks
	__HAL_RCC_SPI2_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	//MOSI output
	IOInit.Pin = GPIO_PIN_1;
	IOInit.Mode = GPIO_MODE_AF_PP;
	IOInit.Pull = GPIO_NOPULL;
	IOInit.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOC, &IOInit);

	//SCK output
	IOInit.Pin = GPIO_PIN_10;
	IOInit.Mode = GPIO_MODE_AF_PP;
	IOInit.Pull = GPIO_NOPULL;
	IOInit.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB, &IOInit);

	//set SCK to initial high
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

	//D/C output config
	IOInit.Pin = GPIO_PIN_12;
	IOInit.Mode = GPIO_MODE_OUTPUT_PP;
	IOInit.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &IOInit);

	//set D/C output to high (set to data transmission by default, changed by command function)
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);

	//SS output config
	IOInit.Pin = GPIO_PIN_10;
	IOInit.Mode = GPIO_MODE_OUTPUT_PP;
	IOInit.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &IOInit);

	//set SS output to low (only using one slave device)
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

	//RST output config
	IOInit.Pin = GPIO_PIN_8;
	IOInit.Mode = GPIO_MODE_OUTPUT_PP;
	IOInit.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &IOInit);

	//reset display and delay for reset
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_Delay(100);
}

//config SPI for display
void displayConfigSPI()
{
	//reset CR1
	SPI2->CR1 = 0x00;

	//software SS management
	SPI2->CR1 |= SPI_CR1_SSM;
	SPI2->CR1 |= SPI_CR1_SSI;

	//master select
	SPI2->CR1 |= SPI_CR1_MSTR;

	//polarity select - CPOL = 1 CPHA = 1
	SPI2->CR1 |= SPI_CR1_CPOL | SPI_CR1_CPHA;

	//enable
	SPI2->CR1 |= SPI_CR1_SPE;
}

//send single byte through FIFO buffer
void displayWrite8(uint8_t data)
{
	//wait for empty buffer
	while (!(SPI2->SR & SPI_SR_TXE)) {};

	//send byte
	*(uint8_t*)&(SPI2->DR) = data;
};

//send 2 bytes through FIFO buffer
void displayWrite16(uint16_t data)
{
	//wait for empty buffer
	while (!(SPI2->SR & SPI_SR_TXE)) {};

	//flip data (for -endian quirks)
	data = (((data & 0x00FF) << 8) | ((data & 0xFF00) >> 8));

	//send 2 bytes
	*(uint16_t*)&(SPI2->DR) = data;
}

//send command byte
void displayWriteCommand(uint8_t data)
{
	//wait for not busy
	while (SPI2->SR & SPI_SR_BSY) {};

	//pull D/C to low, transmit, pull D/C to high
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

	//transmit
	displayWrite8(data);

	//wait for busy
	while (SPI2->SR & SPI_SR_BSY) {};
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
}

//initialize display - sourced from Adafruit arduino display library (https://github.com/adafruit/Adafruit_ILI9341)
void displayInit()
{
	//display off
	displayWriteCommand(0x28);

	//initialization commands
	displayWriteCommand(0xEF);
	displayWrite8(0x03);
	displayWrite8(0x80);
	displayWrite8(0x02);
	displayWriteCommand(0xCF);
	displayWrite8(0x00);
	displayWrite8(0xC1);
	displayWrite8(0x30);
	displayWriteCommand(0xED);
	displayWrite8(0x64);
	displayWrite8(0x03);
	displayWrite8(0x12);
	displayWrite8(0x81);
	displayWriteCommand(0xE8);
	displayWrite8(0x85);
	displayWrite8(0x00);
	displayWrite8(0x78);
	displayWriteCommand(0xCB);
	displayWrite8(0x39);
	displayWrite8(0x2C);
	displayWrite8(0x00);
	displayWrite8(0x34);
	displayWrite8(0x02);
	displayWriteCommand(0xF7);
	displayWrite8(0x20);
	displayWriteCommand(0xEA);
	displayWrite8(0x00);
	displayWrite8(0x00);
	// PWCTR1
	displayWriteCommand(0xC0);
	displayWrite8(0x23);
	// PWCTR2
	displayWriteCommand(0xC1);
	displayWrite8(0x10);
	// VMCTR1
	displayWriteCommand(0xC5);
	displayWrite8(0x3E);
	displayWrite8(0x28);
	// VMCTR2
	displayWriteCommand(0xC7);
	displayWrite8(0x86);
	// MADCTL
	displayWriteCommand(0x36);
	displayWrite8(0x48);
	// VSCRSADD
	displayWriteCommand(0x37);
	displayWrite8(0x00);
	// PIXFMT
	displayWriteCommand(0x3A);
	displayWrite8(0x55);
	// FRMCTR1
	displayWriteCommand(0xB1);
	displayWrite8(0x00);
	displayWrite8(0x18);
	// DFUNCTR
	displayWriteCommand(0xB6);
	displayWrite8(0x08);
	displayWrite8(0x82);
	displayWrite8(0x27);
	displayWriteCommand(0xF2);
	displayWrite8(0x00);
	// GAMMASET
	displayWriteCommand(0x26);
	displayWrite8(0x01);
	// (Actual gamma settings)
	displayWriteCommand(0xE0);
	displayWrite8(0x0F);
	displayWrite8(0x31);
	displayWrite8(0x2B);
	displayWrite8(0x0C);
	displayWrite8(0x0E);
	displayWrite8(0x08);
	displayWrite8(0x4E);
	displayWrite8(0xF1);
	displayWrite8(0x37);
	displayWrite8(0x07);
	displayWrite8(0x10);
	displayWrite8(0x03);
	displayWrite8(0x0E);
	displayWrite8(0x09);
	displayWrite8(0x00);
	displayWriteCommand(0xE1);
	displayWrite8(0x00);
	displayWrite8(0x0E);
	displayWrite8(0x14);
	displayWrite8(0x03);
	displayWrite8(0x11);
	displayWrite8(0x07);
	displayWrite8(0x31);
	displayWrite8(0xC1);
	displayWrite8(0x48);
	displayWrite8(0x08);
	displayWrite8(0x0F);
	displayWrite8(0x0C);
	displayWrite8(0x31);
	displayWrite8(0x36);
	displayWrite8(0x0F);

	//exit sleep mode
	//display on
	displayWriteCommand(0x11);
	HAL_Delay(1000);
	displayWriteCommand(0x29);
	HAL_Delay(1000);
}

//********** TIMERS **************

//TIM1 config - used to track seconds, drives TIM3
void TIM1_Config()
{
	//***** Basic config

	//clock enable
	__TIM1_CLK_ENABLE();

	//CR1 config
	//set temp CR1 - all defaults with 0
	uint32_t tempcr1;
	tempcr1 = TIM1->CR1;
	tempcr1 &= 0x00000000U;
	//set CR1
	TIM1->CR1 = tempcr1;
	//ARR config
	TIM1->ARR = (uint32_t)41999;
	//PSC config
	TIM1->PSC = (uint32_t)1999;
	//RCR config
	TIM1->RCR = (uint32_t)0;
	//EGR config - generate update event
	TIM1->EGR = TIM_EGR_UG;
	//clear update flag
	TIM1->SR &= ~TIM_SR_UIF;

	//***** Clock config

	//SMCR config
	//set temp SMCR - all disabled (not using slave mode)
	uint32_t tempsmcr;
	tempsmcr = TIM1->SMCR;
	tempsmcr &= ~(TIM_SMCR_SMS | TIM_SMCR_TS);
	tempsmcr &= ~(TIM_SMCR_ETF | TIM_SMCR_ETPS | TIM_SMCR_ECE | TIM_SMCR_ETP);
	//set SMCR
	TIM1->SMCR = tempsmcr;

	//***** Master config

	//CR2 configs
	//set temp CR2 - master mode: update (trigger output on update event)
	uint32_t tempcr2;
	tempcr2 = TIM1->CR2;
	//clear and set MMS
	tempcr2 &= ~TIM_CR2_MMS;
	tempcr2 |= TIM_CR2_MMS_1;
	//set CR2
	TIM1->CR2 = tempcr2;

	//SMCR config 2
	uint32_t tempsmcr2;
	tempsmcr2 = TIM1->SMCR;
	//clear and set MSM
	tempsmcr2 &= ~TIM_SMCR_MSM;
	tempsmcr2 |= 0x00000000U;
	//set SMCR
	TIM1->SMCR = tempsmcr2;
}

//TIM3 config - used to track minutes, incremented on TIM1 update event
void TIM3_Config()
{
	//***** basic config

	//clock enable
	__TIM3_CLK_ENABLE();

	//CR1 config
	//set temp CR1 - all defaults with 0
	uint32_t tempcr1;
	tempcr1 = TIM3->CR1;
	tempcr1 &= 0x00000000U;

	//set CR1
	TIM3->CR1 = tempcr1;
	//ARR config
	TIM3->ARR = (uint32_t)59;
	//PSC config
	TIM3->PSC = (uint32_t)0;
	//RCR config
	TIM3->RCR = (uint32_t)0;
	//EGR config - generate update event
	TIM3->EGR = TIM_EGR_UG;
	//clear update flag
	TIM3->SR &= ~TIM_SR_UIF;

	//****** clock config

	//SMCR config
	//set temp SMCR
	uint32_t tempsmcr;
	tempsmcr = TIM3->SMCR;
	//set trigger - default ITR0
	tempsmcr &= ~TIM_SMCR_TS;
	//set slave mode - external clock mode 1
	tempsmcr |= (TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0);
	//set SMCR
	TIM3->SMCR = tempsmcr;

}

//TIM2 config - used to drive DAC frequency
void TIM2_Config()
{
	//***** Basic config

	//clock enable
	__TIM2_CLK_ENABLE();

	//CR1 config
	//set temp CR1 - all defaults with 0
	uint32_t tempcr1;
	tempcr1 = TIM2->CR1;
	tempcr1 &= 0x00000000U;
	//set CR1
	TIM2->CR1 = tempcr1;
	//ARR config
	TIM2->ARR = (uint32_t)4799;
	//PSC config
	TIM2->PSC = (uint32_t)0;
	//RCR config
	TIM2->RCR = (uint32_t)0;
	//EGR config - generate update event
	TIM2->EGR = TIM_EGR_UG;
	//clear update flag
	TIM2->SR &= ~TIM_SR_UIF;

	//***** Clock config

	//SMCR config
	//set temp SMCR - all disabled (not using slave mode)
	uint32_t tempsmcr;
	tempsmcr = TIM2->SMCR;
	tempsmcr &= ~(TIM_SMCR_SMS | TIM_SMCR_TS);
	tempsmcr &= ~(TIM_SMCR_ETF | TIM_SMCR_ETPS | TIM_SMCR_ECE | TIM_SMCR_ETP);
	//set SMCR
	TIM2->SMCR = tempsmcr;

	//***** Master config

	//CR2 configs
	//set temp CR2
	uint32_t tempcr2;
	tempcr2 = TIM2->CR2;
	//clear and set MMS  - master mode: update (trigger output on update event)
	tempcr2 &= ~TIM_CR2_MMS;
	tempcr2 |= TIM_CR2_MMS_1;
	//set CR2
	TIM2->CR2 = tempcr2;

	//SMCR config 2
	uint32_t tempsmcr2;
	tempsmcr2 = TIM2->SMCR;
	//clear and set MSM
	tempsmcr2 &= ~TIM_SMCR_MSM;
	tempsmcr2 |= 0x00000000U;
	//set SMCR
	TIM2->SMCR = tempsmcr2;
}

//TIM1 start
void TIM1_Start_IT()
{
	//enable interrupt in NVIC
	//set priority - keep >5 to avoid RTOS conflicts
	uint32_t prioritygroup = 0x00U;
	prioritygroup = NVIC_GetPriorityGrouping();
	NVIC_SetPriority(TIM1_UP_TIM10_IRQn, NVIC_EncodePriority(prioritygroup, 6, 6));

	//enable interrupt
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

	//enable interrupt on timer
	TIM1->DIER |= TIM_DIER_UIE;

	//start timer
	TIM1->CR1 |= TIM_CR1_CEN;
}

//TIM3 start
void TIM3_Start_IT()
{
	//enable interrupt in NVIC
	//set priority - keep >5 to avoid RTOS conflicts
	uint32_t prioritygroup = 0x00U;
	prioritygroup = NVIC_GetPriorityGrouping();
	NVIC_SetPriority(TIM3_IRQn, NVIC_EncodePriority(prioritygroup, 6, 6));

	//enable interrupt
	NVIC_EnableIRQ(TIM3_IRQn);

	//enable interrupt on timer
	TIM3->DIER |= TIM_DIER_UIE;

	//start timer
	TIM3->CR1 |= TIM_CR1_CEN;
}

//TIM2 start
void TIM2_Start_IT()
{
	//enable interrupt in NVIC
	//set priority - keep >5 to avoid RTOS conflicts
	uint32_t prioritygroup = 0x00U;
	prioritygroup = NVIC_GetPriorityGrouping();
	NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(prioritygroup, 6, 6));

	//enable interrupt
	NVIC_EnableIRQ(TIM2_IRQn);

	//enable interrupt on timer
	TIM2->DIER |= TIM_DIER_UIE;

	//start timer
	TIM2->CR1 |= TIM_CR1_CEN;
}

//********** ADC *************

//ADC1 config - currently reading ambient light level
void ADC1_Config()
{
	//enable clock
	__HAL_RCC_ADC1_CLK_ENABLE();

	//base config
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	HAL_ADC_Init(&hadc1);

	//channel config
	ADC_ChannelConfTypeDef chanConfig = {0};
	chanConfig.Channel = ADC_CHANNEL_0;
	chanConfig.Rank = 1;
	chanConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	HAL_ADC_ConfigChannel(&hadc1, &chanConfig);
}

//*********** DAC ************

//DAC config
void DAC_Config()
{
	//enable clock
	__HAL_RCC_DAC_CLK_ENABLE();

	//basic config
	hdac.Instance = DAC;
	hdac.DMA_Handle1 = &hdma_dac;
	HAL_DAC_Init(&hdac);

	//channel config - conversion synced with TIM2
	DAC_ChannelConfTypeDef chanConfig = {0};
	chanConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
	chanConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	HAL_DAC_ConfigChannel(&hdac, &chanConfig, DAC_CHANNEL_1);

	//link to DMA
	__HAL_LINKDMA(&hdac, DMA_Handle1, hdma_dac);

	//fill input buffer
	for (uint16_t i = 0; i < DAC_SAMPLES; i++)
	{
		//populating for sine wave - eventually replace with lookup table
		valueDAC = (uint16_t) rint((sinf(((2 * PI) / DAC_SAMPLES) * i) + 1) * 2048);

		//check for overflow
		inputDAC[i] = valueDAC < 4096 ? valueDAC : 4095;
	}
}

//DAC start - currently producing sine wave for fun, eventually drive audio for alarm
void DAC_Start()
{
	//enter critical section
	taskENTER_CRITICAL();

	//start DAC in DMA mode
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*) inputDAC, DAC_SAMPLES, DAC_ALIGN_12B_R);

	//exit critical section
	taskEXIT_CRITICAL();
}

//********* USART *************

//USART2 config - currently used for debug clock output
void USART2_Config()
{
	//basic config
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&huart2);
}

//********* UTILITY *************

//overwrite newlib malloc/free to use FreeRTOS implementation - ensures thread safe use
//malloc
void *malloc (size_t size)
{
	return pvPortMalloc(size);
}
//free
void free (void *ptr)
{
	vPortFree(ptr);
}

//general GPIO config
void IO_GeneralConfig()
{
	GPIO_InitTypeDef IOInit = {0};

	//enable clocks
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	//set initial output
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	//user button config
	IOInit.Pin = GPIO_PIN_13;
	IOInit.Mode = GPIO_MODE_IT_FALLING;
	IOInit.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &IOInit);

	//LED config
	IOInit.Pin = GPIO_PIN_5;
	IOInit.Mode = GPIO_MODE_OUTPUT_PP;
	IOInit.Pull = GPIO_NOPULL;
	IOInit.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &IOInit);
}

//*********** INTERRUPTS *************

//DMA IRQ handler
void DMA1_Stream4_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_spi2);
}

//DMA IRQ handler
void DMA1_Stream5_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_dac);
}

//TIM1 IRQ handler - called every second
void TIM1_UP_TIM10_IRQHandler(void)
{
	//toggle led
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

	//check for alarm time
	if (hourPassed == alarmHour && minPassed == alarmMin && secPassed == alarmSec)
	{
		//set flag
		osEventFlagsSet(eventFlagID, ALARM_TRIGGER);
	}

	//update seconds
	if (secPassed < 59)
	{
		secPassed++;
		osEventFlagsSet(eventFlagID, TIME_UPDATE_READY);
	}
	else
	{
		secPassed = 0U;
		osEventFlagsSet(eventFlagID, TIME_UPDATE_READY);
	}

	//poll ADC1 - ready to receive value when thread is called
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 2);

	//set ADC flag - currently checking ADC value every second
	osEventFlagsSet(eventFlagID, ADC1_UPDATE_READY);

	//clear interrupt flag
	TIM1->SR &= ~TIM_SR_UIF;
}

//TIM3 IRQ handler - called every minute
void TIM3_IRQHandler(void)
{
	//update time trackers
	//check/update minutes
	if (minPassed < 59)
	{
		minPassed++;
		osEventFlagsSet(eventFlagID, TIME_UPDATE_READY);
	}
	else
	{
		minPassed = 0U;
		osEventFlagsSet(eventFlagID, TIME_UPDATE_READY);

		//check/update hours
		if (hourPassed < 23)
		{
			hourPassed++;
			osEventFlagsSet(eventFlagID, TIME_UPDATE_READY);
		}
		else
		{
			hourPassed = 0U;
			osEventFlagsSet(eventFlagID, TIME_UPDATE_READY);
		}
	}

	//clear interrupt flag
	TIM3->SR &= ~TIM_SR_UIF;
}

//TIM2 IRQ handler
void TIM2_IRQHandler(void)
{
	//clear interrupt flag
	TIM2->SR &= ~TIM_SR_UIF;
}

//TIM callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6)
	{
		//used by HAL to update time base variable - DO NOT CHANGE
	    HAL_IncTick();
	}
}

//************* HAL DEFAULTS **************

//HAL default system clock config
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

//default task definition
void StartDefaultTask(void *argument)
{
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
}

//error handler
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
	  //can implement here
  }
}

//HAL asserts
#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
