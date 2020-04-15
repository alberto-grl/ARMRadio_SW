/*******************************************************************************

                   SDR_InitPeriph.c module of the program ARM_Radio
						                          
						                          Copyright 2015 by Alberto I2PHD, June 2015
						                                      
    This file is part of ARM_Radio.

    ARM_Radio is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ARM_Radio is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with ARM_Radio.  It is contained in the file Copying.txt in the
    same ZIP file where this file was extracted from.				                                      
						                                      
*******************************************************************************/

// In this module all the components of the ARM chip that are used by this application
// are initialized

#include "main.h"
//#include "periphs.h"

/*
//-----------------------------------------------------------------------------
void SYSCFG_EXTILineConfig(uint8_t EXTI_PortSourceGPIOx, uint8_t EXTI_PinSourcex)
{
  uint32_t tmp = 0x00;

  tmp = ((uint32_t)0x0F) << (0x04 * (EXTI_PinSourcex & (uint8_t)0x03));
  SYSCFG->EXTICR[EXTI_PinSourcex >> 0x02] &= ~tmp;
  SYSCFG->EXTICR[EXTI_PinSourcex >> 0x02] |= 
  (((uint32_t)EXTI_PortSourceGPIOx) << (0x04 * (EXTI_PinSourcex & (uint8_t)0x03)));
}
*/

//-----------------------------------------------------------------------------
void Set_NVIC_PriorityGroupConfig(uint32_t PriorityGroup)
{
  /* Set the PRIGROUP[10:8] bits according to NVIC_PriorityGroup value */
  SCB->AIRCR = AIRCR_VECTKEY_MASK | PriorityGroup;
}
//-----------------------------------------------------------------------------
void SDR_InitEXTI(void)
{
	EXTI_StructInit(&EXTI_InitStructure);   // set default values

// Enable and set EXTI Line0 Interrupt for the User push-button

  /* Connect EXTI Line0 to PA0 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

  /* Configure EXTI Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;   
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
// Enable and set EXTI Line1 Interrupt for the base band processing routine
  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);	
	
	NVIC_ClearPendingIRQ(EXTI0_IRQn); 
	NVIC_ClearPendingIRQ(EXTI1_IRQn); 
	NVIC_EnableIRQ(EXTI0_IRQn);  
	NVIC_EnableIRQ(EXTI1_IRQn);  
	
	EXTI->IMR |= EXTI_Line1;
}	
//-----------------------------------------------------------------------------
// Init here the various GPIO ports used



/**
  * @brief  Initializes the GPIOx peripheral according to the specified parameters in the GPIO_InitStruct.
  * @param  GPIOx: where x can be (A..I) to select the GPIO peripheral.
  * @param  GPIO_InitStruct: pointer to a GPIO_InitTypeDef structure that contains
  *         the configuration information for the specified GPIO peripheral.
  * @retval None
  */
void SDR_GPIO_Init(GPIO_TypeDef* GPIOx, SDR_GPIO_InitTypeDef* GPIO_InitStruct)
{
  uint32_t pinpos = 0x00, pos = 0x00 , currentpin = 0x00;

  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_InitStruct->GPIO_Pin));
  assert_param(IS_GPIO_MODE(GPIO_InitStruct->GPIO_Mode));
  assert_param(IS_GPIO_PUPD(GPIO_InitStruct->GPIO_PuPd));

  /* -------------------------Configure the port pins---------------- */
  /*-- GPIO Mode Configuration --*/
  for (pinpos = 0x00; pinpos < 0x10; pinpos++)
  {
    pos = ((uint32_t)0x01) << pinpos;
    /* Get the port pins position */
    currentpin = (GPIO_InitStruct->GPIO_Pin) & pos;

    if (currentpin == pos)
    {
      GPIOx->MODER  &= ~(GPIO_MODER_MODER0 << (pinpos * 2));
      GPIOx->MODER |= (((uint32_t)GPIO_InitStruct->GPIO_Mode) << (pinpos * 2));

      if ((GPIO_InitStruct->GPIO_Mode == GPIO_Mode_OUT) || (GPIO_InitStruct->GPIO_Mode == GPIO_Mode_AF))
      {
        /* Check Speed mode parameters */
        assert_param(IS_GPIO_SPEED(GPIO_InitStruct->GPIO_Speed));

        /* Speed mode configuration */
        GPIOx->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (pinpos * 2));
        GPIOx->OSPEEDR |= ((uint32_t)(GPIO_InitStruct->GPIO_Speed) << (pinpos * 2));

        /* Check Output mode parameters */
        assert_param(IS_GPIO_OTYPE(GPIO_InitStruct->GPIO_OType));

        /* Output mode configuration*/
        GPIOx->OTYPER  &= ~((GPIO_OTYPER_OT_0) << ((uint16_t)pinpos)) ;
        GPIOx->OTYPER |= (uint16_t)(((uint16_t)GPIO_InitStruct->GPIO_OType) << ((uint16_t)pinpos));
      }

      /* Pull-up Pull down resistor configuration*/
      GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << ((uint16_t)pinpos * 2));
      GPIOx->PUPDR |= (((uint32_t)GPIO_InitStruct->GPIO_PuPd) << (pinpos * 2));
    }
  }
}


void SDR_InitGPIO()
{
	SDR_GPIO_InitTypeDef  GPIO_InitStructure;

	GPIO_StructInit(&GPIO_InitStructure);   // set default values
// Enable SYSCFG clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
// Enable GPIOA clock.  GPIOA is used to read the User push-button
// and for the DAC output	
  __GPIOA_CLK_ENABLE();

// Enable GPIOF clock.  GPIOF is used to toggle the PF8 bit 
// connected to the 'scope
  __GPIOF_CLK_ENABLE();
	
// Enable GPIOG Ports clock. GPIOG is used for the two green and red leds  
  __GPIOG_CLK_ENABLE();

// Configure PA0 pin as input floating for the User push-button
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  SDR_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
// Configure PA5 for DAC channel 2 (DAC_OUT2)
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  SDR_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
// Configure PF8 in output pushpull mode for the oscilloscope measurements
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;   // low slew rate
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  SDR_GPIO_Init(GPIOF, &GPIO_InitStructure);

// Configure PC5 (ADC Channel 15) as analog input
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  SDR_GPIO_Init(GPIOC, &GPIO_InitStructure);

// Configure GPIO pins: PG13 PG14 for the green and red leds
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  SDR_GPIO_Init(GPIOG, &GPIO_InitStructure);
}

//-----------------------------------------------------------------------------
/**
  *         System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 200000000
  *            HCLK(Hz)                       = 200000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 400
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  */
//-----------------------------------------------------------------------------

/*

void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

	HAL_RCC_DeInit();
// Enable Power Control clock
  __PWR_CLK_ENABLE();
  
// Enable HSE Oscillator and activate PLL with HSE as source
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM       = 8;
  RCC_OscInitStruct.PLL.PLLN       = 400;
  RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ       = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
// Initialization Error - should never land here....
    Error_Handler();
  }
  
// Select PLL as system clock source and configure the 
// HCLK, PCLK1 and PCLK2 clocks dividers
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
// Initialization Error - should never land here....
    Error_Handler();
  }
}


*/


//-----------------------------------------------------------------------------
// Init ADC1 and ADC2 in interleaved mode
void SDR_InitADC(void)
{

#define ADC_CDR_ADDRESS    ((uint32_t)0x40012308)

  SDR_ADC_InitTypeDef   ADC_InitStructure;
	SDR_DMA_InitTypeDef   DMA_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;

	SDR_ADC_StructInit(&ADC_InitStructure);         // set default values
	ADC_CommonStructInit(&ADC_CommonInitStructure); // set default values
	DMA_StructInit(&DMA_InitStructure);             // set default values

//*****************************************************************************/
//               ADCs interface clock, pin and DMA configuration               /
//*****************************************************************************/
       
// Enable peripheral clocks
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2, ENABLE);

// Configure the Interrupt Controller for DMA requests
	NVIC_InitStructure.NVIC_IRQChannel  = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_ClearPendingIRQ(DMA2_Stream0_IRQn);
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);  
	
// configure the DMA2 Stream0 channel0
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC_CDR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_Data0;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = BSIZE/2; // each DMA transfer sends 2 ADC results
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);

// configure DMA for double buffering mode (flip-flop)
  DMA_DoubleBufferModeConfig(DMA2_Stream0, (uint32_t)&ADC_Data1, DMA_Memory_0);
  DMA_DoubleBufferModeCmd(DMA2_Stream0, ENABLE);
  DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE); 

// enable now DMA2_Stream0
  DMA_Cmd(DMA2_Stream0, ENABLE);
  while(DMA_GetCmdStatus(DMA2_Stream0) == DISABLE);  // wait until enabled

//*******************************************/
//  ADCs configuration: double interleaved   /
//*******************************************/

// ADC Common configuration 
  ADC_CommonInitStructure.ADC_Mode = ADC_DualMode_Interl;
// delay 12 => 2.083333.. MS/sec 
// delay 13 => 1.923077.. MS/sec
// delay 14 => 1.785714.. MS/sec  we use this delay
// delay 15 => 1.666667.. MS/sec
// delay 16 => 1.562500   MS/sec
// delay 18 => 1.388889.. MS/sec
// delay 20 => 1.250000   MS/sec
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_14Cycles;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_2; // halfword by halfword
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;      // then fADC = 25 MHz
  ADC_CommonInit(&ADC_CommonInitStructure);
	
// valid only for ADC_Prescaler_Div4  
// and ADC_SampleTime_3Cycles 
// and ADC_Resolution_12b and delay = 14
	SamplingRate = ((SystemCoreClock / 2) / 4) / 14.f; 
  AudioRate = SamplingRate / 16.f / 4.f; 
  
// ADC1 regular channel 15 configuration
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  SDR_ADC_Init(ADC1, &ADC_InitStructure);

  ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_3Cycles);

// Enable ADC1 DMA
  ADC_DMACmd(ADC1, ENABLE);

// ADC2 regular channel 15 configuration
  SDR_ADC_Init(ADC2, &ADC_InitStructure);
  ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 1, ADC_SampleTime_3Cycles);
  
// Enable DMA request after last transfer (multi-ADC mode)
  ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

// Enable ADC1 **************************************************************
  ADC_Cmd(ADC1, ENABLE);

// Enable ADC2 **************************************************************
  ADC_Cmd(ADC2, ENABLE);
 }
//----------------------------------------------------------------------------
void SDR_StartADC()
{   
// Start ADC1 Software Conversion
  ADC_SoftwareStartConv(ADC1);
}
//----------------------------------------------------------------------------
// Configure here the parameters for Timer6, used to trigger the output DAC
static void TIM6_Config(void)
{
  TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
// TIM6 Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  
/* --------------------------------------------------------
  TIM6 input clock (TIM6CLK) is set to 2 * APB1 clock (PCLK1), 
  since APB1 prescaler is different from 1.   
    TIM6CLK = 2 * PCLK1  
    TIM6CLK = HCLK / 2 = SystemCoreClock /2 
          
  TIM6 Update event occurs each TIM6CLK/3584 which means :
	   APB1 = HCLK/4, i.e. 200 MHz / 4 = 50 MHz
	   TIM6CLK = 2 * APB1 = 100 MHz
     TIM6 update event = 100e6 / 3584 = 27901.786 Hz, 
                                        which is the output sampling frequency 	
----------------------------------------------------------- */
// Time base configuration
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 

// reload value = 3584 :  100MHz / 3584 = 27901.786 Hz
// which is the sampling freq. of the DAC, and also 1/4
// of the freq. after the decimation...  1.7857143e6 / 16 = 111607.143 Hz 
// and 111607.143 / 4 = 27901.786 Hz (AudioRate)

  TIM_TimeBaseStructure.TIM_Period = (uint32_t) (100.e6f / AudioRate) - 1; //rimettere 100.e6f
  TIM_TimeBaseStructure.TIM_Prescaler = 0;       
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;   
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

// TIM6 TRGO selection
  TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);
  
// enable TIM6 counter
  TIM_Cmd(TIM6, ENABLE);
}
//-----------------------------------------------------------------------------
// Init here the output DAC, which uses DMA1 Stream 6 in double buffered mode 
void SDR_InitDAC()
{  
  SDR_DMA_InitTypeDef DMA_InitStructure;
  DAC_InitTypeDef DAC_InitStructure;
  
// DMA1 clock enable
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
                        
// DAC Periph clock enable
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

// TIM6 Configuration
  TIM6_Config();  

// Enable and set DMA1 Stream6 Interrupt for the DAC 
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);	
  
	NVIC_ClearPendingIRQ(DMA1_Stream6_IRQn); 
	NVIC_EnableIRQ(DMA1_Stream6_IRQn);  
  
  DAC_StructInit(&DAC_InitStructure);    // set default values         
	DMA_StructInit(&DMA_InitStructure);    // set default values

// DAC channel2 Configuration
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
  DAC_Init(DAC_Channel_2, &DAC_InitStructure);

// DMA1_Stream6 channel7 configuration
  DMA_DeInit(DMA1_Stream6);
  DMA_InitStructure.DMA_Channel = DMA_Channel_7;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)DAC_DHR12R2_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&AudioOut0;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize = BSIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream6, &DMA_InitStructure);

// set the double buffered mode (flip-flop) for DMA1 Stream 6
  DMA_DoubleBufferModeConfig(DMA1_Stream6, (uint32_t)&AudioOut1, DMA_Memory_0);
  DMA_DoubleBufferModeCmd(DMA1_Stream6, ENABLE);

// Enable DMA1_Stream6
  DMA_Cmd(DMA1_Stream6, ENABLE);
  while(DMA_GetCmdStatus(DMA1_Stream6) == DISABLE);  // wait until enabled

// Enable DMA for DAC Channel2
  DAC_DMACmd(DAC_Channel_2, ENABLE);

// Enable DAC Channel2
  DAC_Cmd(DAC_Channel_2, ENABLE);
}
//----------------------------------------------------------------------------

