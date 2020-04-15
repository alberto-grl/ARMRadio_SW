/*******************************************************************************

                   main.c module of the program ARM_Radio

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

/*******************************************************************************
  * @file    ADC_DualModeInterleaved/main.c
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    11-November-2013
  * @brief   This example provides a short description of how to use the ADC 
  *          peripheral to convert a regular channel in Dual interleaved mode 
  *          using DMA in mode 3 with 6Msps.
  ******************************************************************************/

/*
 * Download STSW-STM32138 STM32F429 discovery firmware package (UM1662)
 * https://my.st.com/content/my_st_com/en/products/embedded-software/mcu-mpu-embedded-software/stm32-embedded-software/stm32-standard-peripheral-library-expansion/stsw-stm32138.license=1586852341997.product=STSW-STM32138.version=1.0.1.html
 * and expand in C:\
 *
 *

 * CMSIS library in that package ( V3.20 ) cause an hardfault.
 * (https://community.arm.com/thread/6192 http://community.silabs.com/t5/32-bit-MCU/arm-rfft-fast-f32-crashes-controller/td-p/108929)
 * Mettere le 4.5 scaricate da ARM ma copiarci dentro la cartella Device delle CMSIS ST.
 * Download CMSIS 4.5 from https://github.com/ARM-software/CMSIS/archive/v4.5.0.zip
 * Rename C:\STM32F429I-Discovery_FW_V1.0.1\Libraries\CMSIS to C:\STM32F429I-Discovery_FW_V1.0.1\Libraries\CMSIS_FromST
 * Copy CMSIS 4.5 in C:\STM32F429I-Discovery_FW_V1.0.1\Libraries
 * Add the Device directory from ST's package
 * Copy C:\STM32F429I-Discovery_FW_V1.0.1\Libraries\CMSIS_FromST\Device to C:\STM32F429I-Discovery_FW_V1.0.1\Libraries\CMSIS\Device
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * LinkerScript.ld has been changed for CCM
 * MEMORY
{
  RAM (xrw)		: ORIGIN = 0x20000000, LENGTH = 192K
  ROM (rx)		: ORIGIN = 0x8000000, LENGTH = 2048K
  CCM (xrw)		: ORIGIN = 0x10000000, LENGTH = 64K
}
 * ++++++++++++
 * .ccm : {
  . = ALIGN(4);
  _sccm = .;
  *(.ccm)
  . = ALIGN(4);
  _eccm = .;
}>CCM

 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Check startup_stm32.s for all interrupt vectors
 *
 **************************************************************
 **************************************************************
 **************************************************************
 *Audio DAC out is on pin PA5
 *RF In ADC is on pin PC3
 *
 *******************************************************************************************************
 * printf for float should be enabled: https://www.openstm32.org/forumthread2108
 */

#define IN_MAIN

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "FIRcoefs.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/ 

  #define LINENUM            0x15
  #define FONTSIZE         Font12x12


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint16_t uhADCDualConvertedValue;
  
/* Private function prototypes -----------------------------------------------*/

#ifdef USE_LCD
static void Display_Init(void);
static void Display(void);
static void Touch(void);
#endif /* USE_LCD */


/* Private functions ---------------------------------------------------------*/

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int main(void)
{


  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       files (startup_stm32f429_439xx.s) before to branch to application main. 
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */

	// Configure the System clock for a frequency of 200 MHz
//	  SystemClock_Config();

	//TODO sembra non funzionare, per ora cambiato PLL_N    400 in system_stm32f4xx.c
//	 SetSysClockTo200();


#ifdef USE_LCD
  /* LCD Display init  */
  Display_Init();
#endif /* USE_LCD */


  volume= 0.1;


  // Set now the default values for some variables
    cwpitch = CWPITCH;
    os_time = 0;
    meanavg = 0.f;
    Qfactor = 0.987f;         // Q factor for the CW peak filter
    Muted   = false;
    AMindex  = LSBindex = 1;
    USBindex = CWindex  = 1;
    bw[AM]   = bw[LSB]  = Wide;
    bw[USB]  = bw[CW]   = Wide;
    agc[AM]  = agc[LSB] = Slow;
    agc[USB] = Slow;
    agc[CW]  = Fast;
    AGC_decay[Fast] = 0.9995f;
    AGC_decay[Slow] = 0.99995f;
    Hangcount[Fast] = 2;
    Hangcount[Slow] = 30;
    AgcThreshold    = 1.92e-4f;

  SDR_compute_IIR_parms();  // compute the IIR parms for the CW peak filter

 // init the decimating FIR control blocks
   arc = arm_fir_decimate_init_f32(&SfirR, NUMFIRCOEFS, 4, FIRcoefs, FIRstate1R, BSIZE*4);
   while(arc != ARM_MATH_SUCCESS);   // spin loop if error
   arc = arm_fir_decimate_init_f32(&SfirI, NUMFIRCOEFS, 4, FIRcoefs, FIRstate1I, BSIZE*4);
   while(arc != ARM_MATH_SUCCESS);   // spin loop if error

 // initialize the NVIC for PriorityGroup 4
   Set_NVIC_PriorityGroupConfig(PriorityGroup_4);
    Load_Presets();
    Tune_Preset(1);      // Set the initial tuning to Preset 1
    SDR_InitGPIO();
    SDR_InitEXTI();
    SDR_InitADC();
    SDR_InitDAC();
    SDR_StartADC();      // Start now the data acquisition/processing cycle



    /* Enable the LTDC */

    LTDC_Cmd(ENABLE);
    IOE_Config();

    DisplayFrequency();
    SetFstep(3);

  while (1)
  {  
  /* Display ADCs converted values on LCD */

	  #ifdef USE_LCD
	Touch();
    Display();
    #endif /* USE_LCD */
  }
}


void SetSysClockTo180(void)
{
  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/
  /* RCC system reset(for debug purpose) */

ErrorStatus HSEStartUpStatus;

  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if (HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(ENABLE);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_5);

    /* PLL configuration */
    RCC_PLLConfig(RCC_PLLSource_HSE, 8, 360, 2, 7);
    /* Enable PLL */
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
  else
  { /* If HSE fails to start-up, the application will have wrong clock configuration.
    User can add here some code to deal with this error */

    /* Go to infinite loop */
    while (1)
    {
    }
  }
}


void SetSysClockTo200(void)
{
  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/
  /* RCC system reset(for debug purpose) */

ErrorStatus HSEStartUpStatus;

 // RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if (HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(ENABLE);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_5);

    /* PLL configuration */
    RCC_PLLConfig(RCC_PLLSource_HSE, 8, 400, 2, 7);
    /* Enable PLL */
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
  else
  { /* If HSE fails to start-up, the application will have wrong clock configuration.
    User can add here some code to deal with this error */

    /* Go to infinite loop */
    while (1)
    {
    }
  }
}



#ifdef USE_LCD
/**
  * @brief  Display ADCs converted values on LCD
  * @param  None
  * @retval None
  */

 void DisplayFrequency(void)
{
	 uint8_t aTextBuffer[50];
	 sprintf((char*)aTextBuffer, "F %-4.3f KHz", (LOfreq/1000.0));
	    LCD_DisplayStringLine(LCD_LINE_3, (uint8_t*)aTextBuffer);
}

static void Touch(void)
{
	 static TP_STATE* TP_State;
	 static uint16_t Old_TP_State, AllowAutoRepeatCounter;

    TP_State = IOE_TP_GetState();

    if (Old_TP_State!=0)
    {
    	Old_TP_State=TP_State->TouchDetected;

    	if(AllowAutoRepeatCounter<20)
    	{
    		AllowAutoRepeatCounter++;
    		return;
    	}
    }
    else
    {
    	AllowAutoRepeatCounter=0;
    }

	Old_TP_State = TP_State->TouchDetected;

	if ((TP_State->TouchDetected) && (TP_State->Y <= 320)
			&& (TP_State->Y >= 320 - 36) && (TP_State->X >= 0)
			&& (TP_State->X <= 120)) {
		FminusClicked();
		DisplayFrequency();
	} else if ((TP_State->TouchDetected) && (TP_State->Y <= 320)
			&& (TP_State->Y >= 320 - 36) && (TP_State->X >= 121)
			&& (TP_State->X <= 240)) {
		FplusClicked();
		DisplayFrequency();
	} else if ((TP_State->TouchDetected) && (TP_State->Y <= 320 - 36)
			&& (TP_State->Y >= 320 - 2 * 36) && (TP_State->X >= 0)
			&& (TP_State->X <= 120)) {
		volume -= 0.1;
		if (volume < 0)
			volume = 0;
	} else if ((TP_State->TouchDetected) && (TP_State->Y <= 320 - 36)
			&& (TP_State->Y >= 320 - 2 * 36) && (TP_State->X >= 121)
			&& (TP_State->X <= 240)) {
		volume += 0.1;
		if (volume > 1.0)
			volume = 1.0;
	} else if ((TP_State->TouchDetected) && (TP_State->Y <= 320 - 36 * 2)
			&& (TP_State->Y >= 320 - 36 * 3) && (TP_State->X >= 0)
			&& (TP_State->X <= 60)) {
		SetFstep(4);
	} else if ((TP_State->TouchDetected) && (TP_State->Y <= 320 - 36 * 2)
			&& (TP_State->Y >= 320 - 36 * 3) && (TP_State->X >= 61)
			&& (TP_State->X <= 120)) {
		SetFstep(3);
	} else if ((TP_State->TouchDetected) && (TP_State->Y <= 320 - 36 * 2)
			&& (TP_State->Y >= 320 - 36 * 3) && (TP_State->X >= 121)
			&& (TP_State->X <= 180)) {
		SetFstep(2);
	} else if ((TP_State->TouchDetected) && (TP_State->Y <= 320 - 36 * 2)
			&& (TP_State->Y >= 320 - 36 * 3) && (TP_State->X >= 181)
			&& (TP_State->X <= 240)) {
		SetFstep(1);
	} else if ((TP_State->TouchDetected) && (TP_State->Y <= 320 - 36 * 3)
			&& (TP_State->Y >= 320 - 36 * 4) && (TP_State->X >= 0)
			&& (TP_State->X <= 60)) {
		SetMode(AM);
	} else if ((TP_State->TouchDetected) && (TP_State->Y <= 320 - 36 * 3)
			&& (TP_State->Y >= 320 - 36 * 4) && (TP_State->X >= 61)
			&& (TP_State->X <= 120)) {
		SetMode(LSB);
	} else if ((TP_State->TouchDetected) && (TP_State->Y <= 320 - 36 * 3)
			&& (TP_State->Y >= 320 - 36 * 4) && (TP_State->X >= 121)
			&& (TP_State->X <= 180)) {
		SetMode(USB);
	} else if ((TP_State->TouchDetected) && (TP_State->Y <= 320 - 36 * 3)
			&& (TP_State->Y >= 320 - 36 * 4) && (TP_State->X >= 181)
			&& (TP_State->X <= 240)) {
		SetMode(CW);
	} else if ((TP_State->TouchDetected) && (TP_State->Y <= 320 - 36 * 4)
			&& (TP_State->Y >= 320 - 36 * 5) && (TP_State->X >= 0)
			&& (TP_State->X <= 60)) {
		SetAGC(Fast);
	} else if ((TP_State->TouchDetected) && (TP_State->Y <= 320 - 36 * 4)
			&& (TP_State->Y >= 320 - 36 * 5) && (TP_State->X >= 61)
			&& (TP_State->X <= 120)) {
		SetAGC(Slow);
	} else if ((TP_State->TouchDetected) && (TP_State->Y <= 320 - 36 * 4)
			&& (TP_State->Y >= 320 - 36 * 5) && (TP_State->X >= 121)
			&& (TP_State->X <= 180)) {
		SetBW(Narrow);
	} else if ((TP_State->TouchDetected) && (TP_State->Y <= 320 - 36 * 4)
			&& (TP_State->Y >= 320 - 36 * 5) && (TP_State->X >= 181)
			&& (TP_State->X <= 240)) {
		SetBW(Wide);
	} else {
	}
}

static void Display(void)
{
  uint8_t aTextBuffer[50];
  float SValue;

  /* "The recommendation defines a difference of one S-unit corresponds to a difference of 6 decibels (dB),
   *  equivalent to a voltage ratio of two, or power ratio of four." https://en.wikipedia.org/wiki/S_meter
   */

  SValue = 10 / 3.01 * log10(PeakAudioValue * 2000.0);


  sprintf((char*)aTextBuffer, "S %-4.1f ", SValue);
  LCD_DisplayStringLine(LCD_LINE_2, (uint8_t*)aTextBuffer);

}

/**
  * @brief  Display Init (LCD)
  * @param  None
  * @retval None
  */
static void Display_Init(void)
{
	uint16_t n;
  /* Initialize the LCD */
  LCD_Init();
  LCD_LayerInit();

  /* Eable the LTDC */
  LTDC_Cmd(ENABLE);

  /* Set LCD Background Layer  */
  LCD_SetLayer(LCD_BACKGROUND_LAYER);

  /* Configure the transparency for background */
  LCD_SetTransparency(0);

  /* Set LCD Foreground Layer  */
  LCD_SetLayer(LCD_FOREGROUND_LAYER);

  /* Configure the transparency for foreground */
  LCD_SetTransparency(255);

  /* Clear the Foreground Layer */
  LCD_Clear(LCD_COLOR_BLACK);



  /* Set the LCD Back Color and Text Color*/
  LCD_SetBackColor(LCD_COLOR_BLACK);

  LCD_SetTextColor(LCD_COLOR_GREEN);
  LCD_SetFont(&Font12x12);

  LCD_DisplayStringLine(LINE(24), (uint8_t*) "  F-             F+");
  LCD_DisplayStringLine(LINE(21), (uint8_t*) "  V-             V+");
  LCD_DisplayStringLine(LINE(18), (uint8_t*) "10   100   1K   10K");
  LCD_DisplayStringLine(LINE(15), (uint8_t*) "AM   LSB   USB   CW");
  LCD_DisplayStringLine(LINE(12), (uint8_t*) "FAST SLOW  NRRW WIDE");

  for (n=0; n<6; n++)
  {
	  LCD_DrawLine(0, 319 - n*36, 240, LCD_DIR_HORIZONTAL);
  }
  LCD_DrawLine(120, 319 - 5*36, 5*36, LCD_DIR_VERTICAL);
  LCD_DrawLine(60, 319 - 5*36, 3*36, LCD_DIR_VERTICAL);
  LCD_DrawLine(180, 319 - 5*36, 3*36, LCD_DIR_VERTICAL);
  LCD_SetFont(&Font16x24);

}
#endif /* USE_LCD */


//-----------------------------------------------------------------------------



#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
