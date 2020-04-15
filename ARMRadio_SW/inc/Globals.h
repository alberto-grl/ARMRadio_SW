/*******************************************************************************

                   Globals.h module of the program ARM_Radio
						                          
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GLOBALS_H
#define __GLOBALS_H

// allocate actual memory only when invoked from the main.c module
#ifdef IN_MAIN
 #define EXTERN
#else
 #define EXTERN extern
#endif  
 
#include "stdint.h" 
#include "stm32f4xx.h"
#include "main.h"
#include "arm_math.h"



 
// place the following variables in the CCM section of RAM ------------------
// ****AG CCM should be defined in LinkerScript.ld
EXTERN float Rbasedata[BSIZE*4] __attribute__ ((section (".ccm")));
EXTERN float Ibasedata[BSIZE*4] __attribute__ ((section (".ccm")));
EXTERN float Rbase[BSIZE*4]     __attribute__ ((section (".ccm")));
EXTERN float Ibase[BSIZE*4]     __attribute__ ((section (".ccm")));
EXTERN float Rdata[BSIZE]       __attribute__ ((section (".ccm")));
EXTERN float Idata[BSIZE]       __attribute__ ((section (".ccm")));
EXTERN float FFTmask[FFTLEN*2]  __attribute__ ((section (".ccm")));
EXTERN float FFTbuf[FFTLEN*2]   __attribute__ ((section (".ccm")));
EXTERN float FFTbuf2[FFTLEN*2]  __attribute__ ((section (".ccm")));
// end of CCM placing --------------------------------------------------------

EXTERN volatile uint16_t ADC_Data0[BSIZE];
EXTERN volatile uint16_t ADC_Data1[BSIZE];

#pragma pack(16)
EXTERN float ADC_Rdata[BSIZE];
EXTERN float ADC_Idata[BSIZE];
EXTERN float IQdata[BSIZE*2];             // IQdata  is a complex signal
EXTERN float fCbase[FFTLEN*2];            // fCbase  is a complex signal
EXTERN float tmpSamp[BSIZE*2+12];         // tmpSamp is a complex signal
EXTERN float LO_R[BSIZE], LO_I[BSIZE];    // LO is a complex signal
EXTERN float fAudio[BSIZE];
EXTERN short AudioOut0[BSIZE];
EXTERN short AudioOut1[BSIZE];

EXTERN float FIRstate1R[NUMFIRCOEFS + BSIZE*4 - 1];
EXTERN float FIRstate1I[NUMFIRCOEFS + BSIZE*4 - 1];
EXTERN Agctype agc[4];
EXTERN Bwidth  bw[4];
EXTERN Presets psets[MAXPRESETS];

EXTERN arm_status arc;
EXTERN arm_fir_decimate_instance_f32 SfirR;
EXTERN arm_fir_decimate_instance_f32 SfirI;

EXTERN float TestSampledValue;

EXTERN float     volume, Qfactor, a1, a2, b0, cwpitch, audiotmp,
	               AgcThreshold, AGC_decay[2], LOfreq, mean, meanavg, Decay[4];
EXTERN uint16_t  Hangcount[2], AMindex, LSBindex, USBindex, CWindex, Hcount[4];    
EXTERN Mode      currentmode;
EXTERN float32_t SamplingRate, AudioRate;

EXTERN uint32_t  os_time, Fstep;
EXTERN __IO uint32_t uwTick;

//EXTERN WM_HWIN   hWin, hItem;
EXTERN u8        Muted;
EXTERN char      msg[32];

EXTERN NVIC_InitTypeDef      NVIC_InitStructure;
EXTERN EXTI_InitTypeDef      EXTI_InitStructure;

EXTERN float PeakAudioValue;

//EXTERN WM_MESSAGE *GlobalMsgPtr;

#endif /* __GLOBALS_H */
