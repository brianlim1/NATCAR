/*
 * File:		a16dc.c
 * Purpose:		Simple Driver or API for AdC16/PGA
 * This file contains the following two functions:
 *  1. ADC_Cal: calibrates the ADC
 *  2. ADC_Config_Alt: Simply configures an ADC per a structure
 *
 * Typical usage: Fill the structure with the ADC and PGA register contents 
 * needed for the ADC usage.

 * a) Call the ADC_Config_Alt function to configure an ADC, (ADC0 or ADC1)
 * b) Call the ADC_Cal function to calibrate that ADC
 * c) Call the ADC_Config_Alt function again to restore desired configuation
 *    after a calibration
 *
 *
 */

#include <MKL25Z4.h>
#include "adc16.h"




/******************************************************************************
Function 1. Name	AUTO CAL ROUTINE   

Parameters		ADC module pointer points to adc0 or adc1 register map 
                         base address.
Returns			Zero indicates success.
Notes         		Calibrates the ADC16. Required to meet specifications 
                        after reset and before a conversion is initiated.
******************************************************************************/
unsigned char ADC_Cal()
{

  unsigned short cal_var;
  
  ADC0->SC2 &=  ~ADC_SC2_ADTRG_MASK ; // Enable Software Conversion Trigger for Calibration Process    - ADC0_SC2 = ADC0_SC2 | ADC_SC2_ADTRGW(0);   
  ADC0->SC3 &= ( ~ADC_SC3_ADCO_MASK & ~ADC_SC3_AVGS_MASK ); // set single conversion, clear avgs bitfield for next writing
  ADC0->SC3 |= ( ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(AVGS_32) );  // Turn averaging ON and set at max value ( 32 )
  
  
  ADC0->SC3 |= ADC_SC3_CAL_MASK ;      // Start CAL
  while ( (ADC0->SC1[0] & ADC_SC1_COCO_MASK ) == COCO_NOT ); // Wait calibration end
  	
  if ((ADC0->SC3& ADC_SC3_CALF_MASK) == CALF_FAIL )
  {  
   return(1);    // Check for Calibration fail error and return 
  }
  // Calculate plus-side calibration
  cal_var = 0x00;
  
  cal_var =  ADC0->CLP0; 
  cal_var += ADC0->CLP1;
  cal_var += ADC0->CLP2;
  cal_var += ADC0->CLP3;
  cal_var += ADC0->CLP4;
  cal_var += ADC0->CLPS;

  cal_var = cal_var/2;
  cal_var |= 0x8000; // Set MSB

  ADC0->PG = ADC_PG_PG(cal_var);
 

  // Calculate minus-side calibration
  cal_var = 0x00;

  cal_var =  ADC0->CLM0; 
  cal_var += ADC0->CLM1;
  cal_var += ADC0->CLM2;
  cal_var += ADC0->CLM3;
  cal_var += ADC0->CLM4;
  cal_var += ADC0->CLMS;

  cal_var = cal_var/2;

  cal_var |= 0x8000; // Set MSB

  ADC0->MG = ADC_MG_MG(cal_var); 
  
  ADC0->SC3 &= ~ADC_SC3_CAL_MASK ; /* Clear CAL bit */

  return(0);
}




/******************************************************************************
Function 2 Name 	ADC_Config_Alt 
Parameters		the register values to be set in the adc in a structure
Returns			NONE
Notes         		Configures ADC0 
                        Prior to calling this function populate the structure
                        elements with the desired ADC configuration.
******************************************************************************/


void ADC_Config_Alt(tADC_ConfigPtr ADC_CfgPtr)
{
 ADC0->CFG1 = ADC_CfgPtr->CONFIG1;
 ADC0->CFG2 = ADC_CfgPtr->CONFIG2;
 ADC0->CV1  = ADC_CfgPtr->COMPARE1; 
 ADC0->CV2  = ADC_CfgPtr->COMPARE2;
 ADC0->SC2  = ADC_CfgPtr->STATUS2;
 ADC0->SC3  = ADC_CfgPtr->STATUS3;
 //ADC0->PGA  = ADC_CfgPtr->PGA;  pbd
 ADC0->SC1[0]= ADC_CfgPtr->STATUS1A;       
 ADC0->SC1[1]= ADC_CfgPtr->STATUS1B;
}


void ADC_Read_Cal(tADC_Cal_Blk *blk)
{
  blk->OFS  = ADC0->OFS;
  blk->PG   = ADC0->PG; 
  blk->MG   = ADC0->MG; 
  blk->CLPD = ADC0->CLPD; 
  blk->CLPS = ADC0->CLPS; 
  blk->CLP4 = ADC0->CLP4;
  blk->CLP3 = ADC0->CLP3; 
  blk->CLP2 = ADC0->CLP2; 
  blk->CLP1 = ADC0->CLP1;
  blk->CLP0 = ADC0->CLP0;
  blk->CLMD = ADC0->CLMD; 
  blk->CLMS = ADC0->CLMS; 
  blk->CLM4 = ADC0->CLM4;
  blk->CLM3 = ADC0->CLM3; 
  blk->CLM2 = ADC0->CLM2; 
  blk->CLM1 = ADC0->CLM1;
  blk->CLM0 = ADC0->CLM0;
  
}


