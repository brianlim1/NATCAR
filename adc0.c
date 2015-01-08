 
void init_ADC0(void);

#include <MKL25Z4.h>
#include "adc16.h"

void init_ADC0(void){

	tADC_Config Master_Adc_Config;
  
	SIM->SCGC6 |= (SIM_SCGC6_ADC0_MASK );	// Enable ADC0 clock

               
// setup the initial ADC default configuration
              
	Master_Adc_Config.CONFIG1  = ADLPC_NORMAL
              | ADC_CFG1_ADIV(ADIV_8)
              | ADLSMP_LONG
              | ADC_CFG1_MODE(MODE_8)
              | ADC_CFG1_ADICLK(ADICLK_BUS);

	Master_Adc_Config.CONFIG2  = MUXSEL_ADCA
              | ADACKEN_DISABLED
              | ADHSC_HISPEED
              | ADC_CFG2_ADLSTS(ADLSTS_2) ;

	Master_Adc_Config.COMPARE1 = 0x1234u ;	// can be anything
	Master_Adc_Config.COMPARE2 = 0x5678u ;	// can be anything

	Master_Adc_Config.STATUS2  = ADTRG_SW
              | ACFE_DISABLED
              | ACFGT_GREATER
              | ACREN_DISABLED
              | DMAEN_DISABLED
              | ADC_SC2_REFSEL(REFSEL_EXT);
            
	Master_Adc_Config.STATUS3  = CAL_OFF
              | ADCO_SINGLE
              | AVGE_ENABLED
              | ADC_SC3_AVGS(AVGS_32);
            
	Master_Adc_Config.STATUS1A = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(31);
              
// Configure ADC as it will be used, but becuase ADC_SC1_ADCH is 31,
// the ADC will be inactive.  Channel 31 is just disable function.
// There really is no channel 31.
            
	ADC_Config_Alt(&Master_Adc_Config);  // config ADC
              
            
// Calibrate the ADC in the configuration in which it will be used:

	ADC_Cal();                    // do the calibration
            
// The structure still has the desired configuration.  So restore it.
// Why restore it?  The calibration makes some adjustments to the
// configuration of the ADC.  The are now undone:
            
// config the ADC again to desired conditions

	Master_Adc_Config.CONFIG1  = ADLPC_NORMAL
              | ADC_CFG1_ADIV(ADIV_2)
              | ADLSMP_LONG
              | ADC_CFG1_MODE(MODE_8)
              | ADC_CFG1_ADICLK(ADICLK_BUS);

	Master_Adc_Config.STATUS3  = CAL_OFF	// no hardware averaging
              | ADCO_SINGLE;

	ADC_Config_Alt(&Master_Adc_Config);
}
