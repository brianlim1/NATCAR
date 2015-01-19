// PWM Example program using Timer Overflow Interrupts.
// PWM Channel: TPM1_CH1 (PTB1) PWM frequency - 1 kHz; 
// Pulse width oscillates in continuously between 0% and 100% duty cycle 
// where the pulse width is updated in the TPM interrupt handler.

#include "MKL25Z4.h"
#include "uart.h"
#include "main.h"
#include "adc16.h"
#include "stdio.h"

void init_ADC0(void);

volatile unsigned short PW1 = 4500;	//initialize as 1.5ms
volatile unsigned short PWa = 300;	//initialize as 0.1ms
volatile unsigned short PWb = 300;	//initialize as 0.1ms
volatile char TPMflag = 0;
volatile unsigned short counter = 0;
volatile char LEDflag = 0;
const uint32_t led_mask[] = {1UL << 18, 1UL << 19, 1UL << 1};
int dutyA; int dutyB;
int dutyApercent; int dutyBpercent;
int CLKcount=0;
int buffSwitch = 2;
int camSwitch = 1;
int i=0;
int Done=0;
int count=0;
int sum1=0; int avg1=0; int max1=0; int min1=400; 
int sum2=0; int avg2=0; int max2=0; int min2=400;
int voltMid1=0; int voltMid2=0;
int voltCounter1=0; int voltCounter2=0;
int voltThreshold1; int voltThreshold2;
char ping1[130]; char pong1[130];
char ping2[130]; char pong2[130];
char zeroOne1[130]; char zeroOne2[130];
char ascii[2];
char key;
int A_IFB; int B_IFB;

#define LED_RED    0
#define LED_GREEN  1
#define LED_BLUE	 2
#define LED_A      0
#define LED_B      1
#define LED_C      2
#define LED_D      3
#define LED_CLK    4
#define CLOCK_SETUP 1

int charToInt(char c){
	int res;
	res = (int)c;
	return res;
  }

void intToHex(int c){
	int count1=0;
	while (c>=16){
		count1++;
	  c = c-16;}
	if (count1 ==0){
		ascii[0] = '0';}
	else if (count1 == 1) {
		ascii[0] = '1';}
	else if (count1 == 2) {
		ascii[0] = '2';}
	else if (count1 == 3) {
		ascii[0] = '3';}
	else if (count1 == 4) {
		ascii[0] = '4';}
	else if (count1 == 5) {
		ascii[0] = '5';}
	else if (count1 == 6) {
		ascii[0] = '6';}
	else if (count1 == 7) {
		ascii[0] = '7';}
	else if (count1 == 8) {
		ascii[0] = '8';}
	else if (count1 == 9) {
		ascii[0] = '9';}
	else if (count1 == 10) {
		ascii[0] = 'A';}
	else if (count1 == 11) {
		ascii[0] = 'B';}
	else if (count1 == 12) {
		ascii[0] = 'C';}
	else if (count1 == 13) {
		ascii[0] = 'D';}
	else if (count1 == 14) {
		ascii[0] = 'E';}
	else if (count1 == 15) {
		ascii[0] = 'F';}
	
	if (c==0){
		ascii[1] = '0';}
	else if (c==1){
		ascii[1] = '1';}
	else if (c==2){
		ascii[1] = '2';}
	else if (c==3){
		ascii[1] = '3';}
	else if (c==4){
		ascii[1] = '4';}
	else if (c==5){
		ascii[1] = '5';}
	else if (c==6){
		ascii[1] = '6';}
	else if (c==7){
		ascii[1] = '7';}
	else if (c==8){
		ascii[1] = '8';}
	else if (c==9){
		ascii[1] = '9';}
	else if (c==10){
		ascii[1] = 'A';}
	else if (c==11){
		ascii[1] = 'B';}
	else if (c==12){
		ascii[1] = 'C';}
	else if (c==13){
		ascii[1] = 'D';}
	else if (c==14){
		ascii[1] = 'E';}
	else if (c==15){
		ascii[1] = 'F';}
}

void put(char *ptr_str)
{
	while(*ptr_str)
		uart0_putchar(*ptr_str++);
}

void LED_Initialize(void) {

  SIM->SCGC5    |= (1UL <<  10) | (1UL <<  12); // Enable Clock to Port B & D  
	PORTB->PCR[0] = (1UL << 8);   								// Pin PTB0  is GPIO
	PORTB->PCR[1] = (1UL << 8); 									// Pin PTB1  is GPIO
  PORTB->PCR[18] = (1UL <<  8);                 // Pin PTB18 is GPIO 
  PORTB->PCR[19] = (1UL <<  8);                 // Pin PTB19 is GPIO
	//PORTB->PCR[2] = (1UL << 8);										//PTB2 is GPIO; (POT2 on TFC Shield, HBridge B)
	//PORTB->PCR[3] = (1UL << 8);										//PTB3 is GPIO; (POT1 on TFC Shield, HBridge A)
	PORTC->PCR[2] = (1UL << 8);										// Pin PTC2  is GPIO (HBridge_A In2)
	PORTC->PCR[4] = (1UL << 8);										// Pin PTC4  is GPIO (HBridge_B In2)
	PORTC->PCR[13] = (1UL << 8);									// Pin PTC13 is GPIO; (SW1 on TFC Shield)
	PORTC->PCR[17] = (1UL << 8);									// Pin PTC17 is GPIO; (SW2 on TFC Shield)
  PORTD->PCR[1]  = (1UL <<  8);                 // Pin PTD1  is GPIO
	PORTD->PCR[7]  = (1UL <<  8);									// Pin PTD7  is GPIO, SI
	PORTE->PCR[1]  = (1UL <<  8);									// Pin PTE1  is GPIO, CLK 
	PORTE->PCR[21] = (1UL << 8);									// Pin PTE21 is GPIO (HBridge Enable Pin)
	
  FPTB->PDOR = (led_mask[0] | led_mask[1] );          // switch Red/Green LED off  
  FPTB->PDDR = (led_mask[0] | led_mask[1] | 1UL << 0 | 1UL << 1 ); // enable PTB18/19 as Output 
  FPTD->PDOR = led_mask[2];           // switch Blue LED off  
  FPTD->PDDR =  (led_mask[2] | 1UL << 7);       // enable PTD1 and PTD7 as Output 
	FPTE->PDDR |= (1UL << 21 | 1UL << 1);					// enable PTE1 and PTE21 as output
	FPTC->PDDR |= (1UL << 2);											// enable PTC2 as output
	FPTC->PDDR |= (1UL << 4);											// enable PTC4 as output
	
}

/*----------------------------------------------------------------------------
  ADC functions
 *----------------------------------------------------------------------------*/
void Init_ADC(void) {
	
	init_ADC0();																															// initialize and calibrate ADC0 
	ADC0->CFG1 = (ADLPC_LOW | ADIV_1 | ADLSMP_LONG | MODE_8 | ADICLK_BUS_2);	// Normal Power configuration, Bus clock/2 = 12 MHz, long sample time, 8 bit, 
	ADC0->SC2 = 0;																														// ADTRG=0 (software trigger mode)
	NVIC_SetPriority(ADC0_IRQn, 128); // 0, 64, 128 or 192
	NVIC_ClearPendingIRQ(ADC0_IRQn); 
	NVIC_EnableIRQ(ADC0_IRQn);	
}


void ADC0_IRQHandler() {
	
	//120ns
	NVIC_ClearPendingIRQ(ADC0_IRQn);		//Clear Interrupt Request
	if(i<128){
		if (buffSwitch == 2){
			if(camSwitch == 1){
				ping1[i] = ADC0->R[0];}					//Read AO1 value to ping1 if pong1 is done
			else if(camSwitch == 2){
				ping2[i] = ADC0->R[0];}}				//Read AO2 value to ping2 if pong2 is done
		else if(buffSwitch == 1){
			if(camSwitch == 1){
				pong1[i] = ADC0->R[0];}					//Read AO1 value to pong1 if ping1 is done
			else if(camSwitch == 2){
				pong2[i] = ADC0->R[0];}}				//Read AO2 value to pong2 if ping2 is done
	}
	else if(i>=128){
		if(CLKcount == 129){
			A_IFB = ADC0->R[0];}
		else if(CLKcount == 130){
			B_IFB = ADC0->R[0];}
		}
	if(camSwitch == 1){									//If AO1 has just been converted, start conversion on AO2
		camSwitch = 2;
		ADC0->CFG2 |= ADC_CFG2_MUXSEL_MASK;
		ADC0->SC1[0] = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(7);}
	else if(camSwitch == 2){						//If AO2 has finished conversion, shift next pixel
		FPTE->PTOR = (1UL << 1);					//Toggle CLK high
		i++;															//shift buffer index
		CLKcount++;												//Increment CLK count
		if (CLKcount <129){
			camSwitch = 1;
			ADC0->CFG2 |= ADC_CFG2_MUXSEL_MASK;
			ADC0->SC1[0] = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(6);}
		else if(CLKcount ==129){
			ADC0->CFG2 &= ~(1UL << 4);	//Set channel to a
			ADC0->SC1[0] = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(7);}//Start ADC conversion on ADC0_SE7a (PTE23; H-Bridge_A_IFB)
		else if(CLKcount ==130){
			ADC0->CFG2 &= ~(1UL << 4);
			ADC0->SC1[0] = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(3);}	//Start ADC conversion on ADC0_SE3 (PTE22; H-Bridge_B_IFB)
		else{
			camSwitch=1;
			if(buffSwitch == 1){						//When buffer 2 is full
				buffSwitch = 2;}							//Switch to buffer 1
			else if(buffSwitch == 2){				//When buffer 1 is full
				buffSwitch = 1;}							//Switcch to buffer 2
			Done = 1;
			if(i>=128){i=0;}
			FPTB->PTOR = (1UL << 0);}
	FPTE->PTOR = (1UL << 1);}

}



/*----------------------------------------------------------------------------
  PWM functions
 *----------------------------------------------------------------------------*/
void Init_PWM(void) {
	// Set up the clock source for MCGPLLCLK/2. 
	// See p. 124 and 195-196 of the KL25 Sub-Family Reference Manual, Rev. 3, Sept 2012
	// TPM clock will be 48.0 MHz if CLOCK_SETUP is 1 in system_MKL25Z4.c.
	SIM-> SOPT2 |= (SIM_SOPT2_TPMSRC(1) | SIM_SOPT2_PLLFLLSEL_MASK);
	// See p. 207 of the KL25 Sub-Family Reference Manual, Rev. 3, Sept 2012
	SIM->SCGC6 |= (SIM_SCGC6_TPM0_MASK | SIM_SCGC6_TPM1_MASK);
	
	// See p. 163 and p. 183-184 of the KL25 Sub-Family Reference Manual, Rev. 3, Sept 2012
	PORTB->PCR[0] = PORT_PCR_MUX(3); // Configure PTB0 as TPM1_CH1
	PORTC->PCR[1] = PORT_PCR_MUX(4); // Configure PTC1 as TPM0_CH0
	PORTC->PCR[3] = PORT_PCR_MUX(4); // Configure PTC3 as TPM0_CH2

	// Set channel TPM1_CH0, TPM0_CH0/2 to edge-aligned, high-true PWM, enabled channel interrupt
	TPM1->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK | TPM_CnSC_CHIE_MASK;
	TPM0->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK /*| TPM_CnSC_CHIE_MASK*/;
	TPM0->CONTROLS[2].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK /*| TPM_CnSC_CHIE_MASK*/;
	
	// Set period and pulse widths
	TPM1->MOD = 60000-1;	// Freq. = (48 MHz / 16) / 60000 = 50 Hz
	TPM0->MOD = 600-1;	// Freq. = (48 MHz / 16) / 600 = 5 kHz
	TPM1->CONTROLS[0].CnV = PW1;
	TPM0->CONTROLS[0].CnV = PWb;
	TPM0->CONTROLS[2].CnV = PWa;
	
	// set TPM0/1 to up-counter, divide by 16 prescaler and clock mode
	TPM1->SC = (/*TPM_SC_TOIE_MASK | */TPM_SC_CMOD(1) | TPM_SC_PS(4));
	TPM0->SC = (/*TPM_SC_TOIE_MASK | */TPM_SC_CMOD(1) | TPM_SC_PS(4));
	
	// clear the overflow mask by writing 1 to CHF
	if(TPM1->CONTROLS[0].CnSC & TPM_CnSC_CHF_MASK){
		TPM1->CONTROLS[0].CnSC |= TPM_CnSC_CHF_MASK;}
	if(TPM0->CONTROLS[0].CnSC & TPM_CnSC_CHF_MASK){
		TPM0->CONTROLS[0].CnSC |= TPM_CnSC_CHF_MASK;}
	if(TPM0->CONTROLS[2].CnSC & TPM_CnSC_CHF_MASK){
		TPM0->CONTROLS[2].CnSC |= TPM_CnSC_CHF_MASK;}

	// Enable Interrupts
	NVIC_SetPriority(TPM1_IRQn, 192); // 0, 64, 128 or 192
	NVIC_ClearPendingIRQ(TPM1_IRQn); 
	NVIC_EnableIRQ(TPM1_IRQn);
	
	NVIC_SetPriority(TPM0_IRQn, 192); // 0, 64, 128 or 192
	NVIC_ClearPendingIRQ(TPM0_IRQn); 
	NVIC_EnableIRQ(TPM0_IRQn);	
}

void TPM1_IRQHandler(void) {
	//clear pending IRQ
	NVIC_ClearPendingIRQ(TPM1_IRQn);

	// clear the overflow mask by writing 1 to CHF
	if(TPM1->CONTROLS[0].CnSC & TPM_CnSC_CHF_MASK){
		TPM1->CONTROLS[0].CnSC |= TPM_CnSC_CHF_MASK;}
	TPM1->CONTROLS[0].CnV = PW1;

	/*
	// modify pulse width for TPM1_CH0
	if (!TPMflag) {
		TPM1->CONTROLS[0].CnV = ++PW1;
		if (PW1 >= 6000) //2.0ms
			TPMflag = 1;
	} 
	else {
		TPM1->CONTROLS[0].CnV = --PW1;
		if (PW1 <= 3000) //1.0ms
			TPMflag = 0;
	}
	*/
	counter++;
	if (counter >= 50) {
		counter = 0;
		if (PW1 >= 6000 || PW1 <= 3000){
			FPTB->PSOR = led_mask[LED_GREEN];
			FPTB->PTOR = led_mask[LED_RED];
		}
		else{
			FPTB->PSOR = led_mask[LED_RED];
			FPTB->PTOR = led_mask[LED_GREEN];}
	}
}

void TPM0_IRQHandler(void){
}

/*----------------------------------------------------------------------------
  PIT functions
 *----------------------------------------------------------------------------*/
void Init_PIT(unsigned period_us) {
	
	SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;											//Enable clock gate to PIT module
	PIT->MCR &= ~PIT_MCR_MDIS_MASK;												//Enable PIT clock module
	PIT->MCR |= PIT_MCR_FRZ_MASK;													//Freeze clocks when debugging
	//PIT->CHANNEL[0].LDVAL = PIT_LDVAL_TSV(2400); 				//Load countdown value to Channel 0 of PIT; Gives interrupt frequency of 10kHz
	PIT->CHANNEL[0].LDVAL = PIT_LDVAL_TSV(24*period_us);	//Load countdown value to Channel 0 of PIT; Gives interrupt frequency of 100Hz
	PIT->CHANNEL[0].TCTRL &= PIT_TCTRL_CHN_MASK;					//Disable chaining
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK;					//Enables timer interrupts when timer reaches 0
	// Enable Interrupts
	NVIC_SetPriority(PIT_IRQn, 128); // 0, 64, 128 or 192
	NVIC_ClearPendingIRQ(PIT_IRQn); 
	NVIC_EnableIRQ(PIT_IRQn);	
}

void Start_PIT(void) {
	
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;	//Enables timer
}

void Stop_PIT(void) {
	
	PIT->CHANNEL[0].TCTRL &= ~PIT_TCTRL_TEN_MASK;	//Disables timer
}

void PIT_IRQHandler(void) {
	//clear pending IRQ
	NVIC_ClearPendingIRQ(PIT_IRQn);
	
	// check to see which channel triggered interrupt 
	if (PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK) {	
		FPTD->PTOR = (1UL << 7);										//Toggle PTD7 high (SI)
		FPTB->PTOR = (1UL << 0); 										//Toggle PTB0 (conversion time measure)
		PIT->CHANNEL[0].TFLG &= PIT_TFLG_TIF_MASK;	//Clear interrupt flag for Channel 0
		FPTE->PTOR = (1UL << 1);										//Toggle PTE1 high (CLK)
		CLKcount=1;																	//Increment clock counter, check done flag, update buffer index
		FPTD->PTOR = (1UL << 7);										//Toggle PTD7 low (SI)
		FPTE->PTOR = (1UL << 1);										//Toggle PTE1 low (CLK)
		
		ADC0->CFG2 |= ADC_CFG2_MUXSEL_MASK;
		ADC0->SC1[0] = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(6);		//Start ADC conversion on ADC0_SE6b
	}
	else if (PIT->CHANNEL[1].TFLG & PIT_TFLG_TIF_MASK) {
		PIT->CHANNEL[1].TFLG &= PIT_TFLG_TIF_MASK;	//Clear interrupt flag for Channel 1
	} 
}


/*----------------------------------------------------------------------------
	H-Bridge functions
 *----------------------------------------------------------------------------*/
void disable_HBridge(void){
	FPTE->PCOR = (1UL << 21);}

void enable_HBridge(void){
	FPTE->PSOR = (1UL << 21);}

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
	char str[] = "\r\nTurn on power supply, then press SW2\r\n";
	char str2[] = "\r\nSet duty cycles using POT1 and POT2\r\n";
	char str3[] = "\r\nPress 'c' to continue scanning cameras or 'q' to quit\r\n";
	char str4[80];
	int uart0_clk_khz;
	int SW1_Not_Pressed = 1;

	SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK
	| SIM_SCGC5_PORTB_MASK
	| SIM_SCGC5_PORTC_MASK
	| SIM_SCGC5_PORTD_MASK
	| SIM_SCGC5_PORTE_MASK );
	SIM->SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK; // set PLLFLLSEL to select the PLL for this clock source
	SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1); // select the PLLFLLCLK as UART0 clock source
	
	PORTA->PCR[1] = PORT_PCR_MUX(0x2);		// Enable the UART0_RX function on PTA1
	PORTA->PCR[2] = PORT_PCR_MUX(0x2);		// Enable the UART0_TX function on PTA2
	
	uart0_clk_khz = (48000000 / 1000); // UART0 clock frequency will equal half the PLL frequency	
	uart0_init (uart0_clk_khz, TERMINAL_BAUD);
	
	SystemCoreClockUpdate();
	LED_Initialize();
	Init_PWM();
	Init_PIT(40000);													//Load countdown value to Channel 0 of PIT; interrupt frequency = 50Hz
	Init_ADC();
	disable_HBridge();
	put(str);		//Turn on power supply, then press SW2

	while (!(FPTC->PDIR & (1UL << 17))){;}	//Poll until SW2 has been pressed
	
	enable_HBridge();
	put(str2);	//Set duty cycles using POT1 and POT2
	//FPTC->PSOR = (1UL << 2);
	//FPTC->PSOR = (1UL << 4);
	
	
	while (1) {
	/*
//__wfi();
	while(!uart0_getchar_present()){;}
	if(uart0_getchar() == 'R')
		{PW1 = 3000;}
	else if(uart0_getchar() == 'r')
		{PW1 = 3750;}
	else if(uart0_getchar() == 'C' || uart0_getchar() == 'c')
		{PW1 = 4500;}
	else if(uart0_getchar() == 'l')
		{PW1 = 5250;}
	else if(uart0_getchar() == 'L')
		{PW1 = 6000;}
	else if(uart0_getchar() == 'q' || uart0_getchar() == 'Q')
		{return 0;}
	else if(uart0_getchar() == '+')
		{if (PW1 < 6000){
			PW1+=50;}}
	else if(uart0_getchar() == '-')
		{if (PW1 > 3000){
			PW1-=50;}}
			*/ //block comment: Servo Program controls from Lab5
		while(SW1_Not_Pressed){
			ADC0->SC1[0] = DIFF_SINGLE | ADC_SC1_ADCH(13);		//Start ADC conversion on ADC0_SE13 (PTB3; POT1)
			while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)) {	; }		// wait for conversion to complete (polling)
			dutyA = ADC0->R[0];		//Read 8-bit digital value of POT1
			sprintf(str4, "%d", dutyA); put(str4); put(" ");
			
			ADC0->SC1[0] = DIFF_SINGLE | ADC_SC1_ADCH(12);		//Start ADC conversion on ADC0_SE12 (PTB2; POT2)
			while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)) {	; }		// wait for conversion to complete (polling)
			dutyB = ADC0->R[0];		//Read 8-bit digital value of POT2
			sprintf(str4, "%d", dutyB); put(str4); put("\r\n");
			
			PWa = (600*dutyA)/255;				//Multiply max pulse width by percentage according to POT1
			PWb = (600*dutyB)/255;				//Multiply max pulse width by percentage according to POT2
			TPM0->CONTROLS[0].CnV = PWb;	//Set pulse width of H_Bridge A according to POT1
			TPM0->CONTROLS[2].CnV = PWa;	//Set pulse width of H_Bridge B according to POT2
	
			if ((FPTC->PDIR & (1UL << 13))){SW1_Not_Pressed = 0;}	//Check if SW1 has been pressed
		}
		Start_PIT();
		while(!(SW1_Not_Pressed)){
			if (Done){
				if(!uart0_getchar_present()){
					count=0;
					if(buffSwitch == 1){													//Check if buffer 1 is done or buffer 2
						while (count<128){
							if(ping1[count]>max1){max1=ping1[count];}
							if(ping1[count]<min1){min1=ping1[count];}
							if(ping2[count]>max2){max2=ping2[count];}
							if(ping2[count]<min2){min2=ping2[count];}
							count++;}
						count=0;
						voltThreshold1 = (max1+min1)/2;								//Calculate voltage threshold using max/min
						voltThreshold2 = (max2+min2)/2;
						while(count<128){														//Compare entire buffer with threshold
							if(count>14 & count<113){								//Ignores buffer values <14 and >113 because they are inaccurate
								//Begin voltage scheme
								if(ping1[count] >= voltThreshold1)
									{zeroOne1[count] = '1';}								//If greater than threshold, black/white array gets 1
								else if(ping1[count] < voltThreshold1){	//If less than threshold, black/white array gets 0
									zeroOne1[count] = '0';																		
									voltMid1 = voltMid1 + count;						//Add middle-of-black-line index
									voltCounter1++;}				//Increment middle-of-black-line counter
								if(ping2[count] >= voltThreshold2)
									{zeroOne2[count] = '1';}
								else if(ping2[count] < voltThreshold2){
									zeroOne2[count] = '0';
									voltMid2 = voltMid2 + count;
									voltCounter2++;}
								//End voltage scheme
							}
							else
								{zeroOne1[count]='1';
								zeroOne2[count]='1';}									//Buffer values <14 and >113 automatically get 1
							count++;}
						__disable_irq();
						put("\r\nPing: \r\n");}
					else if (buffSwitch == 2){
						while (count<128){
							if(pong1[count]>max1){max1=pong1[count];}
							if(pong1[count]<min1){min1=pong1[count];}
							if(pong2[count]>max2){max2=pong2[count];}
							if(pong2[count]<min2){min2=pong2[count];}
							count++;}
						count=0;
						voltThreshold1 = (max1+min1)/2;
						voltThreshold2 = (max2+min2)/2;
						while(count<128){
							if(count>14 & count< 113){
								//Begin voltage scheme
								if(pong1[count] >= voltThreshold1)
									{zeroOne1[count] = '1';}
								else if(pong1[count] < voltThreshold1){
									zeroOne1[count] = '0';
									voltMid1 = voltMid1 + count;
									voltCounter1++;}
								if(pong2[count] >= voltThreshold2)
									{zeroOne2[count] = '1';}
								else if(pong2[count] < voltThreshold2){
									zeroOne2[count] = '0';
									voltMid2 = voltMid2 + count;
									voltCounter2++;}
								//End voltage scheme
							}
							else
								{zeroOne1[count]='1';
								zeroOne2[count]='1';}
							count++;}
						__disable_irq();
						put("\r\nPong: \r\n");}
				
					voltMid1 = voltMid1/voltCounter1;	//Calculate voltage midpoint by dividing all black indices with counter
					voltMid2 = voltMid2/voltCounter2;
					//Adjust servo here
					//Right now, the program will compare the calculated midpoint of the black line to a preset value, then adjust the servo PWM.
					//The range of values of the midpoint is from 14-113, except when the black line is out of range, then the midpoint defaults to -1.
					//Ideally, we want the servo to adjust more or less depending on how big the error is. I haven't done that yet.
					//The code here doesn't work correctly, it was just rough code used to finish lab 7. It is programmed to stay within a straight line, but not to do turns
					/*
					//looks at camera 1, (should be mounted on the left side of the car)
					if(voltMid1 > 39){
						PW1-=300;}	//Slight right turn
					//looks at camera 2, (should be mounted on the right side of the car)
					if(voltMid2 < 88){
						PW1+=300;}	//Slight left turn
					else{PW1=4500;}	//Centers the servo
					*/
					
					TPM1->CONTROLS[0].CnV = PW1;
					put("Cam1: ");put(zeroOne1); //put("\r\n");
					sprintf(str4, "%d", voltMid1); put(" "); put(str4); put("\r\n");
					put("Cam2: ");put(zeroOne2); //put("\r\n");
					sprintf(str4, "%d", voltMid2); put(" "); put(str4); put("\r\n");
					__enable_irq();
					Done=0;count=0;
					sum1=0;avg1=0;max1=0;min1=400;voltMid1=0;voltCounter1=0;
					sum2=0;avg2=0;max2=0;min2=400;voltMid2=0;voltCounter2=0;
				}
				else if(uart0_getchar() == 'p'){
					while(1){;}
					/*
					Stop_PIT();
					__disable_irq();
					
					//if(camSwitch == 1){								//Print Camera 1
						put("\r\nCamera 1:");
						put("\r\nPing1: \r\n");
						while(count<128){
							intToHex(ping1[count]);
							put(ascii); put(" ");
							count++;}
						count=0;
						put("\r\nPong1: \r\n");
						while(count<128){
							intToHex(pong1[count]);
							put(ascii); put(" ");
							count++;}
						put("\r\n");
					//}
					//else if (camSwitch == 2){					//Print Camera 2
						count=0;
						put("\r\nCamera 2:");
						put("\r\nPing2:\r\n");
						while(count<128){
							intToHex(ping2[count]);
							put(ascii); put(" ");
							count++;}
						count=0;
						put("\r\nPong2: \r\n");
						while(count<128){
							intToHex(pong2[count]);
							put(ascii); put(" ");
							count++;}
						put("\r\n");
						put("\r\nA_IFB: ");
						intToHex(A_IFB); put(ascii);
						put("\r\nB_IFB: ");
						intToHex(B_IFB); put(ascii); put("\r\n");
					//}
					*/
					put(str3);
					while(1){
						key = uart0_getchar();
						if(key == 'q'){
							put("\r\nQuitting Program\r\n");
							disable_HBridge();
							return 0;}
						else if(key == 'c'){
							__enable_irq();
							Start_PIT();
							count=0; Done=0;
							break;}
					}
				}
				intToHex(A_IFB); put(ascii); put(" ");
				intToHex(B_IFB); put(ascii); put("\r\n");
			}
			else if(!Done){continue;}
		}
	}
}
