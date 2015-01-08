/*
 * File:        uart.c
 * Purpose:     Provide common UART routines for serial IO
 *
 * Notes:       
 *              
 */

#include "main.h"

/***************************************************************************
 * Begin UART0 functions
 **************************************************************************/
/********************************************************************/
/*
 * Initialize the uart for 8N1 operation, interrupts disabled, and
 * no hardware flow-control
 *
 * NOTE: Since the uarts are pinned out in multiple locations on most
 *       Kinetis devices, this driver does not enable uart pin functions.
 *       The desired pins should be enabled before calling this init function.
 *
 * Parameters:
 *  uartch      uart channel to initialize
 *  sysclk      uart module Clock in kHz(used to calculate baud)
 *  baud        uart baud rate
 */
void uart0_init (int sysclk, int baud)
{
    uint8_t i;
    uint32_t calculated_baud = 0;
    uint32_t baud_diff = 0;
    uint32_t osr_val = 0;
    uint32_t sbr_val, uart0clk;
    uint32_t baud_rate;
    uint32_t reg_temp = 0;
    uint32_t temp = 0;
    
    SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;
    
    // Disable UART0 before changing registers
    UART0->C2 &= ~(UART0_C2_TE_MASK | UART0_C2_RE_MASK);
  
    // Verify that a valid clock value has been passed to the function 
    if ((sysclk > 50000) || (sysclk < 32))
    {
        sysclk = 0;
        reg_temp = SIM->SOPT2;
        reg_temp &= ~SIM_SOPT2_UART0SRC_MASK;
        reg_temp |= SIM_SOPT2_UART0SRC(0);
        SIM->SOPT2 = reg_temp;
			
			  // Enter inifinite loop because the 
			  // the desired system clock value is 
			  // invalid!!
			  while(1)
				{}
    }
    // Verify that a valid value has been passed to TERM_PORT_NUM and update
    // uart0_clk_hz accordingly.  Write 0 to TERM_PORT_NUM if an invalid 
    // value has been passed.  
    if (TERM_PORT_NUM != 0)
    {
        reg_temp = SIM->SOPT2;
        reg_temp &= ~SIM_SOPT2_UART0SRC_MASK;
        reg_temp |= SIM_SOPT2_UART0SRC(0);
        SIM->SOPT2 = reg_temp;
			
			  // Enter inifinite loop because the 
			  // the desired terminal port number 
			  // invalid!!
			  while(1)
				{}
    }
    // Initialize baud rate
    baud_rate = baud;
    
    // Change units to Hz
    uart0clk = sysclk * 1000;
    // Calculate the first baud rate using the lowest OSR value possible.  
    i = 4;
    sbr_val = (uint32_t)(uart0clk/(baud_rate * i));
    calculated_baud = (uart0clk / (i * sbr_val));
        
    if (calculated_baud > baud_rate)
        baud_diff = calculated_baud - baud_rate;
    else
        baud_diff = baud_rate - calculated_baud;
    
    osr_val = i;
        
    // Select the best OSR value
    for (i = 5; i <= 32; i++)
    {
        sbr_val = (uint32_t)(uart0clk/(baud_rate * i));
        calculated_baud = (uart0clk / (i * sbr_val));
        
        if (calculated_baud > baud_rate)
            temp = calculated_baud - baud_rate;
        else
            temp = baud_rate - calculated_baud;
        
        if (temp <= baud_diff)
        {
            baud_diff = temp;
            osr_val = i; 
        }
    }
    
    if (baud_diff < ((baud_rate / 100) * 3))
    {
        // If the OSR is between 4x and 8x then both
        // edge sampling MUST be turned on.  
        if ((osr_val >3) && (osr_val < 9))
            UART0->C5|= UART0_C5_BOTHEDGE_MASK;
        
        // Setup OSR value 
        reg_temp = UART0->C4;
        reg_temp &= ~UART0_C4_OSR_MASK;
        reg_temp |= UART0_C4_OSR(osr_val-1);
    
        // Write reg_temp to C4 register
        UART0->C4 = reg_temp;
        
        reg_temp = (reg_temp & UART0_C4_OSR_MASK) + 1;
        sbr_val = (uint32_t)((uart0clk)/(baud_rate * (reg_temp)));
        
         /* Save off the current value of the uartx_BDH except for the SBR field */
        reg_temp = UART0->BDH & ~(UART0_BDH_SBR(0x1F));
   
        UART0->BDH = reg_temp |  UART0_BDH_SBR(((sbr_val & 0x1F00) >> 8));
        UART0->BDL = (uint8_t)(sbr_val & UART0_BDL_SBR_MASK);
        
#if UART_MODE == INTERRUPT
        UART0->C2 |= UART_C2_RIE_MASK;
#endif
        
        /* Enable receiver and transmitter */
        UART0->C2 |= (UART0_C2_TE_MASK | UART0_C2_RE_MASK );
    }
    else
		{
        // Unacceptable baud rate difference
        // More than 3% difference!!
        // Enter infinite loop!
        while(1)
				{}
		}					
    
}
/********************************************************************/
/*
 * Wait for a character to be received on the specified uart
 *
 * Parameters:
 *  channel      uart channel to read from
 *
 * Return Values:
 *  the received character
 */
char uart0_getchar()
{
      /* Wait until character has been received */
      while (!(UART0->S1 & UART0_S1_RDRF_MASK));
    
      /* Return the 8-bit data from the receiver */
      return UART0->D;
}
/********************************************************************/
/*
 * Wait for space in the uart Tx FIFO and then send a character
 *
 * Parameters:
 *  channel      uart channel to send to
 *  ch			 character to send
 */ 
void uart0_putchar (char ch)
{
      /* Wait until space is available in the FIFO */
      while(!(UART0->S1 & UART0_S1_TDRE_MASK));
    
      /* Send the character */
      UART0->D = (uint8_t)ch;
    
 }
/********************************************************************/
/*
 * Check to see if a character has been received
 *
 * Parameters:
 *  channel      uart channel to check for a character
 *
 * Return values:
 *  0       No character received
 *  1       Character has been received
 */
int uart0_getchar_present ()
{
    return (UART0->S1 & UART0_S1_RDRF_MASK);
}
/********************************************************************/
#if UART_MODE == INTERRUPT
void UART0_IRQHandler (void)
{
  char c = 0;
  if (UART0->S1&UART_S1_RDRF_MASK)
  {
    c = UART0->D;
    
    if ((UART0->S1&UART_S1_TDRE_MASK)||(UART0->S1&UART_S1_TC_MASK))
    {
    	UART0->D  = c;
    }
  }
}
#endif
