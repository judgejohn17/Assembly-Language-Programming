/*********************************************************************/
/* Exercise 12: D/A Conversion, A/D Conversion, PWM, and Servos      */
/*              with Mixed Assembly Language and C                   */
/* Name:  <Your name here>                                           */
/* Date:  <Date completed>                                           */
/* Class:  CMPE 250                                                  */
/* Section:  <Your section here>                                     */
/*-------------------------------------------------------------------*/
/* Template:  R. W. Melton                                           */
/*            November 16, 2015                                      */
/*********************************************************************/
#include "MKL46Z4.h"
#include "Exercise12_C.h"

#define FALSE      (0)
#define TRUE       (1)

#define MAX_STRING (79)

/* ADC0 */
#define ADC_MAX (0x3FFu)
/* ------------------------------------------------------------------*/
/* ADC0_CFG1:  ADC0 configuration register 1                         */
/*  0-->31-8:(reserved):read-only:0                                  */
/*  1-->   7:ADLPC=ADC low-power configuration                       */
/* 10--> 6-5:ADIV=ADC clock divide select                            */
/*           Internal ADC clock = input clock / 2^ADIV               */
/*  1-->   4:ADLSMP=ADC long sample time configuration               */
/* 10--> 3-2:MODE=conversion mode selection                          */
/*           00=(DIFF'):single-ended 8-bit conversion                */
/*              (DIFF):differential 9-bit 2's complement conversion  */
/*           01=(DIFF'):single-ended 12-bit conversion               */
/*              (DIFF):differential 13-bit 2's complement conversion */
/*           10=(DIFF'):single-ended 10-bit conversion               */
/*              (DIFF):differential 11-bit 2's complement conversion */
/*           11=(DIFF'):single-ended 16-bit conversion               */
/*              (DIFF):differential 16-bit 2's complement conversion */
/* 01--> 1-0:ADICLK=ADC input clock select                           */
/*           00=bus clock                                            */
/*           01=bus clock / 2                                        */
/*           10=alternate clock (ALTCLK)                             */
/*           11=asynchronous clock (ADACK)                           */
/* BUSCLK = CORECLK / 2 = PLLCLK / 4                                 */
/* PLLCLK is 96 MHz                                                  */
/* BUSCLK is 24 MHz                                                  */
/* ADCinCLK is BUSCLK / 2 = 12 MHz                                   */
/* ADCCLK is ADCinCLK / 4 = 3 MHz                                    */
#define ADC0_CFG1_ADIV_BY2 (2u)
#define ADC0_CFG1_MODE_SGL10 (2u)
#define ADC0_CFG1_ADICLK_BUSCLK_DIV2 (1u)
#define ADC0_CFG1_LP_LONG_SGL10_3MHZ (ADC_CFG1_ADLPC_MASK | \
              (ADC0_CFG1_ADIV_BY2 << ADC_CFG1_ADIV_SHIFT) | \
                                     ADC_CFG1_ADLSMP_MASK | \
            (ADC0_CFG1_MODE_SGL10 << ADC_CFG1_MODE_SHIFT) | \
                               ADC0_CFG1_ADICLK_BUSCLK_DIV2)
/*-------------------------------------------------------------*/
/* ADC0_CFG2:  ADC0 configuration register 2                   */
/*  0-->31-8:(reserved):read-only:0                            */
/*  0--> 7-5:(reserved):read-only:0                            */
/*  0-->   4:MUXSEL=ADC mux select:  A channel                 */
/*  0-->   3:ADACKEN=ADC asynchronous clock output enable      */
/*  0-->   2:ADHSC=ADC high-speed configuration:  normal       */
/* 00--> 1-0:ADLSTS=ADC long sample time select (ADK cycles)   */
/*           default longest sample time:  24 total ADK cycles */
#define ADC0_CFG2_CHAN_A_NORMAL_LONG (0x00u)
/*---------------------------------------------------------   */
/* ADC0_SC1:  ADC0 channel status and control register 1      */
/*     0-->31-8:(reserved):read-only:0                        */
/*     0-->   7:COCO=conversion complete flag (read-only)     */
/*     0-->   6:AIEN=ADC interrupt enabled                    */
/*     0-->   5:DIFF=differential mode enable                 */
/* 10111--> 4-0:ADCH=ADC input channel select (23 = DAC0_OUT) */
#define ADC0_SC1_ADCH_AD23 (23u)
#define ADC0_SC1_SGL_DAC0 (ADC0_SC1_ADCH_AD23)
/*-----------------------------------------------------------*/
/* ADC0_SC2:  ADC0 status and control register 2             */
/*  0-->31-8:(reserved):read-only:0                          */
/*  0-->   7:ADACT=ADC conversion active                     */
/*  0-->   6:ADTRG=ADC conversion trigger select:  software  */
/*  0-->   5:ACFE=ADC compare function enable                */
/*  X-->   4:ACFGT=ADC compare function greater than enable  */
/*  0-->   3:ACREN=ADC compare function range enable         */
/*            0=disabled; only ADC0_CV1 compared             */
/*            1=enabled; both ADC0_CV1 and ADC0_CV2 compared */
/*  0-->   2:DMAEN=DMA enable                                */
/* 01--> 1-0:REFSEL=voltage reference selection:  VDDA       */
#define ADC0_SC2_REFSEL_VDDA (1u)
#define ADC0_SC2_SWTRIG_VDDA (ADC0_SC2_REFSEL_VDDA)
/*-------------------------------------------------------------*/
/* ADC0_SC3:  ADC0 status and control register 3               */
/* 31-8:(reserved):read-only:0                                 */
/*  0-->   7:CAL=calibration                                   */
/*          write:0=(no effect)                                */
/*                1=start calibration sequence                 */
/*          read:0=calibration sequence complete               */
/*               1=calibration sequence in progress            */
/*  0-->   6:CALF=calibration failed flag                      */
/* 00--> 5-4:(reserved):read-only:0                            */
/*  0-->   3:ADCO=ADC continuous conversion enable             */
/*           (if ADC0_SC3.AVGE = 1)                            */
/*  0-->   2:AVGE=hardware average enable                      */
/* XX--> 1-0:AVGS=hardware average select:  2^(2+AVGS) samples */
#define ADC0_SC3_SINGLE (0x00u)
#define ADC0_SC3_CAL (ADC_SC3_CAL_MASK | ADC_SC3_AVGE_MASK | \
                                           ADC_SC3_AVGS_MASK)
/*-------------------------------------------------------------*/
/* DAC */
#define DAC_DATH_0V   0x00u
#define DAC_DATL_0V   0x00u
/*---------------------------------------------------------------------*/
/* DAC_C0:  DAC control register 0                                   */
/* 1-->7:DACEN=DAC enabled                                             */
/* 1-->6:DACRFS=DAC reference select VDDA                              */
/* 0-->5:DACTRGSEL=DAC trigger select (X)                              */
/* 0-->4:DACSWTRG=DAC software trigger (X)                             */
/* 0-->3:LPEN=DAC low power control:high power                         */
/* 0-->2:(reserved):read-only:0                                        */
/* 0-->1:DACBTIEN=DAC buffer read pointer top flag interrupt enable    */
/* 0-->0:DACBBIEN=DAC buffer read pointer bottom flag interrupt enable */
#define DAC_C0_ENABLE  (DAC_C0_DACEN_MASK | DAC_C0_DACRFS_MASK)
/*----------------------------------------*/
/* DAC_C1:  DAC control register 1      */
/* 0-->  7:DMAEN=DMA disabled             */
/* 0-->6-3:(reserved)                     */
/* 0-->  2:DACBFMD=DAC buffer mode normal */
/* 0-->  1:(reserved)                     */
/* 0-->  0:DACBFEN=DAC buffer disabled    */
#define DAC_C1_BUFFER_DISABLED  (0x00u)
/*-------------------------------------------------------------*/
/* PORTx_PCRn                                                  */
/*     -->31-25:(reserved):read-only:0                         */
/*    1-->   24:ISF=interrupt status flag (write 1 to clear)   */
/*     -->23-20:(reserved):read-only:0                         */
/* 0000-->19-16:IRQC=interrupt configuration (IRQ/DMA diabled) */
/*     -->15-11:(reserved):read-only:0                         */
/*  ???-->10- 8:MUX=pin mux control                            */
/*     -->    7:(reserved):read-only:0                         */
/*    ?-->    6:DSE=drive strengh enable                       */
/*     -->    5:(reserved):read-only:0                         */
/*    0-->    4:PFE=passive filter enable                      */
/*     -->    3:(reserved):read-only:0                         */
/*    0-->    2:SRE=slew rate enable                           */
/*    0-->    1:PE=pull enable                                 */
/*    x-->    0:PS=pull select                                 */
/* Port E */
#define PTE30_MUX_DAC0_OUT (0u << PORT_PCR_MUX_SHIFT)
#define SET_PTE30_DAC0_OUT (PORT_PCR_ISF_MASK | PTE30_MUX_DAC0_OUT)
#define PTE31_MUX_TPM0_CH4_OUT (3u << PORT_PCR_MUX_SHIFT)
#define SET_PTE31_TPM0_CH4_OUT (PORT_PCR_ISF_MASK | \
                                PTE31_MUX_TPM0_CH4_OUT | \
                                PORT_PCR_DSE_MASK)
/*-------------------------------------------------------*/
/* SIM_SOPT2                                             */
/*   -->31-28:(reserved):read-only:0                     */
/*   -->27-26:UART0SRC=UART0 clock source select         */
/* 01-->25-24:TPMSRC=TPM clock source select (MCGFLLCLK) */
/*   -->23-19:(reserved):read-only:0                     */
/*   -->   18:USBSRC=USB clock source select             */
/*   -->   17:(reserved):read-only:0                     */
/*  1-->   16:PLLFLLSEL=PLL/FLL clock select (MCGFLLCLK) */
/*   -->15- 8:(reserved):read-only:0                     */
/*   --> 7- 5:CLKOUTSEL=CLKOUT select                    */
/*   -->    4:RTCCLKOUTSEL=RTC clock out select          */
/*   --> 3- 0:(reserved):read-only:0                     */
#define SIM_SOPT2_TPMSRC_MCGPLLCLK (1u << SIM_SOPT2_TPMSRC_SHIFT)
#define SIM_SOPT2_TPM_MCGPLLCLK_DIV2 (SIM_SOPT2_TPMSRC_MCGPLLCLK | \
                                            SIM_SOPT2_PLLFLLSEL_MASK)
/*------------------------------------------------------------------*/
/* TPMx_CONF:  Configuration (recommended to use default values     */
/*     -->31-28:(reserved):read-only:0                              */
/* 0000-->27-24:TRGSEL=trigger select (external pin EXTRG_IN)       */
/*     -->23-19:(reserved):read-only:0                              */
/*    0-->   18:CROT=counter reload on trigger                      */
/*    0-->   17:CSOO=counter stop on overflow                       */
/*    0-->   16:CSOT=counter stop on trigger                        */
/*     -->15-10:(reserved):read-only:0                              */
/*    0-->    9:GTBEEN=global time base enable                      */
/*     -->    8:(reserved):read-only:0                              */
/*   00-->  7-6:DBGMODE=debug mode (paused in debug)                */
/*    0-->    5:DOZEEN=doze enable                                  */
/*     -->  4-0:(reserved):read-only:0                              */
/* 15- 0:COUNT=counter value (writing any value will clear counter) */
#define TPM_CONF_DEFAULT (0u)
/*------------------------------------------------------------------*/
/* TPMx_CNT:  Counter                                               */
/* 31-16:(reserved):read-only:0                                     */
/* 15- 0:COUNT=counter value (writing any value will clear counter) */
#define TPM_CNT_INIT (0u)
/*------------------------------------------------------------------------*/
/* TPMx_MOD:  Modulo                                                      */
/* 31-16:(reserved):read-only:0                                           */
/* 15- 0:MOD=modulo value (recommended to clear TPMx_CNT before changing) */
/* Period = 3 MHz / 50 Hz */
#define TPM_MOD_PWM_PERIOD_20ms (60000u)
/*------------------------------------------------------------------*/
/* TPMx_CnSC:  Channel n Status and Control                         */
/* 0-->31-8:(reserved):read-only:0                                  */
/* 0-->   7:CHF=channel flag                                        */
/*              set on channel event                                */
/*              write 1 to clear                                    */
/* 0-->   6:CHIE=channel interrupt enable                           */
/* 1-->   5:MSB=channel mode select B (see selection table below)   */
/* 0-->   4:MSA=channel mode select A (see selection table below)   */
/* 1-->   3:ELSB=edge or level select B (see selection table below) */
/* 0-->   2:ELSA=edge or level select A (see selection table below) */
/* 0-->   1:(reserved):read-only:0                                  */
/* 0-->   0:DMA=DMA enable                                          */
#define TPM_CnSC_PWMH (TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK)
/*--------------------------------------------------------*/
/* TPMx_CnV:  Channel n Value                             */
/* 31-16:(reserved):read-only:0                           */
/* 15- 0:MOD (all bytes must be written at the same time) */
/*--------------------------------------------------------------*/
/* TPMx_SC:  Status and Control                                 */
/*    -->31-9:(reserved):read-only:0                            */
/*   0-->   8:DMA=DMA enable                                    */
/*    -->   7:TOF=timer overflow flag                           */
/*   0-->   6:TOIE=timer overflow interrupt enable              */
/*   0-->   5:CPWMS=center-aligned PWM select (edge align)      */
/*  01--> 4-3:CMOD=clock mode selection (count each TPMx clock) */
/* 100--> 2-0:PS=prescale factor selection                      */
/*    -->        can be written only when counter is disabled   */
#define TPM_SC_CMOD_CLK (1u)
#define TPM_SC_PS_DIV16 (0x4u)
#define TPM_SC_CLK_DIV16 ((TPM_SC_CMOD_CLK << TPM_SC_CMOD_SHIFT) | \
                          TPM_SC_PS_DIV16)
/*- -----*/
/* Servo */
#define SERVO_POSITIONS  (5)

UInt16 *DAC0_table = &DAC0_table_0;
UInt16 *PWM_duty_table = &PWM_duty_table_0;

//Initalize the DAC0 component with an analog output 
void Init_DAC0() {
	extern UInt16 DAC0_table_0;
	/*Enable TPM0 module clock */
	SIM->SCGC6 |= SIM_SCGC6_DAC0_MASK;
	/*Enable port E module clock */
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	/*Connect DAC0_OUT to Port E pin 30 (J4 Pin 11) */
	PORTE->PCR[30] = SET_PTE30_DAC0_OUT;
	/* Set DAC0 DMA disabled and buffer disabled */
	DAC0->C1 = DAC_C1_BUFFER_DISABLED;
	/* Set DAC0 enabled with VDDA as reference voltage. */
	/* and read pointer interrupts disabled */
	DAC0->C0 = DAC_C0_ENABLE;
	/* Set DAC0 output voltage at minimum value */
	//DAC0->DAT[0].DATL = DAC_DATH_MIN;
	//DAC0->DAT[0].DATH = DAC_DATL_MIN;
}


//Initalize and calibrate the KL46 ADC0 for polled conversion of single ended channed 23 (AD23). 
void Init_ADC0() {
	UInt16 pluCalibVal;
	UInt16 mnCalibVal;
	/* Enable ACD0 module clock */
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
	/* Set ADC0 power and timing */
	ADC0->CFG1 = ADC0_CFG1_LP_LONG_SGL10_3MHZ;
	/*Select Channel A and set timing */
	ADC0->CFG2 = ADC0_CFG2_CHAN_A_NORMAL_LONG;
	/* Select SW Trigger and VDDA reference */
	ADC0->SC2 = ADC0_SC2_SWTRIG_VDDA;
	/* Start calibration */
	ADC0->SC3 = ADC0_SC3_CAL;
	/*Wait for calibration to be complete */
	while (ADC0->SC3 & ADC_SC3_CAL_MASK) {
		/* Check for calibration failure */
		/* Occurs when CALF flag is set on ADCO->SC3 */
		if((ADC0->SC3 & 0x40) == 0x40) {
			/* Calibration failure, handle appropriately*/	
		}	
		/* Compute and store plus side calibration value */
		pluCalibVal = ADC0->CLP0 + ADC0->CLP1 + ADC0->CLP2 \
		+ ADC0->CLP3 + ADC0->CLP4 + ADC0->CLPS;
		/* Split up the operation to make it more readable */
		pluCalibVal = pluCalibVal >> 1;
		pluCalibVal = pluCalibVal | 0x8000;
		/* ADC0_PG <- calibVal */
		ADC0->PG = pluCalibVal;
		/* Compute and store minus side calibration value */
		mnCalibVal = ADC0->CLM0 + ADC0->CLM1 + ADC0->CLM2 \
		+ ADC0->CLM3 + ADC0->CLM4 + ADC0->CLMS;
		/* Split up the operation to make it more readable */
		mnCalibVal = mnCalibVal >> 1;
		mnCalibVal = mnCalibVal | 0x8000;
		/* ADC0_PG <- calibVal */
		ADC0->PG = mnCalibVal;
	}
	/* Post calibration Initalization. selecting single conversion */
	ADC0->SC3 = ADC0_SC3_SINGLE;
	
}

//Initalize the KL46 TPM0 to produce on channel 4 (TPM0_CH4) a waveform with a 20ms period and a 2ms duty period. 
void Init_TPM0() {
	/* Enable TPM0 module clock */
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	/* Enable port E module clock */
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	/* Connect TPM0 channel 4 to port E pin 31 */
	/* TODO: ENsure this is the correct #define to use */
	PORTE->PCR[31] = SET_PTE31_TPM0_CH4_OUT;	
	/* Set TPM clock source */
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPM_MCGPLLCLK_DIV2;	
	/* Set TPM0 configuration register to default values */
	TPM0->CONF = TPM_CONF_DEFAULT;
	/* Set TPM0 counter modulo value */
	TPM0->CNT = TPM_CNT_INIT;
	TPM0->MOD = TPM_MOD_PWM_PERIOD_20ms;	
	/* Set TPM0 channel 4 edge-aligned PWM */
	TPM0->CONTROLS[4].CnSC = TPM_CnSC_PWMH;	
	/* Set TPM0 channel 4 value */
	TPM0->CONTROLS[4].CnV = PWM_duty_table_0;	
	/* Set TPM0 counter clock configuration */
	TPM0->SC = TPM_SC_CLK_DIV16;
}


/* This function moves the servos to the point specified by the integer parameter 
*  in ranger [0,5]
*  Input Parameters:
*                   int position: Integer in range [0,5] corresponding to desired
*                                 servo position
*  Return:
*                   N/A
*/
void moveServo(int position) {
  extern UInt16 PWM_duty_table_0;   //Look up table from Exercise12_C.h
  int ServoPosition = position;    //Assign variable
	TPM0->CONTROLS[4].CnV = PWM_duty_table[ServoPosition - 1];  //Write CnV to the position
}

/* This function converts DAC to ADC
*  Input Parameters:
*                   analog: analog representation
*  Return:
*                   UInt16: Digitial representation of analog input
*/
UInt16 getADCConversion() {
	ADC0->SC1[0] = ADC0_SC1_SGL_DAC0;   //Convert 
	while(!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)) {} //Wait until  ADC0_SC1_SGL_DAC0 = 1
	return ADC0->R[0];  //Return digital conversion
}

/* This function gets the DAC value from the LUT. It converts the 
*   ASCI character to a digital signal value to be printed
*  Input Parameters:
*                   input: ASCII character from user representing desired servo position
*  Return:
*                   dacValue: original DAC digital signal value to be printed
*/
UInt16 writeToDAC0(int input) {
	
  int index;    //Where in LUT
  UInt16 dacValue;  //Return this
	
  //Convert from ACII to integer using define form above
  index = input-48-1; 
	dacValue = DAC0_table[index]; //dacValue = LUT[index]
    /*DAC0 needs to be set as the output voltage of the midpoint of the segment 
    two of voltage range*/
	DAC0->DAT[0].DATL = (UInt8) (dacValue & 0xFF);
	DAC0->DAT[0].DATH = (UInt8) (dacValue >> 8);
	return dacValue;    //Retun the value
}

/* This function converts the value from ADC0 in range 
*  [0,4096] to an integer in range [1,5]
*  Input Parameters:
*                   digitalVal: value from ADC0 in range [0,4096]
*  Return:
*                   calcPosition: the servo position [0,5]
*/
int calcServoPosition(UInt16 digitalVal) {
  
	int dividend = 1024;
	int index;
	index = (digitalVal * 5) / dividend; //calculate index of LUT
	return index + 1; //Have to add 1 because there is no position 0
}

int main (void) {
  char input;
  UInt16 original, adcVal, servoPos;
  __asm("CPSID   I");  /* mask interrupts */
  /* Initializations before masking interupts */
  Init_DAC0(); //Initalize the DACO module
  Init_ADC0(); //Initalize the ADC0 module
  Init_TPM0(); //Initialize TPM0 module
  Init_UART0_IRQ (); //Initialize the UART0_IRQ
  __asm("CPSIE   I");  /* unmask interrupts */

  for (;;) { /* do forever */
    PutStringSB("Type a number from 1 to 5: ", MAX_STRING);
	input = GetChar(); //get character
	while(input < '1' || input > '5') {input = GetChar();} //wait until input is valid, compare to literal
	PutChar(input); //echo char
	PutStringSB("\r\n", 3); //print new line, 3 is ASCII end of text 'ETX'	
	original = writeToDAC0(input); //Original digital value
	PutStringSB("Original digital value: 0x", MAX_STRING); //Original digital value is...
	PutNumHex(original); //Print value	
	adcVal = getADCConversion(); //get new value
	PutStringSB("\r\nNew digital value:      0x", MAX_STRING); //New digital value us...
	PutNumHex(adcVal); //Put value 
	servoPos = calcServoPosition(adcVal); //get that servo position, yo
	moveServo(servoPos); //move the servos to that position, like a boss
	PutStringSB("\r\nServo position: ", MAX_STRING); //Servo position is...
	PutChar(servoPos + 48); //print servo position, the 48 is ascii '0' which is the base to add ineger in [0,5] to get ASCII
	PutStringSB("\r\n", 3); //next line, 3 is ASCII end of text 'ETX'
    
  } 
} /* main */
