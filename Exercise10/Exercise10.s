
	;****************************************************************
;Descriptive comment header goes here.
;This program develops a time for the KL46 periodic interrupt
;timer 
;Name:  John Judge
;Date: 4/21/16
;Class:  CMPE-250
;Section: Thursdays 2:00pm-3:50pm
;---------------------------------------------------------------
;Keil Template for KL46
;R. W. Melton
;April 3, 2015
;****************************************************************
;Assembler directives
            THUMB
            OPT    64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL46Z4.s     ;Included by start.s
            OPT  1   ;Turn on listing
;****************************************************************
;EQUates
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port A
PORT_PCR_SET_PTA1_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTA2_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
;---------------------------------------------------------------
;SIM_SCGC4
;1->10:UART0 clock gate control (enabled)
;Use provided SIM_SCGC4_UART0_MASK
;---------------------------------------------------------------
;SIM_SCGC5
;1->09:Port A clock gate control (enabled)
;Use provided SIM_SCGC5_PORTA_MASK
;---------------------------------------------------------------
;SIM_SOPT2
;01=27-26:UART0SRC=UART0 clock source select
;         (PLLFLLSEL determines MCGFLLCLK' or MCGPLLCLK/2)
; 1=   16:PLLFLLSEL=PLL/FLL clock select (MCGPLLCLK/2)
SIM_SOPT2_UART0SRC_MCGPLLCLK  EQU  \
                                 (1 << SIM_SOPT2_UART0SRC_SHIFT)
SIM_SOPT2_UART0_MCGPLLCLK_DIV2 EQU \
    (SIM_SOPT2_UART0SRC_MCGPLLCLK :OR: SIM_SOPT2_PLLFLLSEL_MASK)
;---------------------------------------------------------------
;SIM_SOPT5
; 0->   16:UART0 open drain enable (disabled)
; 0->   02:UART0 receive data select (UART0_RX)
;00->01-00:UART0 transmit data select source (UART0_TX)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR  EQU  \
                               (SIM_SOPT5_UART0ODE_MASK :OR: \
                                SIM_SOPT5_UART0RXSRC_MASK :OR: \
                                SIM_SOPT5_UART0TXSRC_MASK)
;---------------------------------------------------------------
;UART0_BDH
;    0->  7:LIN break detect IE (disabled)
;    0->  6:RxD input active edge IE (disabled)
;    0->  5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (UART0CLK / [9600 * (OSR + 1)]) 
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDH_9600  EQU  0x01
;---------------------------------------------------------------
;UART0_BDL
;26->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDL_9600  EQU  0x38
;---------------------------------------------------------------
;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select 
;        (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=parity enable (disabled)
;0-->0:PT=parity type (even parity--no effect PE=0)
UART0_C1_8N1  EQU  0x00
;---------------------------------------------------------------
;UART0_C2
;0-->7:TIE=transmit IE for TDRE (disabled)
;0-->6:TCIE=transmission complete IE for TC (disabled)
;0-->5:RIE=receiver IE for RDRF (disabled)
;0-->4:ILIE=idle line IE for IDLE (disabled)
;1-->3:TE=transmitter enable (enabled)
;1-->2:RE=receiver enable (enabled)
;0-->1:RWU=receiver wakeup control (normal)
;0-->0:SBK=send break (disabled, normal)
UART0_C2_T_R    EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
UART0_C2_T_RI   EQU  (UART0_C2_RIE_MASK :OR: UART0_C2_T_R)
UART0_C2_TI_RI  EQU  (UART0_C2_TIE_MASK :OR: UART0_C2_T_RI)
;---------------------------------------------------------------
;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
;           10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
;           10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
;            (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:ORIE=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=parity error IE for PF (disabled)
UART0_C3_NO_TXINV  EQU  0x00
;---------------------------------------------------------------
;UART0_C4
;    0-->  7:MAEN1=match address mode enable 1 (disabled)
;    0-->  6:MAEN2=match address mode enable 2 (disabled)
;    0-->  5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
;               = 1 + OSR for 3 <= OSR <= 31
;               = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16           EQU  0x0F
UART0_C4_NO_MATCH_OSR_16  EQU  UART0_C4_OSR_16
;---------------------------------------------------------------
;UART0_C5
;  0-->  7:TDMAE=transmitter DMA enable (disabled)
;  0-->  6:Reserved; read-only; always 0
;  0-->  5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
;  0-->  1:BOTHEDGE=both edge sampling (rising edge only)
;  0-->  0:RESYNCDIS=resynchronization disable (enabled)
UART0_C5_NO_DMA_SSR_SYNC  EQU  0x00
;---------------------------------------------------------------
;UART0_S1
;0-->7:TDRE=transmit data register empty flag; read-only
;0-->6:TC=transmission complete flag; read-only
;0-->5:RDRF=receive data register full flag; read-only
;1-->4:IDLE=idle line flag; write 1 to clear (clear)
;1-->3:OR=receiver overrun flag; write 1 to clear (clear)
;1-->2:NF=noise flag; write 1 to clear (clear)
;1-->1:FE=framing error flag; write 1 to clear (clear)
;1-->0:PF=parity error flag; write 1 to clear (clear)
UART0_S1_CLEAR_FLAGS  EQU  0x1F
;---------------------------------------------------------------
;UART0_S2
;1-->7:LBKDIF=LIN break detect interrupt flag (clear)
;             write 1 to clear
;1-->6:RXEDGIF=RxD pin active edge interrupt flag (clear)
;              write 1 to clear
;0-->5:(reserved); read-only; always 0
;0-->4:RXINV=receive data inversion (disabled)
;0-->3:RWUID=receive wake-up idle detect
;0-->2:BRK13=break character generation length (10)
;0-->1:LBKDE=LIN break detect enable (disabled)
;0-->0:RAF=receiver active flag; read-only
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS  EQU  0xC0
;---------------------------------------------------------------
;---------------------------------------------------------------
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ICER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;22:PIT IRQ pending status
;12:UART0 IRQ pending status
NVIC_ICPR_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
;--PIT
PIT_IRQ_PRIORITY    EQU  0
NVIC_IPR_PIT_MASK   EQU  (3 << PIT_PRI_POS)
NVIC_IPR_PIT_PRI_0  EQU  (PIT_IRQ_PRIORITY << UART0_PRI_POS)
;--UART0
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU  (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU  (UART0_IRQ_PRIORITY << UART0_PRI_POS)
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ISER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;PIT_LDVALn:  PIT load value register n
;31-00:TSV=timer start value (period in clock cycles - 1)
;Clock ticks for 0.01 s at 24 MHz count rate
;0.01 s * 24,000,000 Hz = 240,000
;TSV = 240,000 - 1
PIT_LDVAL_10ms  EQU  239999
;---------------------------------------------------------------
;PIT_MCR:  PIT module control register
;1-->    0:FRZ=freeze (continue'/stop in debug mode)
;0-->    1:MDIS=module disable (PIT section)
;               RTI timer not affected
;               must be enabled before any other PIT setup
PIT_MCR_EN_FRZ  EQU  PIT_MCR_FRZ_MASK
;---------------------------------------------------------------
;PIT_TCTRLn:  PIT timer control register n
;0-->   2:CHN=chain mode (enable)
;1-->   1:TIE=timer interrupt enable
;1-->   0:TEN=timer enable
PIT_TCTRL_CH_IE  EQU  (PIT_TCTRL_TEN_MASK :OR: PIT_TCTRL_TIE_MASK)
;---------------------------------------------------------------
;Interrupt should be set to the highest priority
PIT_IRQ_PRI 	 EQU 0
;---------------------------------------------------

;Max length of queue
Q_BUF_SZ				EQU 4
Q_REC_SZ                EQU 18

;Max length of prompt string
MAX_STRING 				EQU 79
MAX_QUEUE				EQU	80	
IN_PTR					EQU 0
OUT_PTR					EQU 4
BUF_START				EQU 8
BUF_PAST				EQU 12
BUF_SIZE				EQU 16
NUM_ENQD				EQU 17
	
FiveSec					EQU	 500
;****************************************************************
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
            EXPORT  Reset_Handler
            IMPORT  Startup
Reset_Handler
main
;---------------------------------------------------------------
;Mask interrupts
            CPSID   I
;KL46 system startup with 48-MHz system clock
            BL      Startup
;---------------------------------------------------------------
;>>>>> begin main program code <<<<<

			;Enable clock interrupts
			BL      Init_UART0_IRQ
			BL 		Init_PIT_IRQ
Main
			LDR     R0,=RunStopWatch		;Initialize The Stop Watch 
			MOVS    R1,#0					;
			STRB    R1,[R0,#0]			    ;Initialize StopWatch to zero
			LDR     R0,=Count 				;Initialize count 
			MOVS    R1,#0					;
			STRB    R1,[R0,#0]			    ;Initialize count as zero 
			LDR     R0,=AccessCodePrompt	;prompt to print
			BL		LengthStringSB			;Find length for PutStringSB
			BL      PutStringSB				;Print prompt
			MOVS    R0,#0x0D				;Line feed
			BL      PutChar
			MOVS    R0,#0x0A				;Carriage return 
			BL      PutChar
			MOVS    R0,#">"				    ;Input carrot
			BL      PutChar
			LDR     R0,=RunStopWatch		;Change stop watch value
			MOVS    R1,#1
			STRB    R1,[R0, #0]			    ;Store RunStopWatch=1
			MOVS    R1,#MAX_STRING 		    ;Initialize Max_string for GetStringSb
			LDR     R0,=InputString		    ;Initialize input string
			BL      GetStringSB				;Poll for new String
			LDR     R0,=RunStopWatch		;Initialize stopwatch to set to 0
			MOVS    R1,#0					
			STRB    R1,[R0, #0]			    ;Set RunStopWatch = 0
			MOVS    R0,#0x0D				;Carriage Return
			BL      PutChar
			MOVS    R0,#0x0A				;Line Feed
			BL      PutChar
			MOVS    R0,#'<'				    ;Carrot
			BL      PutChar
			LDR     R0,=Count				;Load count
			LDR     R0,[R0,#0]			
			MOVS    R1,R0					;Move to R1 for PutNumU
			BL      PutNumU					;Print Count number
			LDR     R0,=TimeString			;String to add to end of time
			BL		LengthStringSB			;Find length for PutStringSB
			BL      PutStringSB					
			MOVS    R0,#0x0D				;Carriage Return
			BL      PutChar					;
			MOVS    R0,#0x0A				;Line Feed
			BL      PutChar
			MOVS    R0,#10				    ;Divide the count by 10
			BL      DIVU
			LDR		R7,=FiveSec
			CMP     R0,R7				    ;Compare to 5 seconds
			BGE     AccessDenied			;If it took longer than 5 seconds
			LDR     R0,=InputString		    ;Load input string
			LDR     R1,=AccessCode			;Load code 
			MOVS    R2,#7					;Length of code
CheckChar
			LDRB    R3,[R0,R2]			    ;Load input string
			LDRB    R4,[R1,R2]			    ;Load code string
			CMP     R3,R4					;Compare strings
			BNE     AccessDenied			;If strings are not equal
			CMP     R2,#0					;Check to make sur all characters have been checked 
			BEQ     AccessGranted			;If equal then allow access
			SUBS    R2,R2,#1				;Decrement amount and keep looping
			B       CheckChar
AccessGranted
			LDR     R0,=Granted			    ;Access granted string
			BL		LengthStringSB			;Find length for PutStringSB
			BL      PutStringSB				;
			MOVS    R0,#0x0D				;Carriage Return 
			BL      PutChar					;
			MOVS    R0,#0x0A				;Line Feed
			BL      PutChar					;
			LDR     R0,=Mission_Completed	;Mission completed!
			BL		LengthStringSB			;Find length for PutStringSB
			BL      PutStringSB				;
			B 		.					    ;End
			
AccessDenied
			LDR     R0,=Denied				;Dee-nied!
			BL		LengthStringSB			;Find length for PutStringSB
			BL      PutStringSB				;
			MOVS    R0,#0x0D				;Carriage Return
			BL      PutChar					;
			MOVS    R0,#0x0A				;Line Feed
			BL      PutChar					;
			B       Main				    ;Let them try again
;>>>>>   end main program code <<<<<
;Stay here
            B       .
			ALIGN
			LTORG
;>>>>> begin subroutine code <<<<<

;This subroutine prints the least significant unsigned 
;byte value from R0 to the screen. 
;Inputs:
	;R0 = value to print to the terminal screen in UB form
;Outputs
	;None
PutNumUB
			PUSH    {R1,LR}
			MOVS    R1,#0xFF				;Mask
			ANDS    R0,R0,R1				;Mask off everything but the last byte
			BL      PutNumU					;Call PutNumU
			POP     {R1,PC}
            
;Timer Interrupt Service Routine
;THis subroutine increments a counter when RunStopWatch is equal to 0
;Otherwise it does not 
PIT_ISR
			CPSID	I
			LDR 	R0,=RunStopWatch			;Initialize
			LDRB 	R0,[R0,#0]
			CMP 	R0,#0 						;If 1
			BNE 	INCR_COUNT
			B 		END_PIT_ISR					;If 0
			
INCR_COUNT
			LDR 	R0,=Count  				;Add #1 to count if stopwatch is running
			LDR 	R1,[R0,#0]
			ADDS 	R1,R1,#1
			STR 	R1,[R0,#0] 				;Store value

END_PIT_ISR
			LDR 	R0,=PIT_TFLG0 			;;Clear interrupt condition
			LDR 	R1,=PIT_TFLG_TIF_MASK
			STR 	R1,[R0, #0]
			CPSIE 	I
			BX 		LR

;Interrupt service routine for UART0 
;Check status of interrupt that triggered the ISR
;And react appropriately. If transmit interrupt enabled,
;write to UART0 transmit data register. If rx enabled
;enqueue to transmit queue from UART0 recieve data register
UART0_ISR
			CPSID I							;Mask other interrupts
			PUSH {LR}
			LDR R0, =UART0_BASE				;Base
			;If txinterrupt enabled (UART0_C2 Bit 7 is set)
			LDRB R1,[R0,#UART0_C2_OFFSET]
			MOVS R2,#UART0_C2_TIE_MASK
			ANDS R1, R1, R2
			BNE TX_ENABLED
			;If no TxInterrupt, check for Rx
			B CHECK_RX_INT
TX_ENABLED
			LDRB R1,[R0,#UART0_S1_OFFSET]
			MOVS R2, #UART0_S1_TDRE_MASK
			ANDS R1, R1, R2
			BEQ CHECK_RX_INT
			;Dequeue character
			;Load input params to initalize queue structure
			LDR R1, =TxQueueRecord
			MOVS R2, #MAX_QUEUE
			BL Dequeue
			;Dequeue was unsuccessful
			BCS DISABLE_TX
			;Dequeue was successful
			LDR R1, =UART0_BASE
			;Transmit Character Stored in R0
			STRB R0, [R1, #UART0_D_OFFSET]
			B CHECK_RX_INT
DISABLE_TX
			;UART0 <- C2_T_RI
			MOVS R1,#UART0_C2_T_RI
            STRB R1,[R0,#UART0_C2_OFFSET]
CHECK_RX_INT
			LDR R0, =UART0_BASE
			;Check if an RxInterrupt exists
			LDRB R1,[R0,#UART0_S1_OFFSET]
			MOVS R2, #UART0_S1_RDRF_MASK
			ANDS R1, R1, R2
			BEQ END_ISR
			;Receive character and store in R0
			LDR R0, =UART0_BASE
			LDRB R0,[R0, #UART0_D_OFFSET]
			;Enqueue character with character stored in R0
			;Load input params to initalize queue structure
			LDR R1, =RxQueueRecord
			BL Enqueue
			;No need to check return of EnQueue
			;character will be lost if the queue is full!
END_ISR
			;pop relevant registers off the stack
			;Unmask other interrupts
			CPSIE I
			;Return back to our business
			POP {PC}

;This subroutine prints  the hex representation of a value
;in r0 to the console
;Inputs:
;	R0 - Value to print to the screen
;Outputs
;	None
PutNumHex
        PUSH    {R2,R3,R4,LR}
        MOVS    R2,#32      ;Initialize counter
PutNumHexLoop
        CMP     R2,#0       ;Compare counter to 0
        BLT     EndPutNumHex      
        MOVS    R3,R0       ;Move value tp r3
		MOVS    R4,#0x0F    ;Value to shift
		LSRS    R3,R2		;Shift to right most 
		ANDS    R4,R4,R3	;Mask	
        CMP     R4,#10      ;Compare to ascii
        BGE     PrintLetter ;Its a letter        
        ADDS    R4,#'0'     ;Its a number so print number
        B       PrintNumber       
PrintLetter        
        ADDS    R4,R4,#55   ;Convert to ascii     
PrintNumber
        PUSH    {R0}        ;Save R0 value
        MOVS    R0,R4       ;Initialize for put char
        BL      PutChar
        POP     {R0}        ;Restore original value
        ;Reset value in R3 and increment loop counter
        MOVS    R4,#0       ;Reset R4
        SUBS    R2,R2,#4    ;Decrement counter
        B       PutNumHexLoop       
EndPutNumHex      
        POP     {R2,R3,R4,PC}
;This subroutine initializes an empty queue
;Input parameters:
;   R0, queue buffer address 
;   R1, queue record structure
;	R2, queue size
;Output parameters:
;   None 
;Modified Registers
;	None 
InitQueue
		STR		R0,[R1,#IN_PTR]		;Initialize In pointer to start of queue
		STR		R0,[R1,#OUT_PTR]	;Initialize Out pointer to start of queue
		STR		R0,[R1,#BUF_START]	;Initialize buffer start to start of queue
		ADDS 	R0,R0,R2			;Add buff start to queue size
		STR 	R0,[R1,#BUF_PAST]	;store as buff past
		STR 	R2,[R1,#BUF_SIZE]	;Store queue size as buff size
		MOVS 	R0,#0				;Initialize num enqd as zero
		STRB 	R0,[R1,#NUM_ENQD]	;store num enqd 
		BX	    LR
;This subroutine dequeues a character
;Input paramters:
;	R1: Address of queue record structure
;Output:
;	PSR C flag: success(0) or failure (1)
;	r0: Character to dequeued 
;Modify: PSR
Dequeue
		PUSH	{R0-R4}
		LDRB	R3,[R1,#NUM_ENQD]	;get number enqueued
		CMP		R3,#0				;Compare num enqued and 0
		BLS		QueueEmpty			;Branch if num enqd =< 0
		LDR 	R0,[R1,#OUT_PTR]	;get adress of outpointer
		LDRB	 R0,[R0,#0]			;Load actual value from queue
		LDRB 	R3,[R1,#NUM_ENQD]	;Load number enqued
		SUBS 	R3,R3,#1			;Decrement num enqd
		STRB 	R3,[R1,#NUM_ENQD] 	;Store new num enqd value
		LDR 	R3,[R1,#OUT_PTR]	;Load outpointer
		ADDS 	R3,R3,#1			;Increment outpointer
		STR 	R3,[R1,#OUT_PTR] 	;store new outpointer
		LDR 	R4,[R1,#BUF_PAST]	;Load buffer past
		CMP 	R3,R4				;Compare to outpointer
		BGE     NeedToWrapAgain		;If outpointer >= BuffPast
		B 	    ClearPSRDequeue
		
NeedToWrapAgain
		LDR     R3,[R1,#BUF_START]	;Load buffewr start
		STR     R3,[R1,#OUT_PTR]	;Store buffer start as outpointer
ClearPSRDequeue
		MRS     R1,APSR				;Clear the PSR C flag
		MOVS    R3,#0x20
		LSLS    R1,R1,#24
		BICS    R1,R1,R3
		MSR	    APSR,R1
		B EndDequeue
QueueEmpty
		MRS		R0,APSR				;Set the PSR C flag
		MOVS	R1,#0x20
		LSLS	R1,R1,#24
		ORRS	R0,R0,R1
		MSR		APSR, R0
		B		EndDequeue			
EndDequeue
		POP		{R1-R4}
		BX		LR
		
;This subroutine enqueues a character
;Input paramters:
;	r0: Character to enque 
;	R1: Address of queue record structure
;Output:
;	PSR C flag: success(0) or failure (1)
;Modify: PSR
Enqueue
		PUSH	{R2-R4}
		LDRB 	R3,[R1, #NUM_ENQD]	;Load number enqued
		LDRB 	R4,[R1, #BUF_SIZE]	;Load buffer size
		CMP 	R3,R4				;Compare
		BGE 	QueueFull			;If number enqd >= buffsize then Queue full
		B		BeginEnqueue
QueueFull
		MRS 	R1,APSR				;Set PSR C flag to 1		
		MOVS 	R3,#0x20
		LSLS 	R3,R3,#24
		ORRS 	R1,R1,R3
		MSR 	APSR,R1
		B 		EndEnqueue
		
BeginEnqueue
		LDR 	R3,[R1, #IN_PTR]	;Load inpointer
		STRB 	R0,[R3, #0]			;Store character at inpointer
		ADDS 	R3,R3,#1			;Increment inpointer
		STR 	R3,[R1,#IN_PTR]		;Store inpointer
		LDRB 	R3,[R1,#NUM_ENQD]	;Load num enqd
		ADDS 	R3,R3,#1			;Increment num enqd
		STRB 	R3,[R1,#NUM_ENQD]	;Store num enqd
		LDR 	R3,[R1,#IN_PTR]		;Load inpointer
		LDR 	R4,[R1,#BUF_PAST]	;Load buffer past
		CMP 	R3,R4				;If inpointer >= buff past					
		BGE 	NeedToWrap
		MRS 	R2,APSR				;Clear C flag of PSR
		MOVS 	R3,#0x20
		LSLS 	R2,R2,#24
		BICS 	R2,R2,R3
		MSR		APSR,R2	
		B 		EndEnqueue
								
NeedToWrap
		LDR 	R2,[R1,#BUF_START]	;Load buff start
		STR 	R2,[R1,#IN_PTR]		;Store buff start as inpointer
		MRS 	R2,APSR				;Clear the PSR C flag 
		MOVS 	R3,#0x20
		LSLS 	R2,R2,#24
		BICS 	R2,R2,R3
		MSR		APSR,R2		
EndEnqueue		
		POP     {R2,R3,R4}
		BX      LR


;This subroutine sends a character out of UART0 using interrupts
;Inputs
;	R0 - Character to enqueue to TxQueue
;Return 
;	None

PutChar
		PUSH    {R0,R1,LR}
		LDR R1, =TxQueueRecord
REPEAT_ENQ
		;Mask all other interrupts
		CPSID I
		;Critical section -> enqueue character
		;Enqueue character that's already in R0
		BL	Enqueue
		;Enable interrupts
		CPSIE I
		BCS REPEAT_ENQ
		;Enable UART0 Transmitter, reciever, and rx interrupt
		LDR     R0,=UART0_BASE
		MOVS    R1,#UART0_C2_TI_RI
        STRB    R1,[R0,#UART0_C2_OFFSET]
        ;Pop original register values off the stack
		POP     {R0,R1,PC}
           
;This subroutine receives a character from UART0 using interrupts
;Inputs 
;	None
;Return
; R0 - Character dequeued from RxQueue
GetChar
	PUSH {R1,LR} ; Push varibles on the stack to avoid loss
	LDR R1, =RxQueueRecord
REPEAT_DEQ
	;Mask all interrupts
	CPSID I	
	;Critical code section - dequeue
	BL Dequeue
	;Re enable interrupts
	CPSIE I
	BCS REPEAT_DEQ
	POP {R1,PC}


;The follwing subroutine reads a string of chars from the keyboard
;and stores them in a string dtarting from R0. For each character 
;up to R1-1 it echoes the character and when the carage character (13-10 0D-16)
;is not recieved it null terminates the string. it then moves
;the cursor to the begining of the next line
;Input parameters:
;   R1 one more than the oveflow buffer
;	R0 pointer to address of string 
;Output parameters:
;   None
;Modified Registers
;   None. The original contents of the registers are restored 
;               		
GetStringSB
			PUSH 	{R1,R2,R3,LR}
			MOVS 	R2,#0 		;Initalize string offset to zero
GetStringSBLoop
			PUSH 	{R0}		;Push Pointer value on stack
			BL   	GetChar	;Get next char
			MOVS 	R3,R0		;Move char to r3
			POP 	{R0}		;Return pointer value to R0
			CMP 	R3,#13		;Compare char to carrage return
			BEQ 	EndGetStringSB
			CMP 	R1,#0		;If there is no more character
			BEQ 	GetStringSBLoop
			PUSH 	{R0} 		;Save Pointer value on stack
			MOVS 	R0,R3 		;move char to r3 for putchar
			BL   	PutChar
			POP 	{R0}		;Restore pointer value to R0
			STRB 	R3,[R0,R2]	;Store char at pointer with offset
			SUBS 	R1,R1,#1	;Decrement number of chars
			ADDS 	R2,R2,#1	;Increment offset
			B 		GetStringSBLoop
EndGetStringSB
		
			MOVS 	R3,#0		;null
			STRB 	R3,[R0,R2]	;null terminate
			POP 	{R1,R2,R3,PC}

		
;Displays a null terminated string to the terminal screen from memory starting at R0 
;Preventing overun of the buffer capacity specified by R1
;Leaves cursor specified in R1

PutStringSB
		PUSH	{R0-R3,LR}	
		MOVS	R2,R0				;Move adress to R2
		MOVS	R3,#0
ReadChar
		CMP		R1, #0				;If string is empty
        BEQ 	EndPutStringSB
        LDRB 	R0,[R2,R3]			;Put char in R3
		BL 		PutChar
        SUBS    R1,R1,#1			;Decrement number of chars
		ADDS	R3,R3,#1			;Increment offset
        B 		ReadChar	
EndPutStringSB
		POP 	{R0-R3,PC}

;Determines how many characters are in a a null terminated string
;Memory of string starts in R0 and returns number of characters in R2
;Prevents overrun of of buffer capacity specified in R1
;
;Input parameters:
;	R0 pointer to address of string 
;Output parameters:
;   R1: number of characters in String
;Modified Registers
;   None. The original contents of the registers are restored 
;

LengthStringSB
        PUSH    {R0,R2,R3,R4,LR}
        MOVS 	R1,#MAX_STRING
		MOVS 	R2,#0				;Initalize length to 0
		MOVS 	R4,#0				;Initalize offset to 0
LengthStringSBLoop
		CMP 	R2,R1				;Compare Max_String to 0
        BGE 	EndLengthStringSB	;If Max_string < 0
        LDRB 	R3,[R0,R4]			;Load first character in string
		CMP 	R3,#0				;Compare char to 0
		BEQ 	EndLengthStringSB	;If null terminated then end
		ADDS 	R4,R4,#1 			;Increment offset
		ADDS 	R2,R2,#1			;Increment lenghth 
        B 		LengthStringSBLoop
EndLengthStringSB
		MOVS	R1,R2				;Move length to R1
		POP 	{R0,R2,R3,R4,PC}

;Stuff		
DIVU        
        PUSH    {R2}        		;store R2 Value
        MOVS    R2,#0       		;move 0 to R2 for quotient
        CMP     R0,#0       		;Compare divisor to 0
        BEQ     SETCARRY    		;if divisor = 0 go to SETCARRY
WHILE       
        CMP     R1,R0       		;Compare R1 to R0
        BLO     ENDWHILE    		;if dividend<Divisor End loop
        ADDS    R2,R2,#1    		;Add 1 to quotient
        SUBS    R1,R1,R0    		;Dividend - divisor
        B       WHILE       		;branch to start of while  
ENDWHILE    
        MOVS    R0,R2       		;move quotient to R0, so R0 remainder R1
        POP     {R2}        		;revert R2 to value before subroutine
        PUSH    {R0,R1}     		;push R0 and R1
        MRS     R0,APSR     		;Set C flag to 0
        MOVS    R1,#0x20    		;
		LSLS    R1,#24				;Shift 24 places (to most significant byte)
        BICS    R0,R0,R1    		; 
        MSR     APSR,R0    			; 
        POP     {R0,R1}     		;revert R0 and R1 to answer 
        BX      LR          		;Go back to program
SETCARRY    
		POP     {R2}				;Pop R2
        PUSH    {R0,R1}     		;Store R0 and R1
        MRS     R0,APSR     		;Set C flag to 1
        MOVS    R1,#0x20    		;
		LSLS    R1,#24				;Shift 24 places (to most significant byte)
        ORRS    R0,R0,R1    		;
        MSR     APSR,R0     		;
        POP     {R0,R1}     		;Revert R0 and R1 to answer
        BX      LR 

;Prints the text in decimal of unsigned word value R0 in terminal screen
;Using DivU
;Input parameters:
;   R0 is an Unsigned Word Value to print
;Output parameters:
;   None
;Modified Registers
;   PSR: (after return, nothing else)
PutNumU
		PUSH	{R0,R1,LR}				;Push registers as to not overwrite original values
		MOVS    R1,R0
        MOVS 	R0,#10              ;Initialize divisor
		BL 		DIVU    			;Divide binary value by 10, remainder (which is R1) is ASCii
        ADDS    R0,R0,#48           ;Convert MSD to Ascii
        CMP     R0,#48
        BEQ     NEXTDIGIT
		BL 		PutChar             ;Display on terminal, R1 is input for R1
NEXTDIGIT
        ADDS    R1,R1,#48           ;Convert LSD to ASCII
        MOVS    R0,R1               ;Initialize PUTCHAR
		MOVS 	R0, #0x0D
		BL 		PutChar
		MOVS 	R0, #0x0A
		BL 		PutChar
        BL      PutChar             ;Print last digit
        POP     {R0,R1,PC}

		
Init_UART0_IRQ
;Initalize UART0 for Serial Driver
			;Allocate R0-2 for Ri=k 
			;Store prevoius values for restoration
			PUSH {R0, R1, R2, LR}
			;Initalize rxQueue
			LDR R1, =RxQueueRecord
			LDR R0, =RxQueue
			MOVS R2, #Q_BUF_SZ
			BL InitQueue
			LDR R1, =TxQueueRecord
			LDR R0, =TxQueue
			MOVS R2, #Q_BUF_SZ
			BL InitQueue
		    ;Select MCGPLLCLK / 2 as UART0 clock source
		     LDR R0,=SIM_SOPT2
		     LDR R1,=SIM_SOPT2_UART0SRC_MASK
		     LDR R2,[R0,#0]
		     BICS R2,R2,R1
		     LDR R1,=SIM_SOPT2_UART0_MCGPLLCLK_DIV2
		     ORRS R2,R2,R1
		     STR R2,[R0,#0]
		    ;Enable external connection for UART0
		     LDR R0,=SIM_SOPT5
		     LDR R1,= SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
		     LDR R2,[R0,#0]
		     BICS R2,R2,R1
		     STR R2,[R0,#0]
		    ;Enable clock for UART0 module
		     LDR R0,=SIM_SCGC4
		     LDR R1,= SIM_SCGC4_UART0_MASK
		     LDR R2,[R0,#0]
		     ORRS R2,R2,R1
		     STR R2,[R0,#0]
		    ;Enable clock for Port A module
		     LDR R0,=SIM_SCGC5
		     LDR R1,= SIM_SCGC5_PORTA_MASK
		     LDR R2,[R0,#0]
		     ORRS R2,R2,R1
		     STR R2,[R0,#0]
		    ;Connect PORT A Pin 1 (PTA1) to UART0 Rx (J1 Pin 02)
		     LDR R0,=PORTA_PCR1
		     LDR R1,=PORT_PCR_SET_PTA1_UART0_RX
		     STR R1,[R0,#0]
		    ;Connect PORT A Pin 2 (PTA2) to UART0 Tx (J1 Pin 04)
		     LDR R0,=PORTA_PCR2
		     LDR R1,=PORT_PCR_SET_PTA2_UART0_TX
		     STR R1,[R0,#0] 
		     ;Disable UART0 receiver and transmitter
		     LDR R0,=UART0_BASE
		     MOVS R1,#UART0_C2_T_R
		     LDRB R2,[R0,#UART0_C2_OFFSET]
		     BICS R2,R2,R1
		     STRB R2,[R0,#UART0_C2_OFFSET]
		     ;Init NVIC for UART0 Interrupts
		     ;Set UART0 IRQ Priority
		     LDR R0, =UART0_IPR
			 LDR R1, =NVIC_IPR_UART0_MASK
		     LDR R2, =NVIC_IPR_UART0_PRI_3
		     LDR R3, [R0, #0]
			 BICS R3, R3, R1
             ORRS R3, R3, R2
	         STR R3, [R0, #0]
             ;Clear any pending UART0 Interrupts
		     LDR R0, =NVIC_ICPR
	 	     LDR R1, =NVIC_ICPR_UART0_MASK
		     STR R1, [R0, #0]
		     ;Unmask UART0 interrupts
             LDR R0, =NVIC_ISER
		     LDR R1, =NVIC_ISER_UART0_MASK
		     STR R1, [R0, #0]
			 ;Init UART0 for 8N1 format at 9600 Baud,
			 ;and enable the rx interrupt
			 LDR R0, =UART0_BASE
			 MOVS R1,#UART0_BDH_9600
             STRB R1,[R0,#UART0_BDH_OFFSET]
             MOVS R1,#UART0_BDL_9600
             STRB R1,[R0,#UART0_BDL_OFFSET]
             MOVS R1,#UART0_C1_8N1
             STRB R1,[R0,#UART0_C1_OFFSET]
             MOVS R1,#UART0_C3_NO_TXINV
             STRB R1,[R0,#UART0_C3_OFFSET]
             MOVS R1,#UART0_C4_NO_MATCH_OSR_16
             STRB R1,[R0,#UART0_C4_OFFSET]
             MOVS R1,#UART0_C5_NO_DMA_SSR_SYNC
             STRB R1,[R0,#UART0_C5_OFFSET]
             MOVS R1,#UART0_S1_CLEAR_FLAGS
             STRB R1,[R0,#UART0_S1_OFFSET]
             MOVS R1, \
             #UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
             STRB R1,[R0,#UART0_S2_OFFSET] 
			 ;Enable UART0 Transmitter, reciever, and rx interrupt
			 MOVS R1,#UART0_C2_T_RI
             STRB R1,[R0,#UART0_C2_OFFSET] 
	       	 ;Pop prevous R0-2 values off the stack.
			 POP {R0, R1, R2, PC}
;This subroutine initializes PIT to generate an interupt
;every 0.01s from channel 0
;Inputs: 
;	None
;Outputs: 
;	None
Init_PIT_IRQ
			CPSID I
			PUSH {R0, R1, R2, LR}
			LDR R0, =SIM_SCGC6 				;Initialize
			LDR R1, =SIM_SCGC6_PIT_MASK
			LDR R2, [R0, #0]
			ORRS R2, R2, R1					;Set only the PIT bit on SIM_SCGC6
			STR R2, [R0, #0]				;Store set bit back on to the register
			LDR R0, =PIT_CH0_BASE			;Disable timer 0 
			LDR R1, =PIT_TCTRL_TEN_MASK
			LDR R2, [R0, #PIT_TCTRL_OFFSET]
			BICS R2, R2, R1
			STR R2, [R0, #PIT_TCTRL_OFFSET]
			LDR R0, =PIT_IPR  				;Set PIT Interrupt Priority
			LDR R1, =(PIT_IRQ_PRI << PIT_PRI_POS)
			STR R1, [R0, #0]
			LDR R0, =NVIC_ISER				;Unmask PIT Interrupts
			LDR R1, =PIT_IRQ_MASK
			STR R1, [R0, #0]
			LDR R0, =PIT_BASE 				;Enable the PIT timer module
			LDR R1, =PIT_MCR_EN_FRZ			;Enable the FRZ to stop timer in debug mode
			STR R1, [R0, #PIT_MCR_OFFSET]
			LDR R0, =PIT_CH0_BASE			;Request interrupts every 0.01 seconds
			LDR R1, =PIT_LDVAL_10ms ;239,999
			STR R1, [R0, #PIT_LDVAL_OFFSET]
			LDR R0, =PIT_CH0_BASE			;Enable PIT timer channel 0 for interrupts
			MOVS R1, #PIT_TCTRL_CH_IE		;Interrupt enabled mask to write to the register
			STR R1, [R0, #PIT_TCTRL_OFFSET]
			
			
			CPSIE I
			POP {R0, R1, R2, PC}
;>>>>>   end subroutine code <<<<<
            ALIGN
;****************************************************************
;Vector Table Mapped to Address 0 at Reset
;Linker requires __Vectors to be exported
            AREA    RESET, DATA, READONLY
            EXPORT  __Vectors
            EXPORT  __Vectors_End
            EXPORT  __Vectors_Size
            IMPORT  __initial_sp
            IMPORT  Dummy_Handler
__Vectors 
                                      ;ARM core vectors
            DCD    __initial_sp       ;00:end of stack
            DCD    Reset_Handler      ;01:reset vector
            DCD    Dummy_Handler      ;02:NMI
            DCD    Dummy_Handler      ;03:hard fault
            DCD    Dummy_Handler      ;04:(reserved)
            DCD    Dummy_Handler      ;05:(reserved)
            DCD    Dummy_Handler      ;06:(reserved)
            DCD    Dummy_Handler      ;07:(reserved)
            DCD    Dummy_Handler      ;08:(reserved)
            DCD    Dummy_Handler      ;09:(reserved)
            DCD    Dummy_Handler      ;10:(reserved)
            DCD    Dummy_Handler      ;11:SVCall (supervisor call)
            DCD    Dummy_Handler      ;12:(reserved)
            DCD    Dummy_Handler      ;13:(reserved)
            DCD    Dummy_Handler      ;14:PendableSrvReq (pendable request 
                                      ;   for system service)
            DCD    Dummy_Handler      ;15:SysTick (system tick timer)
            DCD    Dummy_Handler      ;16:DMA channel 0 xfer complete/error
            DCD    Dummy_Handler      ;17:DMA channel 1 xfer complete/error
            DCD    Dummy_Handler      ;18:DMA channel 2 xfer complete/error
            DCD    Dummy_Handler      ;19:DMA channel 3 xfer complete/error
            DCD    Dummy_Handler      ;20:(reserved)
            DCD    Dummy_Handler      ;21:command complete; read collision
            DCD    Dummy_Handler      ;22:low-voltage detect;
                                      ;   low-voltage warning
            DCD    Dummy_Handler      ;23:low leakage wakeup
            DCD    Dummy_Handler      ;24:I2C0
            DCD    Dummy_Handler      ;25:I2C1
            DCD    Dummy_Handler      ;26:SPI0 (all IRQ sources)
            DCD    Dummy_Handler      ;27:SPI1 (all IRQ sources)
            DCD    UART0_ISR      	  ;28:UART0 (status; error)
            DCD    Dummy_Handler      ;29:UART1 (status; error)
            DCD    Dummy_Handler      ;30:UART2 (status; error)
            DCD    Dummy_Handler      ;31:ADC0
            DCD    Dummy_Handler      ;32:CMP0
            DCD    Dummy_Handler      ;33:TPM0
            DCD    Dummy_Handler      ;34:TPM1
            DCD    Dummy_Handler      ;35:TPM2
            DCD    Dummy_Handler      ;36:RTC (alarm)
            DCD    Dummy_Handler      ;37:RTC (seconds)
            DCD    PIT_ISR			  ;38:PIT (all IRQ sources)
            DCD    Dummy_Handler      ;39:I2S0
            DCD    Dummy_Handler      ;40:USB0
            DCD    Dummy_Handler      ;41:DAC0
            DCD    Dummy_Handler      ;42:TSI0
            DCD    Dummy_Handler      ;43:MCG
            DCD    Dummy_Handler      ;44:LPTMR0
            DCD    Dummy_Handler      ;45:Segment LCD
            DCD    Dummy_Handler      ;46:PORTA pin detect
            DCD    Dummy_Handler      ;47:PORTC and PORTD pin detect
__Vectors_End
__Vectors_Size  EQU     __Vectors_End - __Vectors
            ALIGN
;****************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
TimeString			DCB " x 0.01 s", 0
AccessCodePrompt	DCB "Enter the access code.", 0
Granted				DCB "--Access granted", 0
Denied				DCB "--Access denied", 0
Mission_Completed	DCB "Mission completed!", 0
AccessCode			DCB "25015110", 0
;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
;Rx Queue
RxQueue 	  SPACE MAX_QUEUE	 
RxQueueRecord SPACE Q_REC_SZ
	
			ALIGN
;Tx Queue
TxQueue 	  SPACE MAX_QUEUE	
TxQueueRecord SPACE Q_REC_SZ
			ALIGN
;Queue	
Queue 		SPACE Q_BUF_SZ	
QueueRecord SPACE Q_REC_SZ
			ALIGN	
StringReversal	SPACE 2
			ALIGN
RunStopWatch	SPACE 1
			ALIGN
Count			SPACE 2
			ALIGN
InputString		SPACE MAX_STRING
;>>>>>   end variables here <<<<<
            ALIGN
            END