;TTL Multiprecision Arithmetic with Mixed C and Assembly Languag Programming 
;****************************************************************
;Descriptive comment header goes here.
;This exercise investagates a KL46 program consisting of both C and assembly
;language components. The objective of this exercise is to perform multi-word number I/O 
;and addition. 
;Name:  John Judge
;Date: 4/28/16
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
	
;****************************************************************
;Program
;Linker requires Reset_Handler
			PRESERVE8
            AREA    MyCode,CODE,READONLY
            EXPORT  AddIntMultiU
            EXPORT  GetStringSB
            EXPORT  PutStringSB
            EXPORT  Init_UART0_IRQ
;>>>>> begin subroutine code <<<<<
;This subroutine adds the n-word unsigned number in register R2 to 
;the n-word unsigned numbe rin register R1. The result
;is then stored in memory at address R0. The value located
;in R3 is the number of words of each number that will be added.
;R0 is then overwritten with either a 0 for success or the 
;value of 1 for failure (overflow)

;Load in values one word (register) at a time
;And add using ADCS to utilize the state of the ASPR c 
;bit when carrying operations over.

;Inputs:
	;R0 = mem address to store result added number
    ;R1 = mem address of number 1 to add
    ;R2 = mem address of number 2 to add
    ;R3 = size (number of words) that numbers in R1 and R2 coorespond to
;Outputs
	;R0 = Status value (0 for success and 1 for overflow)
AddIntMultiU
			PUSH 	{R1-R7}
            MOVS 	R5,#0			;Initalize offset to 0
			MOVS	R4,#4
			MULS	R3,R4,R3		;Multiply words by 4 to compare
			ADDS	R5,R5,#0		; clears c flag
			MRS		R4,APSR			;Preserves APSR
AddIntMultiUloop
			LDR 	R6,[R1,R5]		;Load first number
            LDR 	R7,[R2,R5]		;Load second number
			MSR		APSR,R4			;Restore APSR
			ADCS	R6,R6,R7		
			STR 	R6,[R0,R5]		;Store the result
			MRS		R4,APSR			;Preserve APSR
			ADDS	R5,R5,#4		;Increment Offset
            CMP 	R3,R5
            BEQ 	EndAddIntMultiUloop
            B 		AddIntMultiUloop		;Continue adding
	
CheckForOverflow
            MOVS 	R0,#1			;Signal overlflow by settin R1
			POP 	{R1-R7}			;Restore values
			BX		LR

EndAddIntMultiUloop
            MSR		APSR,R4
			BCS 	CheckForOverflow	;If carry set then theres overflow
            MOVS	R0,#0 
            POP 	{R1-R7}			;Restore values
			BX		LR


UART0_ISR
			;description: UART0 Interrupt Service routine. Checks what triggered ISR and acts accordingly.
			;subroutines: Dequeue, Enqueue
			;Input:   none
			;Output:  none
			;Modify:  none
			CPSID	I				; mask interrupts
            PUSH    {LR}            ; preserve link register
			PUSH	{R4,R5}	; preserve registers
			LDR		R1,=UART0_BASE	; base address
			;Interrupt source can be found in UART0_S1
checkTX     LDRB    R4,[R1,#UART0_C2_OFFSET] ; loads current UART0 C2 settings
            MOVS    R5,#UART0_C2_TIE_MASK 	; Transmit Interrupt Enabled
            ANDS    R5,R5,R4    ; turns off everything except pin 7, if it was on to begin with
            BEQ     checkRX     ; if  pin 7 was off, TX interrupt is disabled
            LDRB	R4,[R1,#UART0_S1_OFFSET] ; load S1 to check TDRE
			MOVS	R5,#UART0_S1_TDRE_MASK		; load tdre mask
            ANDS    R4,R4,R5
            BEQ		checkRX 		; if not, check receive!
            ;otherwise, it is set! Let's dequeue character from TxQBuffer
            LDR     R1,=TxQueueRecord
            BL      Dequeue
            BCS		ISRtxfail	; if the dequeue failed (BUFFER IS EMPTY), branch! otherwise, run
            ; If it succeeded, write character to UART0 transmit data reg.
            LDR     R1,=UART0_BASE  ; base address
            STRB    R0,[R1,#UART0_D_OFFSET] ; stores character from dequeue
            B		checkRX      ; quit
ISRtxfail	; if TxQBuffer is empty, disable Tx
            LDR     R1,=UART0_BASE              ; base address
            LDR     R4,=UART0_C2_T_RI			; address of value to store
            STRB	R4,[R1,#UART0_C2_OFFSET]	; offset for transmit/recieve
checkRX     LDRB    R4,[R1,#UART0_S1_OFFSET]
            MOVS    R5,#UART0_S1_RDRF_MASK
            ANDS    R5,R5,R4
            BEQ     ISRend      ; if not, get out!
            ; otherwise, let's enqueue this character
            LDRB    R0,[R1,#UART0_D_OFFSET] ; gets character from data recieve register
            LDR     R1,=RxQueueRecord	; sets queue to enqueue into as RxQRecord
            BL      Enqueue			; enqueues the character from R0 into RxQRecord
            ; character is lost if RxQueue is full
ISRend		POP     {R4,R5} ; restore registers
            CPSIE	I			; unmask interrupts
			POP		{PC}		; return to where it was called from

 

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
		PUSH	{R3-R6}
		LDRB	R3,[R1,#NUM_ENQD]	;get number enqueued
		CMP		R3,#0				;Compare num enqued and 0
		BEQ		QueueEmpty			;Branch if num enqd =< 0
		LDR 	R5,[R1,#OUT_PTR]	;get adress of outpointer
		LDR 	R6,[R1,#BUF_PAST]	;Load buffer past
		LDRB	R0,[R5,#0]			;Load actual value from queue
		SUBS 	R3,R3,#1			;Decrement num enqd
		STRB 	R3,[R1,#NUM_ENQD] 	;Store new num enqd value
		ADDS 	R5,R5,#1			;Increment outpointer
		CMP 	R5,R6				;Compare to outpointer
		BLO		ClearPSRDequeue
		LDR     R5,[R1,#BUF_START]	;Load buffewr start
		
ClearPSRDequeue
		STR     R5,[R1,#OUT_PTR]	;Store buffer start as outpointer
		MRS     R3,APSR				;Clear the PSR C flag
		MOVS    R4,#0x20
		LSLS    R4,R4,#24
		BICS    R3,R3,R4
		MSR	    APSR,R3
		B 		EndDequeue
QueueEmpty
		MRS		R3,APSR				;Set the PSR C flag
		MOVS	R4,#0x20
		LSLS	R4,R4,#24
		ORRS	R3,R3,R4
		MSR		APSR, R3
		B		EndDequeue			
EndDequeue
		POP		{R3-R6}
		BX		LR
		
;This subroutine enqueues a character
;Input paramters:
;	r0: Character to enque 
;	R1: Address of queue record structure
;Output:
;	PSR C flag: success(0) or failure (1)
;Modify: PSR
		
;This subroutine enqueues a character
;Input paramters:
;	r0: Character to enque 
;	R1: Address of queue record structure
;Output:
;	PSR C flag: success(0) or failure (1)
;Modify: PSR
Enqueue
		PUSH	{R3-R6}
		LDRB 	R3,[R1, #NUM_ENQD]	;Load number enqued
		LDRB 	R4,[R1, #BUF_SIZE]	;Load buffer size
		CMP 	R3,R4				;Compare
		BGE 	QueueFull			;If number enqd >= buffsize then Queue full
		LDR 	R5,[R1, #IN_PTR]	;Load inpointer
		LDR 	R6,[R1,#BUF_PAST]	;Load buffer past
		STRB 	R0,[R5, #0]			;Store character at inpointer	
		ADDS 	R3,R3,#1			;Increment inpointer
		STRB 	R3,[R1,#NUM_ENQD]	;Store num enqd
		ADDS 	R5,R5,#1			;Increment num enqd
		CMP 	R5,R6				;If inpointer >= buff past
		BLO		SetCarry
		LDR 	R5,[R1,#BUF_START]	;Load buff start
SetCarry
		STR 	R5,[R1,#IN_PTR]		;Store buff start as inpointer
		MRS 	R3,APSR				;Clear the PSR C flag 
		MOVS 	R4,#0x20
		LSLS 	R4,R4,#24
		BICS 	R3,R3,R4
		MSR		APSR,R3
		B 		EndEnqueue

QueueFull
		MRS 	R3,APSR				;Set PSR C flag to 1		
		MOVS 	R4,#0x20
		LSLS 	R4,R4,#24
		ORRS 	R3,R3,R4
		MSR 	APSR,R3
		
EndEnqueue		
		POP     {R3-R6}
		BX      LR

;This subroutine sends a character out of UART0 using interrupts
;Inputs
;	R0 - Character to enqueue to TxQueue
;Return 
;	None

PutChar
		PUSH    {R1,R2,LR}
		LDR 	R1,=TxQueueRecord
REPEAT_ENQ
        ;Mask all other interrupts
		CPSID 	I
		;Critical section -> enqueue character
		;Enqueue character that's already in R0
		BL		Enqueue
		;Enable interrupts
		CPSIE 	I
		BCS 	REPEAT_ENQ
		;Enable UART0 Transmitter, reciever, and rx interrupt
		LDR     R1,=UART0_BASE
		MOVS    R2,#UART0_C2_TI_RI
        STRB    R2,[R1,#UART0_C2_OFFSET]
        ;Pop original register values off the stack
		POP     {R1,R2,PC}
           
;This subroutine receives a character from UART0 using interrupts
;Inputs 
;	None
;Return
; R0 - Character dequeued from RxQueue
GetChar
	PUSH 	{R1,LR} ; Push varibles on the stack to avoid loss
	LDR 	R1,=RxQueueRecord
REPEAT_DEQ
	;Mask all interrupts
	CPSID	I	
	;Critical code section - dequeue
	BL 		Dequeue
	;Re enable interrupts
	CPSIE 	I
	BCS 	REPEAT_DEQ
	POP 	{R1,PC}


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
			PUSH 	{R0,R1,R2,R3,LR}
			MOVS 	R2,#0 		;Initalize string offset to zero
			SUBS	R1,R1,#1
GetStringSBLoop
			PUSH 	{R0}		;Push Pointer value on stack
			BL   	GetChar		;Get next char
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
			POP 	{R0,R1,R2,R3,PC}

		
;Displays a null terminated string to the terminal screen from memory starting at R0 
;Preventing overun of the buffer capacity specified by R1
;Leaves cursor specified in R1

PutStringSB
		PUSH	{R0-R3,LR}	
		;BL 		LengthStringSB
		;MOVS 	R1,R2
ReadChar
		CMP		R1,#0				;If string is empty
        BEQ 	EndPutStringSB
        LDRB 	R3,[R0,#0]			;Put char in R3
        CMP     R3,#0x00            ;Check if null
        BEQ     EndPutStringSB      ;End if null
		PUSH 	{R0}
		MOVS	R0,R3
		BL 		PutChar
		POP		{R0}
        SUBS    R1,R1,#1			;Decrement number of chars
		ADDS	R0,R0,#1			;Increment offset
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
		PUSH	{R0,R1,R2,LR}		;Push registers as to not overwrite original values
		MOVS    R2,#0               ;Initialize offset	
DivideNumber
		CMP     R0,#10              ;Compare to 10, Div U divides by 10
		BLT     PutNumUComplete    ;If its less than 10, stop dividing
		MOVS    R1,R0               ;Initialize divided into R1
		MOVS    R0,#10              ;Initialize divisor to 10
		BL      DIVU                ;Divide!
		PUSH    {R0}                ;Save R0's value
		LDR     R0,=StringReversal  ;Reverse! Reverse!
		STRB    R1,[R0,R2]          ;Store in reversed string
		ADDS    R2,R2,#1            ;Increment offset
		POP     {R0}
		B       DivideNumber             ;Keep looping until number is less than 10  
PutNumUComplete
		ADDS    R0,R0,#'0'          ;Convert to ascii 
		BL      PutChar             ;Print it 
		SUBS    R2,R2,#1            ;Decrement offset		
PutNumUPrintChar		
		LDR     R0,=StringReversal  ;Initialize pointer		
		CMP     R2,#0               ;Compare to see if end is reached
		BLT     PutNumUEnd          ;If at end of string then end
		LDRB    R0,[R0,R2]          ;Get char
		ADDS    R0,R0,#'0'          ;Turn to ascii
		BL      PutChar             ;Print
		SUBS    R2,R2,#1            ;Decrement offset/counter
		B       PutNumUPrintChar          ;Loop	
PutNumUEnd
		POP     {R0,R1,R2,PC}

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
        

Init_UART0_IRQ
;Initalize UART0 for Serial Driver
			;Allocate R0-2 for Ri=k 
			;Store prevoius values for restoration
			PUSH {R0, R1, R2, R3, LR}
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
			LDR     R0,=SIM_SOPT2
			LDR     R1,=SIM_SOPT2_UART0SRC_MASK
			LDR     R2,[R0,#0]
			BICS    R2,R2,R1
			LDR     R1,=SIM_SOPT2_UART0_MCGPLLCLK_DIV2
			ORRS    R2,R2,R1
			STR     R2,[R0,#0]
		;Enable external connection for UART0
			LDR     R0,=SIM_SOPT5
			LDR     R1,=SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
			LDR     R2,[R0,#0]
			BICS    R2,R2,R1
			STR     R2,[R0,#0]
		;Enable clock for UART0 module 
			LDR     R0,=SIM_SCGC4
			LDR     R1,=SIM_SCGC4_UART0_MASK
			LDR     R2,[R0,#0]
			ORRS    R2,R2,R1
			STR     R2,[R0,#0]
		;Enable clock for Port A module
			LDR     R0,=SIM_SCGC5
			LDR     R1,=SIM_SCGC5_PORTA_MASK
			LDR     R2,[R0,#0]
			ORRS    R2,R2,R1
			STR     R2,[R0,#0]
		;Connect PORT A Pin 1 (PTA1) to UART0 Rx (J1 Pin 02)
			LDR     R0,=PORTA_PCR1
			LDR     R1,=PORT_PCR_SET_PTA1_UART0_RX
			STR     R1,[R0,#0]
		;Connect PORT A Pin 2 (PTA2) to UART0 Tx (J1 Pin 04)
			LDR     R0,=PORTA_PCR2
			LDR     R1,=PORT_PCR_SET_PTA2_UART0_TX
			STR     R1,[R0,#0] 
		;Disable UART0 receiver and transmitter
			LDR		R0,=UART0_BASE
			MOVS 	R1,#UART0_C2_T_R
			LDRB 	R2,[R0,#UART0_C2_OFFSET]
			BICS 	R2,R2,R1
			STRB 	R2,[R0,#UART0_C2_OFFSET]
	   ;set UART0 IRQ priority
			LDR		R0,=UART0_IPR
			;LDR	R1,=NVIC_IPR_UART0_MASK
			LDR		R2,=NVIC_IPR_UART0_PRI_3
			LDR		R3,[R0,#0]
			;BICS	R3,R3,R1
			ORRS	R3,R3,R2
			STR		R3,[R0,#0]
	   ;clear any pending UART0 interrupts
			LDR		R0,=NVIC_ICPR
			LDR		R1,=NVIC_ICPR_UART0_MASK
			STR		R1,[R0,#0]
	   ;Unmask UART0 interrupts
			LDR 	R0,=NVIC_ISER
			LDR 	R1,=NVIC_ISER_UART0_MASK
			STR 	R1,[R0,#0]
	   ;Set UART0 for 9600 baud, 8N1 protocol
            LDR     R0,=UART0_BASE
			MOVS 	R1,#UART0_BDH_9600
			STRB 	R1,[R0,#UART0_BDH_OFFSET]
			MOVS 	R1,#UART0_BDL_9600
			STRB 	R1,[R0,#UART0_BDL_OFFSET]
			MOVS 	R1,#UART0_C1_8N1
			STRB 	R1,[R0,#UART0_C1_OFFSET]
			MOVS 	R1,#UART0_C3_NO_TXINV
			STRB 	R1,[R0,#UART0_C3_OFFSET]
			MOVS 	R1,#UART0_C4_NO_MATCH_OSR_16
			STRB 	R1,[R0,#UART0_C4_OFFSET]
			MOVS 	R1,#UART0_C5_NO_DMA_SSR_SYNC
			STRB 	R1,[R0,#UART0_C5_OFFSET]
			MOVS 	R1,#UART0_S1_CLEAR_FLAGS
			STRB 	R1,[R0,#UART0_S1_OFFSET]
			MOVS 	R1,#UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
			STRB 	R1,[R0,#UART0_S2_OFFSET]
		;Enable UART0 receiver and transmitter
			MOVS 	R1,#UART0_C2_T_RI
			STRB 	R1,[R0,#UART0_C2_OFFSET]
	       	 ;Pop prevous R0-2 values off the stack.
			 POP {R0, R1, R2, R3, PC}

;>>>>>   end subroutine code <<<<<
            ALIGN
;****************************************************************
;Vector Table Mapped to Address 0 at Reset
;Linker requires __Vectors to be exported
            AREA    RESET, DATA, READONLY
            IMPORT  Reset_Handler
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
            DCD    Dummy_Handler	  ;38:PIT (all IRQ sources)
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
APSRState           SPACE 2
            ALIGN
InputString		SPACE MAX_STRING
            ALIGN
;>>>>>   end variables here <<<<<
            ALIGN
            END