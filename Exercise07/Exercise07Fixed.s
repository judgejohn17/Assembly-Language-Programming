;Exercise07 Circular Fifo Queue Operations
;****************************************************************
;Name:  John Judge
;Date:  3/31/16
;Class:  CMPE-250
;Section:  Thursdays 2:00Pm-3:50PM

;---------------------------------------------------------------
;Keil Template for KL46
;R. W. Melton
;ApR0l 3, 2015
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
LETTERS EQU 21
MAX_STRING EQU 79   ;MaxStringCharacters(including null termination)
;Exercise06 Secure String I/0 and Number Output 

;Managment record structure field displacements 
IN_PTR		EQU		0
OUT_PTR		EQU		4
BUF_START	EQU		8
BUF_PAST	EQU		12
BUF_SIZE	EQU		16
NUM_ENQD	EQU		17
	
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port A
PORT_PCR_SET_PTA1_UART0_RX EQU (PORT_PCR_ISF_MASK :OR: PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTA2_UART0_TX EQU (PORT_PCR_ISF_MASK :OR: PORT_PCR_MUX_SELECT_2_MASK)
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
; (PLLFLLSEL determines MCGFLLCLK' or MCGPLLCLK/2)
; 1= 16:PLLFLLSEL=PLL/FLL clock select (MCGPLLCLK/2)
SIM_SOPT2_UART0SRC_MCGPLLCLK EQU (1 << SIM_SOPT2_UART0SRC_SHIFT)
SIM_SOPT2_UART0_MCGPLLCLK_DIV2 EQU (SIM_SOPT2_UART0SRC_MCGPLLCLK :OR: SIM_SOPT2_PLLFLLSEL_MASK)
;---------------------------------------------------------------
;SIM_SOPT5
; 0-> 16:UART0 open drain enable (disabled)
; 0-> 02:UART0 receive data select (UART0_RX)
;00->01-00:UART0 transmit data select source (UART0_TX)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR EQU (SIM_SOPT5_UART0ODE_MASK :OR: SIM_SOPT5_UART0RXSRC_MASK :OR: SIM_SOPT5_UART0TXSRC_MASK)
    ;---------------------------------------------------------------
;UART0_BDH
; 0-> 7:LIN break detect IE (disabled)
; 0-> 6:RxD input active edge IE (disabled)
; 0-> 5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (BUSCLK / [9600 * (OSR + 1)])
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;BUSCLK is 24 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDH_9600 EQU 0x01
;---------------------------------------------------------------
;UART0_BDL
;26->7-0:SBR[7:0] (BUSCLK / [9600 * (OSR + 1)])
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;BUSCLK is 24 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDL_9600 EQU 0x38
;---------------------------------------------------------------
;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select
; (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=parity enable (disabled)
;0-->0:PT=parity type (even parity--no effect PE=0)
UART0_C1_8N1 EQU 0x00
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
UART0_C2_T_R EQU (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK) 
;---------------------------------------------------------------
;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
; 10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
; 10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
; (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:ORIE=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=parity error IE for PF (disabled)
UART0_C3_NO_TXINV EQU 0x00
;---------------------------------------------------------------
;UART0_C4
; 0--> 7:MAEN1=match address mode enable 1 (disabled)
; 0--> 6:MAEN2=match address mode enable 2 (disabled)
; 0--> 5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
; = 1 + OSR for 3 <= OSR <= 31
; = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16 EQU 0x0F
UART0_C4_NO_MATCH_OSR_16 EQU UART0_C4_OSR_16 
    ;---------------------------------------------------------------
;UART0_C5
; 0--> 7:TDMAE=transmitter DMA enable (disabled)
; 0--> 6:Reserved; read-only; always 0
; 0--> 5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
; 0--> 1:BOTHEDGE=both edge sampling (rising edge only)
; 0--> 0:RESYNCDIS=resynchronization disable (enabled)
UART0_C5_NO_DMA_SSR_SYNC EQU 0x00
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
UART0_S1_CLEAR_FLAGS EQU 0x1F
;---------------------------------------------------------------
;UART0_S2
;1-->7:LBKDIF=LIN break detect interrupt flag (clear)
; write 1 to clear
;1-->6:RXEDGIF=RxD pin active edge interrupt flag (clear)
; write 1 to clear
;0-->5:(reserved); read-only; always 0
;0-->4:RXINV=receive data inversion (disabled)
;0-->3:RWUID=receive wake-up idle detect
;0-->2:BRK13=break character generation length (10)
;0-->1:LBKDE=LIN break detect enable (disabled)
;0-->0:RAF=receiver active flag; read-only
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS EQU 0xC0
;--------------------------------------------------------------- 
;---------------------------------------------------------------
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
        BL      Init_UART0_Polling
        LDR     R0,=QBUFFER     ;Initialize starting point of queue
        LDR		R1,=QRECORD		;Initialize queue record
		MOVS	R2,#4			;Initialize queue length, this will not change
        BL    	InitQueue       ;Initalize queu record structure
MAIN
        LDR     R0,=MainString  ;MainString in R0 
        BL      PutStringSB     ;Print MainString
INPUTLOOP                       ;Loop for reading input
        BL      GETCHAR         ;read input char put in R0
        MOVS    R4,R0           ;Char goes to R4 for copy
        CMP     R0,#94          ;If Char is lowercase
        BGT     Skip
        ADDS    R0,R0,#32       ;Add decimal 32, turns it to lowercase
Skip         
        CMP     R0,#100         ;If d
        BEQ     DOSTUFFd        ;
        CMP     R0,#101         ;If e
        BEQ     DOSTUFFe
        CMP     R0,#104         ;If h
        BEQ     DOSTUFFh
        CMP     R0,#112         ;If p
        BEQ     DOSTUFFp
		CMP     R0,#115         ;If s
        BEQ     DOSTUFFs        
 
        B       INPUTLOOP
      
        

        B       .
		
		LTORG
;>>>>>   end main program code <<<<<
;---------------------------------------------------------------
;>>>>> begin subroutine code <<<<<
;This subroutine preforms deques when d is entered in the terminal
;Input parameters:
;   R0, lower d
;Output parameters:
;   None
;Modified Registers
;   
DOSTUFFd
        MOVS    R0,R4
        BL      PUTCHAR             ;Echo character
        BL      NEXTLINE
		
		BL		Dequeue				;Attempt to dequeue character
DidDequeueWork
		BCC		DequeueWorked
		BCS		DequeueDidNotWork
DequeueWorked
		LDRB	R0,[R0,#0]			;get char to print
		BL		PUTCHAR				;Print dequeued character
        MOVS    R0,#":"              ;SemiColon
        BL      PUTCHAR             ;Print SemiColon
        BL		ContinueDOSTUFFd
DequeueDidNotWork
		LDR		R0,=FailureString
		BL		PutStringSB
		BL		ContinueDOSTUFFd
ContinueDOSTUFFd
		LDR    R0,=TabString       ;need tab between success/failure and status
        BL      PutStringSB
        BL      PrintStatus
        BL      NEXTLINE            ;NextLine
        B       MAIN                ;Back to begining of main
        
		
;This subroutine enques a character when e is entered 
;Input parameters:
;   R0=e
;Output parameters:
;   None
;Modified Registers
;   
DOSTUFFe
        MOVS    R0,R4
        BL      PUTCHAR             ;Echo
        BL		NEXTLINE
		LDR		R0,=eString
		BL		PutStringSB			;Print string 
		BL		GETCHAR
		BL		PUTCHAR				;echo chartacter
		LDR		R1,=QRECORD			;Initialize queue record
		BL		Enqueue
		BL		NEXTLINE
		BCC		EnqueueWorked
		BCS		EnqueueDidNotWork
EnqueueWorked
		LDR		R0,=SuccessString
		BL		PutStringSB
		BL		ContinueDOSTUFFe
EnqueueDidNotWork
		LDR		R0,=FailureString
		BL		PutStringSB
		BL		ContinueDOSTUFFe
ContinueDOSTUFFe
		LDR    R0,=TabString       ;need tab between success/failure and status
        BL      PutStringSB
        BL      PrintStatus
        BL      NEXTLINE            ;NextLine
        B       MAIN                ;Back to begining of main
               
;This subroutine displays the help menu when e is entered in the terminal
;Input parameters:
;   R0 = h
;Output parameters:
;   None
;Modified Registers
;   
DOSTUFFh
        MOVS    R0,R4
        BL      PUTCHAR           ;Echo
        BL      NEXTLINE
        LDR     R0,=HelpString    ;help string to print
        BL      PutStringSB 
        BL      NEXTLINE            
        B       MAIN
        
;This subroutine displays the status of the queue when s is entered in the terminal
;Input parameters:
;   R0 = s
;Output parameters:
;   None
;Modified Registers
;   
DOSTUFFs
        MOVS    R0,R4
        BL      PUTCHAR                 ;Echo
        BL      NEXTLINE
        LDR     R0,=StatusString        ;"status:"
        BL      PutStringSB 
        BL      PrintStatus
        BL      NEXTLINE            
        B       MAIN		
;This subroutine prints the queue when p is entered in the terminal
;Input parameters:
;   R0 = p
;Output parameters:
;   None
;Modified Registers
;   
DOSTUFFp
        MOVS        R0,R4
        BL          PUTCHAR            ;Print Character to Console
        BL          NEXTLINE           ;NextLine
        MOVS        R0,#">"             ;">" 
        BL          PUTCHAR            ;Print ">"
        LDR			R1,=QRECORD			;Initialize queue record
		LDRB		R2,[R1,#NUM_ENQD]	;get number enqueued
		MOVS		R5,#0
		CMP			R2,R5				;Compare num enqued and 0
		BLO			EndPrint 			;Branch if num enqd =< 0
		LDR			R3,[R1,#OUT_PTR]	;get adress of outpointer
        LDR			R4,[R1,#IN_PTR]		;Load R4 with inpt
LoopieCond
		
		CMP			R2,R5				;Compare Num enqd and 0
		BEQ			EndPrint			;If equal you have reached the end
		
Loopie
		LDRB		R4,[R3,#0]			;Store character in R4
		MOVS		R0,R4
		BL			PUTCHAR				;Print character
		ADDS		R3,R3,#1			;Increment outpointer*
        SUBS        R2,R2,#1            ;Sub NumEnqd*
		LDR			R6,[R1,#BUF_PAST]	;get buffer past
		CMP			R3,R6				;Compare Outpointer* to BufferPast
		BGT			NeedToWrapToBegining
		B			LoopieCond
NeedToWrapToBegining
		LDR			R3,[R1,#BUF_START]	;Move outpointer* to buff start
		B			LoopieCond 			;Loop

EndPrint
        MOVS        R0,#"<"             ;"<" 
        BL          PUTCHAR            ;Print "<"
        BL          NEXTLINE           ;NextLine
        B           MAIN               ;Back to begining of main



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
        PUSH    {LR}
		STR		R0,[R1,#IN_PTR]		;Initialize In pointer to start of queue
		STR		R0,[R1,#OUT_PTR]	;Initialize Out pointer to start of queue
		STR		R0,[R1,#BUF_START]	;Initialize buffer start to start of queue
		STR		R2,[R1,#BUF_SIZE]	;Initialize buffer size to 4
		MOVS	R3,#0
		STRB		R3,[R1,#NUM_ENQD]	;Initialize num enqd to 0
		ADDS	R0,R0,R2			;End of queue
		STR		R0,[R1,#BUF_PAST]	;Initialize buffer past
		POP     {PC}
	
;This subroutine enqueues a character
;Input paramters:
;	r0: Character to enque 
;	R1: Address of queue record structure
;Output:
;	PSR C flag: success(0) or failure (1)
;Modify: PSR
Enqueue
		PUSH	{R0-R4,LR}
		MOVS	R6,R0
        LDR		R1,=QRECORD			;Initialize queue record
		LDR     R0,=QBUFFER     ;Initialize starting point of queue
		LDRB	R2,[R1,#NUM_ENQD]	;get number enqued
		MOVS		R3,#4
		CMP		R2,R3				;Compare num enqd to buff size
		BGE		Queuefull			;Branch if num enqd >= buff size
		LDR		R4,[R1,#IN_PTR]		;get inpointer address
		STRB	R6,[R0,R2]		;Store character at In pointer
		ADDS	R4,R4,#1			;Increment inpointer address
		STRB		R4,[R1,#IN_PTR]		;Store incremented inpointer value
		ADDS	R2,R2,#1			;Increment Numenqd
		STRB		R2,[R1,#NUM_ENQD]		;Store incremented enqd value
		LDR		R2,[R1,#BUF_PAST]	;get buff past value
		CMP		R4,R2				;Compare inpointer to outpionter
		BEQ		NeedToWrap			;If Inptr < Buff past
		MRS		R0,APSR				;Clear carry to show success (Lec notes M,2/8 slide #5) 
		MOVS	R1,#0x20
		LSLS	R1,R1,#24
		BICS	R0,R0,R1
		MSR		APSR,R0
		B		EndEnqueue			;End 
Queuefull
		MRS		R0,APSR				;Show unsuccesfull enque by setting carry flag
		MOVS	R1,#0x20
		LSLS	R1,R1,#24
		ORRS	R0,R0,R1
		MSR		APSR, R0
		B		EndEnqueue			;End
NeedToWrap
		
		LDR		R2,[R1,#BUF_START]	;get buffer start
		STR		R2,[R1,#IN_PTR]		;adjust inpointer to start of queue
		MRS		R0,APSR				;Clear carry to show success (Lec notes M,2/8 slide #5) 
		MOVS	R1,#0x20
		LSLS	R1,R1,#24
		BICS	R0,R0,R1
		MSR		APSR,R0
		B		EndEnqueue			;End
EndEnqueue
		POP		{R0-R4,PC}
		BX		LR

;This subroutine dequeues a character
;Input paramters:
;	R1: Address of queue record structure
;Output:
;	PSR C flag: success(0) or failure (1)
;	r0: Character to dequeued 
;Modify: PSR
Dequeue
		PUSH	{R0-R7}
		LDR     R0,=QBUFFER     ;Initialize starting point of queue
        LDR		R1,=QRECORD		;Initialize queue record
		LDRB		R2,[R1,#NUM_ENQD]	;get number enqueued
		MOVS	R3,#0
		CMP		R2,R3				;Compare num enqued and 0
		BLS		QueueEmpty			;Branch if num enqd =< 0
		LDR 	R4,[R1,#OUT_PTR]	;get adress of outpointer
		ADDS	R4,R4,#1			;Increment out pointer
		STR 	R4,[R1,#OUT_PTR]	;Store new adredd of outpointer
		SUBS	R2,R2,#1			;decrement number enqued
		STRB	R2,[R1,#NUM_ENQD]	;store number enqued
		LDR		R2,[R1,#BUF_PAST]	;get buffer past
		SUBS	R7,R4,#1			;get address of dequeued character
		CMP		R4,R2				;Compare Outpointer to BufferStart
		BGE		NeedToWrapAgain
		MRS		R0,APSR				;Clear carry to show success (Lec notes M,2/8 slide #5) 
		MOVS	R1,#0x20
		LSLS	R1,R1,#24
		BICS	R0,R0,R1
		MSR		APSR,R0
		B		EndDequeue			;End
NeedToWrapAgain
		LDR     R0,=QBUFFER     ;Initialize starting point of queue
		STR		R0,[R1,#OUT_PTR]	;Move outpointer to buff start
		MRS		R0,APSR				;Clear carry to show success (Lec notes M,2/8 slide #5) 
		MOVS	R1,#0x20
		LSLS	R1,R1,#24
		BICS	R0,R0,R1
		MSR		APSR,R0
		B		EndDequeue			;End
QueueEmpty
		MRS		R0,APSR				;Show unsuccesfull enque by setting carry flag
		MOVS	R1,#0x20
		LSLS	R1,R1,#24
		ORRS	R0,R0,R1
		MSR		APSR, R0
		B		EndDequeue			;End
EndDequeue
		
		MOVS	R0,R7				;Move address of deued dequeued character to R0
		POP		{R1-R7}
		B		DidDequeueWork
		
		

;This subroutine displays the status of the queue when called
;Input parameters:
;   None
;Output parameters:
;   None
;Modified Registers
;   None
PrintStatus
        PUSH    {R0-R7,LR}
        LDR	    R0,=InString            ;"In="
        BL      PutStringSB
        LDR    R1,=QRECORD			    ;Initialize queue record
        LDR     R0,[R1,#IN_PTR]         ;get inpointer value
        BL      PutNumHex
        LDR    R0,=TabString2          ;"  "
        BL      PutStringSB
        LDR    R0,=OutString           ;"Out="
        BL      PutStringSB
        LDR     R0,[R1,#OUT_PTR]        ;get outpointer value
        BL      PutNumHex
        LDR    R0,=TabString2          ;"  "
        BL      PutStringSB
        LDR    R0,=NumString           ;"Num="
        BL      PutStringSB
        LDRB     R0,[R1,#NUM_ENQD]       ;get num enqd value
		ADDS	R0,R0,#48				;get ascii equivalent
        BL      PUTCHAR
        POP     {R0-R7,PC}
        BX      LR
;The follwing subroutine intiializes the board by initializing 
;the clock, the pins, and the USB connection.
;Input parameters:
;   None, Registers R0-R3 are pushed to begin as to not disturb them
;Output parameters:
;   None, Registers R0-R3 are popped to end as to return their original values
;Modified Registers
;   None, Registers R0-R3 are modified but their origianl values are pushed to a 
;    stack and popped following the subroutines execution.
Init_UART0_Polling
        PUSH	{R0,R1,R2,R3}
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
        LDR     R1,= SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
        LDR     R2,[R0,#0]
        BICS    R2,R2,R1
        STR     R2,[R0,#0]
    ;Enable clock for UART0 module 
        LDR     R0,=SIM_SCGC4
        LDR     R1,= SIM_SCGC4_UART0_MASK
        LDR     R2,[R0,#0]
        ORRS    R2,R2,R1
        STR     R2,[R0,#0]
    ;Enable clock for Port A module
        LDR     R0,=SIM_SCGC5
        LDR     R1,= SIM_SCGC5_PORTA_MASK
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
        LDR     R0,=UART0_BASE
        MOVS    R1,#UART0_C2_T_R
        LDRB    R2,[R0,#UART0_C2_OFFSET]
        BICS    R2,R2,R1
        STRB    R2,[R0,#UART0_C2_OFFSET]
    ;Set UART0 for 9600 baud, 8N1 protocol
        MOVS    R1,#UART0_BDH_9600
        STRB    R1,[R0,#UART0_BDH_OFFSET]
        MOVS    R1,#UART0_BDL_9600
        STRB    R1,[R0,#UART0_BDL_OFFSET]
        MOVS    R1,#UART0_C1_8N1
        STRB    R1,[R0,#UART0_C1_OFFSET]
        MOVS    R1,#UART0_C3_NO_TXINV
        STRB    R1,[R0,#UART0_C3_OFFSET]
        MOVS    R1,#UART0_C4_NO_MATCH_OSR_16
        STRB    R1,[R0,#UART0_C4_OFFSET]
        MOVS    R1,#UART0_C5_NO_DMA_SSR_SYNC
        STRB    R1,[R0,#UART0_C5_OFFSET]
        MOVS    R1,#UART0_S1_CLEAR_FLAGS
        STRB    R1,[R0,#UART0_S1_OFFSET]
        MOVS    R1, #UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
        STRB    R1,[R0,#UART0_S2_OFFSET]
    ;Enable UART0 receiver and transmitter
        MOVS    R1,#UART0_C2_T_R
        STRB    R1,[R0,#UART0_C2_OFFSET] 
   ;Poll TDRE until UART0 ready to transmit
        LDR     R1,=UART0_BASE
        MOVS    R2,#UART0_S1_TDRE_MASK
        POP     {R0,R1,R2,R3}
        BX      LR
    
		
;The follwing subroutine recieves a character from the console
;
;Input parameters:
;   None, Registers R1-R3 are pushed to begin as to not disturb them 
;Output parameters:
;   R1 contains the address of UART0_BASE
;   R2 contains the immediate of UART0_S1_TDRE_MASK
;Modified Registers
;   R1 contains the address of UART0_BASE
;   R2 contains the immediate of UART0_S1_TDRE_MASK  
PUTCHAR
	;Poll REDRF until UART0 ready to recieve
        PUSH	{R1,R2,R3,LR}
		LDR		R1,=UART0_BASE
		MOVS	R2, #UART0_S1_TDRE_MASK

;The follwing subroutine works in conjunction with GetChar
;It stores the character in register R0
;Input parameters:
;   R1 contains the address of the char
;Output parameters:
;   R0 conatins the byte from the console
;Modified Registers
;   R3 is modefied but returns its original value 
;   R0 conatins the byte from the console
;       
POLLTX	
        LDRB	R3,[R1, #UART0_S1_OFFSET]
		ANDS	R3,R3,R2
		BEQ		POLLTX
	;Recieve character and store in R0
		STRB	R0,[R1,#UART0_D_OFFSET]
        POP		{R1,R2,R3,PC}


;The follwing subroutine gets the address of the char that was modefied.
;Its value is stored in R2
;Input parameters:
;   None, Registers R1-R3 are pushed to begin as to not disturb them 
;Output parameters:
;   R1 conatins the adress of UART0_BASE
;   R2 contains the adress of the deciphered char
;Modified Registers
;   R1 conatins the adress of UART0_BASE
;   R2 contains the adress of the deciphered char
;       		
GETCHAR
	;Poll TDRE until UART0 ready to transmit
		PUSH	{R1,R2,R3,LR}
		LDR		R1,=UART0_BASE
		MOVS	R2,#UART0_S1_RDRF_MASK

;The follwing subroutine works in conjunction with Get Char to 
;store the deciphered char in R0 and display on the console
;Input parameters:
;   ;R1 contains the address of the char 
;Output parameters:
;   R0 contains the deciphered char
;Modified Registers
;   R3 conatins the adress of the char, offset by a constant value but is 
;       original value is popped at the end
;   R0 conatins the deciphered char 
;               		
POLLRX	
		LDRB	R3,[R1,#UART0_S1_OFFSET]
		ANDS	R3,R3,R2
		BEQ		POLLRX
	;Recieve character and store in R0
		LDRB 	R0,[R1,#UART0_D_OFFSET]
		POP		{R1,R2,R3,PC}

		
;Prints the text in hexadecimal of unsigned word value R0 in terminal screen
;Input parameters:
;   R0 is an Unsigned Word Value to print
;Output parameters:
;   None
;Modified Registers
;   PSR: (after return, nothing else)
PutNumHex   
        PUSH    {LR,R1-R7}
PNH		MOVS	R1,R0				;Copies R0 to R1
		MOVS	R2,R0				;Copies R0 to R2
        LSRS    R1,R1,#24           ;Shifts MSB of hex down
        LSRS    R1,R1,#4            ;shifts the higher nibble to the lsb
        CMP     R1,#9               ;Compares the number to 9
        BLE     PNHnum              ;If digit is between 0 and 9 branch 
        ADDS    R1,R1,#55           ;convert to ascii
        MOVS    R0,R1
        BL      PUTCHAR
        B       PHN1
PNHnum  ADDS    R1,R1,#48           ;if digit s between 0 and 9 add 48
        MOVS    R0,R1               
        BL      PUTCHAR
        B       PHN1                ;Branch to print the next number
PHN1    MOVS    R1,R2               
        LSRS    R1,R1,#24           ;shifts msb of the hex value down
        MOVS    R3,#0xF0            ;moves 0xF0 to R2 to mask other numbers
        BICS    R1,R1,R3            ;clears upper buits
        CMP     R1,#9               ;if digit is between 0 and 9 then branch to PNH1num
        BLE     PNH1num
        ADDS    R1,R1,#55           ;get ascii equivalent
        MOVS    R0,R1
        BL      PUTCHAR
        B       PNH2
PNH1num ADDS    R1,R1,#48           ;get ascii value
        MOVS    R0,R1
        BL      PUTCHAR
        B       PNH2
PNH2    MOVS    R1,R2
        LSLS    R1,R1,#8            ;
        LSRS    R1,R1,#8
        LSRS    R1,R1,#16
        LSRS    R1,R1,#4
        CMP     R1,#9
        BLE     PNH2num
        ADDS    R1,R1,#55
        MOVS    R0,R1
        BL      PUTCHAR
        B       PHN3
PNH2num ADDS    R1,R1,#48
        MOVS    R0,R1
        BL      PUTCHAR
        B       PHN3
PHN3    MOVS    R1,R2
        LSLS    R1,R1,#8
        LSRS    R1,R1,#8
        LSRS    R1,R1,#16
        MOVS    R3,#0XF0
        BICS    R1,R1,R3
        CMP     R1,#9
        BLE     PNH3num
        ADDS    R1,R1,#55
        MOVS    R0,R1
        BL      PUTCHAR
        B       PNH4
PNH3num ADDS    R1,R1,#48
        MOVS    R0,R1
        BL      PUTCHAR
        B       PNH4
PNH4    MOVS    R1,R2
        LSLS    R1,R1,#16
        LSRS    R1,R1,#16
        LSRS    R1,R1,#8
        LSRS    R1,R1,#4
        CMP     R1,#9
        BLE     PNH4num
        ADDS    R1,R1,#55
        MOVS    R0,R1
        BL      PUTCHAR
        B       PHN5
PNH4num ADDS    R1,R1,#48
        MOVS    R0,R1
        BL      PUTCHAR
        B       PHN5
PHN5    MOVS    R1,R2
        LSLS    R1,R1,#16
        LSRS    R1,R1,#16
        LSRS    R1,R1,#8
        MOVS    R3,#0xF0
        BICS    R1,R1,R3
        CMP     R1,#9
        BLE     PNH5num
        ADDS    R1,R1,#55
        MOVS    R0,R1
        BL      PUTCHAR
        B       PNH6
PNH5num ADDS    R1,R1,#48
        MOVS    R0,R1
        BL      PUTCHAR
        B       PNH6
PNH6    MOVS    R1,R2
        LSLS    R1,R1,#24
        LSRS    R1,R1,#24
        LSRS    R1,R1,#4
        CMP     R1,#9
        BLE     PNH6num
        ADDS    R1,R1,#55
        MOVS    R0,R1
        BL      PUTCHAR
        B       PHN7
PNH6num ADDS    R1,R1,#48
        MOVS    R0,R1
        BL      PUTCHAR
        B       PHN7
PHN7    MOVS    R1,R2
        LSLS    R1,R1,#24
        LSRS    R1,R1,#24
        MOVS    R3,#0xF0
        BICS    R1,R1,R3
        CMP     R1,#9
        BLE     PNH7num
        ADDS    R1,R1,#55
        MOVS    R0,R1
        BL      PUTCHAR
        B       PNHEnd
PNH7num ADDS    R1,R1,#48
        MOVS    R0,R1
        BL      PUTCHAR
        B       PNHEnd
PNHEnd  POP     {PC,R1-R7}
        BX      LR

;Displays a null terminated string to the terminal screen from memory starting at R0 
;Preventing overun of the buffer capacity specified by R1
;Leaves cursor specified in R1

PutStringSB
		PUSH	{R1,R2,R3,R4,LR}				;Push registers as to not overwrite original values
        MOVS    R3,#0
        MOVS    R2,R0               ;Address of string to R2
        MOVS    R4,#0
LoopBegin      
        LDRB    R0,[R2,R3]          ;Load Char to R0
		CMP 	R4,R1               ;Compare Max_string to 0
		BEQ     EndThis             ;End if 0
		CMP     R0,#00            ;Compare Char to Null
        BEQ     EndThis             ;End if null
		BL      PUTCHAR             ;DisplayChar
        ADDS    R4,R4,#1            ;Decrement Max_String
        ADDS    R3,R3,#1            ;Increment Memory
        B       LoopBegin
        
EndThis
        BL      PUTCHAR             ;DisplayCharacter
		POP		{R1,R2,R3,R4,PC}	    ;Restore original values


		

;This subroutine moves the terminal to the next line
;Input parameters:
;   None
;Output parameters:
;   None
;Modified Registers
;   R0 is modefied to the carriqage return and the line feed characters but its
;   original value is returned
NEXTLINE
        PUSH    {R0,LR}                ;Dont fuck with R0
        MOVS    R0,#0x0D            ;Carriage Return
        BL      PUTCHAR
        MOVS    R0,#0x0A            ;Line feed
        BL      PUTCHAR
        POP     {R0,PC}                ;Restore R0
 
       
		
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
                                      ;AR3 core vectors
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
            DCD    Dummy_Handler      ;28:UART0 (status; error)
            DCD    Dummy_Handler      ;29:UART1 (status; error)
            DCD    Dummy_Handler      ;30:UART2 (status; error)
            DCD    Dummy_Handler      ;31:ADC0
            DCD    Dummy_Handler      ;32:CMP0
            DCD    Dummy_Handler      ;33:TPM0
            DCD    Dummy_Handler      ;34:TPM1
            DCD    Dummy_Handler      ;35:TPM2
            DCD    Dummy_Handler      ;36:RTC (alaR3)
            DCD    Dummy_Handler      ;37:RTC (seconds)
            DCD    Dummy_Handler      ;38:PIT (all IRQ sources)
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
MainString  		DCB 	"Type a string command(d,e,h,p,s):", 0
FailureString    	DCB 	"Failure:", 0
InString    		DCB 	"In=0x", 0
OutString    		DCB 	"Out=0x", 0
NumString			DCB		"Num=", 0
SuccessString		DCB		"Success:", 0
TabString		    DCB		"       ", 0
TabString2          DCB     "   ", 0
StatusString        DCB     "Status:    ", 0
eString				DCB		"Character to enqueue:", 0
HelpString			DCB		"d (dequeue), e (enqueue), h (help), p (print), s (status)", 0
;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;VaR0ables
            AREA    MyData,DATA,READWRITE
;>>>>> begin vaR0ables here <<<<<
;Queue structure sizes 
QBUFFER		SPACE		4		;Room for 80 characters
QRECORD		SPACE		17		;Management record

;>>>>>   end vaR0ables here <<<<<
            ALIGN
            END


