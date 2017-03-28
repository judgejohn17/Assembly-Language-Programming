
;The following program produces secure subroutiines for the serial
;I/0 of strings and a specified program to write them

;Name:  John Judge
;Date:  4/1/16-4/4/16
;Class:  CMPE-250
;Section:  Thursdays 2:00Pm-3:50PM
;Fix PutNumU and write main. Find way to move cursor
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
;Equates
MAX_STRING EQU 79   ;MaxStringCharacters(including null termination)	
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
		LDR		R0,=InitialString
		LDR		R1,=StartingPoint
		BL		CopyString
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
        CMP     R0,#103         ;If g
        BEQ     DOSTUFFg        ;
        CMP     R0,#104         ;If h
        BEQ     DOSTUFFh
        CMP     R0,#109         ;If m
        BEQ     DOSTUFFm
        CMP     R0,#112         ;If p
        BEQ     DOSTUFFp
		CMP     R0,#114         ;If r
        BEQ     DOSTUFFr
		B		NotValid
        B       .
;>>>>>   end main program code <<<<<
;---------------------------------------------------------------
;This subroutine gets gets called when an not valid character is entered
;in the terminal 
;Input parameters:
;   R0 = char
;Output parameters:
;   None
;Modified Registers
;   None
NotValid
		MOVS    R0,R4
        BL      PUTCHAR             ;Echo character
        LDR     R0,=InvalidString   ;string for invalid command
        BL      PutStringSB
		BL      NEXTLINE            ;NextLine
		B		MAIN				;Return to main


;This subroutine gets the operational string from the console when g
;is entered in the terminal
;Input parameters:
;   R0 = g
;Output parameters:
;   None
;Modified Registers
;   
DOSTUFFg    
        MOVS    R0,R4
        BL      PUTCHAR             ;Echo character
        BL      NEXTLINE 
        LDR     R0,=StartingPoint   ;Initialize starting point of string 
        LDR     R1,=MAX_STRING      ;Initialize Max string parameter
        BL      GetStringSB         ;GetString
        BL      NEXTLINE            ;NextLine
        B       MAIN                ;Back to begining of main
;This subroutine modefies the string when m is entered in the terminal
;Input parameters:
;   R0 = m
;Output parameters:
;   None
;Modified Registers
;   
DOSTUFFm    
        MOVS    R0,R4             	;Move h to R0 for put char
        BL      PUTCHAR           	;Echo
        BL      NEXTLINE
        LDR     R0,=StartingPoint 	;Load Statring point for dtring
		BL		ModifyString
		LDR     R0,=StartingPoint  	;Initialize starting point of string
        LDR     R1,=MAX_STRING     	;Initialize Max string parameter
        BL      PutStringSB        	;GetString
        BL      NEXTLINE            ;NextLine
        B       MAIN                ;Back to begining of main
;This subroutine preforms PUTSTRINGSB when p is entered in the terminal
;Input parameters:
;   R1 = p
;Output parameters:
;   None
;Modified Registers
;   
DOSTUFFp
        MOVS        R0,R4
        BL          PUTCHAR            ;Print Character to Console
        BL          NEXTLINE           ;NextLine
        LDR         R0,=StartingPoint  ;Initialize starting point of string
        LDR         R1,=MAX_STRING     ;Initialize Max string parameter
        BL          PutStringSB        ;GetString
        BL          NEXTLINE           ;NextLine
        B           MAIN               ;Back to begining of main
;This subroutine reverses the string when r is entered in the terminal
;Input parameters:
;   R0 = r
;Output parameters:
;   None
;Modified Registers
;   
DOSTUFFr    
        MOVS    R0,R4             	;Move h to R0 for put char
        BL      PUTCHAR           	;Echo
        BL      NEXTLINE
        LDR     R0,=StartingPoint 	;Load Statring point for dtring
		BL		ReverseString
		LDR     R0,=StartingPoint  	;Initialize starting point of string
        LDR     R1,=MAX_STRING     	;Initialize Max string parameter
        BL      PutStringSB        	;GetString
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
;This subroutine creates a null terminated string in memory starting at
;the address in R1 by copying the character from a null terminated source
;string starting at the address in R0
;Input Parameters
;	R0 starting address of source string
;	R1 starting address of null terminated string to copy into
;Output
;	None
;Modefied Registers
;	PSR
CopyString
				PUSH	{R0-R7,LR}
				MOVS	R2,#0			;Intialize counter/offset
				MOVS	R4,#0x00		;store null
CSCond			
				LDRB	R3,[R0,R2]		;get char from string
				CMP		R3,R4			;compare to null
				BEQ		CSEnd			
CSLoop		
				STRB	R3,[R1,R2]		;store char in new string
				ADDS	R2,R2,#1		;Increment counter/offset
				B		CSCond
CSEnd			
				STRB	R4,[R1,R2]		;Store null at the end
				POP		{R0-R7,PC}
				BX		LR

;This subroutine modefies the null-terminated string in memory staring at the
;address in R0 to replace all space characters as underscores, all upper-case 
;characters with their lower case equivalents, and every number with a pound sign 
;Input Parameters
;	R0 starting address in memory of nulll terminated string
;Output Paramters
;	None
;Modefied Registers
;	PSR
ModifyString
				PUSH	{R0-R7,LR}
				LDRB	R1,[R0,#0]		;Get character at begining of string, store in R1
MSCond			
				LDRB	R1,[R0,#0]		;Get character at begining of string, store in R1
				MOVS	R3,#0x00		;Store null in R3
				CMP		R1,R3			;Compare char to null
				BEQ		MSEnd			;End if the end of string is reached
				CMP		R1,#32		;Compare char to space
				BEQ		CharIsSpace
				CMP		R1,#65			;If 'A'
				BEQ		CharIsUppercase	
				CMP		R1,#66			
				BEQ		CharIsUppercase
				CMP		R1,#67			
				BEQ		CharIsUppercase
				CMP		R1,#68			
				BEQ		CharIsUppercase
				CMP		R1,#69			
				BEQ		CharIsUppercase
				CMP		R1,#70			
				BEQ		CharIsUppercase
				CMP		R1,#71			
				BEQ		CharIsUppercase
				CMP		R1,#72			
				BEQ		CharIsUppercase
				CMP		R1,#73			
				BEQ		CharIsUppercase
				CMP		R1,#74			
				BEQ		CharIsUppercase
				CMP		R1,#75			
				BEQ		CharIsUppercase
				CMP		R1,#76			
				BEQ		CharIsUppercase
				CMP		R1,#77			
				BEQ		CharIsUppercase
				CMP		R1,#78			
				BEQ		CharIsUppercase
				CMP		R1,#79			
				BEQ		CharIsUppercase
				CMP		R1,#80			
				BEQ		CharIsUppercase
				CMP		R1,#81			
				BEQ		CharIsUppercase
				CMP		R1,#82			
				BEQ		CharIsUppercase
				CMP		R1,#83			
				BEQ		CharIsUppercase
				CMP		R1,#84			
				BEQ		CharIsUppercase
				CMP		R1,#85			
				BEQ		CharIsUppercase
				CMP		R1,#86			
				BEQ		CharIsUppercase
				CMP		R1,#87			
				BEQ		CharIsUppercase
				CMP		R1,#88			
				BEQ		CharIsUppercase
				CMP		R1,#89
				BEQ		CharIsUppercase
				CMP		R1,#90				
				BEQ		CharIsUppercase
				CMP		R1,#48			;If 0
				BEQ		CharIsNumber
				CMP		R1,#49			
				BEQ		CharIsNumber
				CMP		R1,#50			
				BEQ		CharIsNumber
				CMP		R1,#51			
				BEQ		CharIsNumber
				CMP		R1,#52			
				BEQ		CharIsNumber
				CMP		R1,#53			
				BEQ		CharIsNumber
				CMP		R1,#54			
				BEQ		CharIsNumber
				CMP		R1,#55			
				BEQ		CharIsNumber
				CMP		R1,#56			
				BEQ		CharIsNumber
				CMP		R1,#57			
				BEQ		CharIsNumber
				STRB	R1,[R0,#0]		;If none of these then store char at pointer
				ADDS	R0,R0,#1		;Increment pointer
				B		MSCond
				
CharIsSpace
				MOVS	R1,#0x5F		;Move underscore into R1
				STRB	R1,[R0,#0]		;Store underscore at pointer 
				ADDS	R0,R0,#1		;Increment pointer
				B		MSCond
CharIsUppercase
				ADDS	R1,R1,#32		;Turn to lower case
				STRB	R1,[R0,#0]		;Store lower case char at pointer 
				ADDS	R0,R0,#1		;Increment pointer
				B		MSCond
CharIsNumber
				MOVS	R1,#0x23		;Move pound sign into R1
				STRB	R1,[R0,#0]		;Store underscore at pointer 
				ADDS	R0,R0,#1		;Increment pointer
				B		MSCond
MSEnd
				POP		{R0-R7,PC}
				BX		LR
;This subrouting reverses the characters of a null-terminated string in memory
;starting at the address in R0
;Input parameters
;	R0 starting address in memory of null terrminated string
;Output Parameters
;	None
;Modefied Registers
;	None
ReverseString
				PUSH	{R0-R7,LR}
				MOVS	R2,#0			;intialize string length counter (down-counter)
				MOVS	R3,#0x00		;Store null in R3
				MOVS	R4,#0			;initialize up-counter
RSFindStringLength
				LDRB	R1,[R0,R2]		;Get character at begining of string, store in R1
				CMP		R1,R3			;Compare to null
				BEQ		RSLoopCond	
				ADDS	R2,R2,#1		;Increment counter since we have not reached end of string
				B		RSFindStringLength
RSLoopCond
				CMP		R2,R4			;Compare counters
				BLS		RSEnd			;When equal(string is odd length) or down counter is less than up counter(string is even length)
				SUBS	R2,R2,#1		;One less than end
				B		RSLoop
RSLoop
				LDRB	R6,[R0,R4]		;Get character at begining, store in R5
				LDRB	R5,[R0,R2]		;Get character at end, store in R5
				STRB	R6,[R0,R2]		;Store char from begining at end
				STRB	R5,[R0,R4]		;Store char from end at begining
				ADDS	R4,R4,#1		;One more than begining
				B		RSLoopCond
RSEnd		
				POP		{R0-R7,PC}
				BX		LR
;The follwing subroutine intiializes the board by initializing 
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
		PUSH	{R0-R3}				;Push registers as to not overwrite original values
		SUBS	R1,R1,#1			;Amount of elements we can check
        MOVS    R3,R0               ;Move address into R3
		MOVS    R2,#0
LoopCond
        BL       GETCHAR             ;GetChar, store Char in R0
        CMP     R0,#0x0D            ;Make Sure Carraige return has not been selected
        BEQ     ENDLOOP             ;End if Carraige Return has been recieved
        CMP     R2,R1               ;Make sure the Max_String value hasnt been reached
        BEQ     LoopCond            ;End if the Max_String value has been reached
        BL 		PUTCHAR             ;Return  to console
        STRB 	R0,[R3,R2] 			;Store char in array 	
        ADDS 	R2,R2,#1 			;R2 increment
        B 		LoopCond
ENDLOOP		
        MOVS 	 R0,#00			    ;Null terminate
        STRB     R0,[R3,R2]         ;Store Null terminate 
        BL       NEXTLINE
        POP		 {R0-R3}		    ;Pop registers back off 
        B        MAIN

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
MainString  		DCB 	"Type a string command(g,h,m,p,r)>", 0
HelpString			DCB		"g (get), h (help), m (modify), p (print), r (reverse)", 0
InvalidString  		DCB 	":Invalid command", 0
InitialString		DCB		"Initial String", 0
;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;VaR0ables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
StartingPoint   SPACE   (4*MAX_STRING)


;>>>>>   end variables here <<<<<
            ALIGN
            END


