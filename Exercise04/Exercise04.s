           ;Iterations and Subroutines 
;****************************************************************
;This exercise tests my knowledge of iterations and subroutines 
;Essentialy, it divides two numbers 
;Name:  John Judge
;Date:  2/18/16
;Class:  CMPE-250
;Section:  Section 02 Thursdays, 2:00PM-3:50PM
;---------------------------------------------------------------
;Keil Simulator Template for KL46
;R. W. Melton
;January 23, 2015
;****************************************************************
;Assembler directives
            THUMB
            OPT    64  ;Turn on listing macro expansions
;****************************************************************
;EQUates
MAX_DATA        EQU     25
;Vectors
VECTOR_TABLE_SIZE EQU 0x000000C0
VECTOR_SIZE       EQU 4           ;Bytes per vector
;Stack
SSTACK_SIZE EQU  0x00000100
;****************************************************************
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
            EXPORT  Reset_Handler
Reset_Handler
main
;---------------------------------------------------------------
;>>>>> begin main program code <<<<<
            BL InitData         ;(2.e)
Petunia
            BL LoadData         ;(2.f)
			BCS Quit 
            LDR R1,=P			;Load P to R1
            LDR R1,[R1,#0]
            LDR R0,=Q			;Load Q to R0
            LDR R0,[R0,#0]
            
            BL DIVU             ;Call DIVU (2.g)
            BCS INVALID         ;Branch if Carry is set, Division was valis (2.h)
			
			LDR R2,=P    	    ;Load Register (Say P is R0)
			STR R0,[R2,#0]	    ;Store Value (Say R1 is equal to R1 and immediate0)
			LDR R2,=Q    	    ;Load Register (Say Q is R0)
			STR R1,[R2,#0]	    ;Store Value (Say R0 is equal to R0 and immediate0)
			B Mulaney 
INVALID

            ;Store P
			MOVS R0,#0			;Set R0 to 0
            MVNS R0,R0          ;Set R0=P=FFFFFFFF
			LDR R2,=P    	    ;Load Register (Say P is R0)
			STR R0,[R2,#0]	    ;Store Value (Say R1 is equal to R1 and immediate0)
            ;Store Q
			MOVS R1,#0			;Set R1 to 0
            MVNS R1,R1          ;Set R1=q=FFFFFFFF
			LDR R2,=Q    	    ;Load Register (Say Q is R0)
			STR R1,[R2,#0]	    ;Store Value (Say R0 is equal to R0 and immediate0)

Mulaney 
            BL TestData         ;(2.i) Call TestData, checks division, stores results in Results Array
			B  Petunia
Quit
;>>>>>   end main program code <<<<<
;Stay here
            B       .
;---------------------------------------------------------------
;>>>>> begin subroutine code <<<<<
DIVU        
            PUSH    {R2}        ;store R2 Value
            MOVS    R2,#0       ;move 0 to R2 for quotient
            CMP     R0,#0       ;Compare divisor to 0
            BEQ     SETCARRY    ;if divisor = 0 go to SETCARRY
            
WHILE       
            CMP     R1,R0       ;Compare R1 to R0
            BLO     ENDWHILE    ;if dividend<Divisor End loop
            ADDS    R2,R2,#1    ;Add 1 to quotient
            SUBS    R1,R1,R0    ;Dividend - divisor
            B       WHILE       ;branch to start of while
            
ENDWHILE    
            MOVS    R0,R2       ;move quotient to R0, so R0 remainder R1
            POP     {R2}        ;revert R2 to value before subroutine
            PUSH    {R0,R1}     ;push R0 and R1
            MRS     R0,APSR     ;Set C flag to 0
            MOVS    R1,#0x20    ;
			LSLS    R1,#24		;Shift 24 places (to most significant byte)
            BICS    R0,R0,R1    ; 
            MSR     APSR,R0     ; 
            POP     {R0,R1}     ;revert R0 and R1 to answer 
            BX      LR          ;Go back to program
SETCARRY    
			POP     {R2}		;Pop R2
            PUSH    {R0,R1}     ;Store R0 and R1
            MRS     R0,APSR     ;Set C flag to 1
            MOVS    R1,#0x20    ;
			LSLS    R1,#24		;Shift 24 places (to most significant byte)
            ORRS    R0,R0,R1    ;
            MSR     APSR,R0     ;
            POP     {R0,R1}     ;Revert R0 and R1 to answer
            BX      LR          ;Go back to program 

;>>>>>   end subroutine code <<<<<
            ALIGN
;****************************************************************
;Machine code provided for Exercise Four
;R. W. Melton 9/14/2015
;Place at the end of your MyCode AREA
            AREA    |.ARM.__at_0x4000|,CODE,READONLY
InitData    DCI.W   0x26002700
            DCI     0x4770
LoadData    DCI.W   0xB40FA316
            DCI.W   0x19DBA13D
            DCI.W   0x428BD209
            DCI.W   0xCB034A10
            DCI.W   0x4B116010
            DCI.W   0x60193708
            DCI.W   0x20000840
            DCI.W   0xBC0F4770
            DCI.W   0x20010840
            DCI     0xE7FA
TestData    DCI.W   0xB40F480C
            DCI.W   0xA13419C0
            DCI.W   0x19C93808
            DCI.W   0x39084A07
            DCI.W   0x4B076812
            DCI.W   0x681BC00C
            DCI.W   0x68084290
            DCI.W   0xD1046848
            DCI.W   0x4298D101
            DCI.W   0xBC0F4770
            DCI.W   0x1C76E7FB
            ALIGN
PPtr        DCD     P
QPtr        DCD     Q
ResultsPtr  DCD     Results
            DCQ     0x0000000000000000,0x0000000000000001
            DCQ     0x0000000100000000,0x0000000100000010
            DCQ     0x0000000200000010,0x0000000400000010
            DCQ     0x0000000800000010,0x0000001000000010
            DCQ     0x0000002000000010,0x0000000100000007
            DCQ     0x0000000200000007,0x0000000300000007
            DCQ     0x0000000400000007,0x0000000500000007
            DCQ     0x0000000600000007,0x0000000700000007
            DCQ     0x0000000800000007,0x8000000080000000
            DCQ     0x8000000180000000,0x000F0000FFFFFFFF
            DCQ     0xFFFFFFFFFFFFFFFF,0xFFFFFFFFFFFFFFFF
            DCQ     0x0000000000000000,0x0000000000000010
            DCQ     0x0000000000000008,0x0000000000000004
            DCQ     0x0000000000000002,0x0000000000000001
            DCQ     0x0000001000000000,0x0000000000000007
            DCQ     0x0000000100000003,0x0000000100000002
            DCQ     0x0000000300000001,0x0000000200000001
            DCQ     0x0000000100000001,0x0000000000000001
            DCQ     0x0000000700000000,0x0000000000000001
            DCQ     0x8000000000000000,0x0000FFFF00001111
            ALIGN
;****************************************************************
;****************************************************************
;Vector Table Mapped to Address 0 at Reset
;Linker requires __Vectors to be exported
            AREA    RESET, DATA, READONLY
            EXPORT  __Vectors
            EXPORT  __Vectors_End
            EXPORT  __Vectors_Size
__Vectors 
                                      ;ARM core vectors
            DCD    __initial_sp       ;00:end of stack
            DCD    Reset_Handler      ;reset vector
            SPACE  (VECTOR_TABLE_SIZE - (2 * VECTOR_SIZE))
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
            AREA    |.ARM.__at_0x1FFFE000|,DATA,READWRITE,ALIGN=3
            EXPORT  __initial_sp
;Allocate system stack
            IF      :LNOT::DEF:SSTACK_SIZE
SSTACK_SIZE EQU     0x00000100
            ENDIF
Stack_Mem   SPACE   SSTACK_SIZE
__initial_sp
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
P       SPACE       4       ;Variable P, Word
Q       SPACE       4       ;Variable Q, Word
Results SPACE       (2*MAX_DATA*4)   ;Word Array with 
            ALIGN
;>>>>>   end variables here <<<<<
            END