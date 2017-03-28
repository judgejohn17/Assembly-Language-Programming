
;****************************************************************
;Exercise03
;This program solves a system of equations
;Name:  John Judge
;Date:  2/11/16
;Class:  CMPE-250
;Section:  Section 02, Thursdays 200PM-3:50PM
;---------------------------------------------------------------
;****************************************************************
;Assembler directives
            THUMB
            OPT    64  ;Turn on listing macro expansions
;****************************************************************
;EQUates
MULT2      EQU      1
MULT4      EQU      2
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
;NEED TO FIND WAYS TO CHECK G and shift PQR for G
;>>>>> begin main program code <<<<<
;Load P,Q,&R into Registers for F
    ;Load P to R1
            LDR R1,=P
            LDR R1,[R1,#0]
    ;Load Q to R2
            LDR R2,=Q
            LDR R2,[R2,#0]
    ;Load R to R3
            LDR R3,=R
            LDR R3,[R3,#0]
;Equate F=2P-3Q+R+51, store F value in R4 (Should only use R0,R1,R2,R3,R4(;except check uses R7)(R0 is used by Const_F))
    ;Store -128 in a register so we can check range later 
            MOVS R7, #128           ;Put 127 into R7
    ;P expression, P=R1
            LSLS R4,R1,#MULT2       ;Multiply P by 2, store result in R4
        ;check 
            CMP R4,#127             ;Compare register to 127
            BGT FUBAR1              ;Branch greater than
            CMN R4,R7               ;Compare register to -128
            BLT FUBAR1              ;Branch less than
        ;end check    
    ;Q term, Q=R2
            LSLS R0,R2,#MULT2       ;Multiply Q by 2, store result in R0
        ;check 
            CMP R0,#127             ;Compare register to 127
            BGT FUBAR1              ;Branch greater than
            CMN R0,R7               ;Compare register to -128
            BLT FUBAR1              ;Branch less than
        ;end check
            ADDS R0,R2,R0           ;Add values of R0 and Q, store values in R0
        ;check 
            CMP R0,#127             ;Compare register to 127
            BGT FUBAR1              ;Branch greater than
            CMN R0,R7               ;Compare register to -128
            BLT FUBAR1              ;Branch less than
        ;end check
            SUBS R4,R4,R0           ;Subtract contents of R4 by R0, store value in R4
        ;check 
            CMP R4,#127             ;Compare register to 127
            BGT FUBAR1              ;Branch greater than
            CMN R4,R7               ;Compare register to -128
            BLT FUBAR1              ;Branch less than
        ;end check
    ;R term, R=R3
            ADDS R4,R4,R3           ;Add contents of R4 and R, store value in R4
        ;check 
            CMP R4,#127             ;Compare register to 127
            BGT FUBAR1              ;Branch greater than
            CMN R4,R7               ;Compare register to -128
            BLT FUBAR1              ;Branch less than
        ;end check
    ;+51, R=R0
            LDR R0,=Const_F         ;Load Register (Say Const_F is R5)
            LDR R0,[R0,#0]          ;Load Value (Say R5 is value of R5 and immediate 0)
            ADDS R4,R4,R0           ;Adds R0 to R4, stores value in R4
        ;check 
            CMP R4,#127             ;Compare register to 127
            BGT FUBAR1              ;Branch greater than
            CMN R4,R7               ;Compare register to -128
            BLT FUBAR1              ;Branch less than
        ;end check
    ;Store F
            LDR R0,=F               ;Load Register (Say F is R4)
            STR R4,[R0,#0]           ;Store Value (Say R4 is equal to R4 and immediate0)
GoToG
;Load P,Q,&R into Registers fot G
    ;Load P to R1
            LDR R1,=P
            LDR R1,[R1,#0]
			LSLS R1,#24				;Shift to most significant byte
    ;Load Q to R2
            LDR R2,=Q
            LDR R2,[R2,#0]
			LSLS R2,#24				;Shift to most significant byte
    ;Load R to R3
            LDR R3,=R
            LDR R3,[R3,#0]
			LSLS R3,#24 			;Shift to most significant byte
	;Check if negative
			MOVS R6,#0				;Set R6 as 0
	;Clear G
			MOVS R5,#0
;Equate G=5P-4Q-2R+7, store G value in R5 (Should only use R0,R1,R2,R3,R5 ((R0 is used for constant))
    ;P term, P=R1
			CMN R1,R6				;Check if negative
			BLT negativeP			;Branch if negative 
            LSLS R5,R1,#MULT4       ;Multiply P by 4, store vlaue in R5
			;Check Shift
			BCS FUBAR4
			;End Check Shift
            ADDS R5,R1,R5           ;Adds P and R5 (multiplies by 5), stores value in R5
G1
		;Check Arithemtic
			BVS FUBAR4
		;EndCheckArithmetic
    ;Q term, Q=R2
			CMN R2,R6				;Check if negative
			BLT negativeQ			;Branch if negative 
            LSLS R0,R2,#MULT4       ;Multiplies Q by 4, stores value in R0
			;Check Shift
			BCS FUBAR4
			;End Check Shift
G2
            SUBS R5,R5,R0           ;Subtracts R5 by R0, stores value in R5
			;Check Arithemtic
			BVS FUBAR4

    ;R term, R=R3
			CMN R3,R6				;Check if negative
			BLT negativeR			;Branch if negative 
            LSLS R0,R3,#MULT2       ;Multiply R by 2, store vlaue in R0
			;Check Shift
			BCS FUBAR4
			;End Check Shift;
G3
            SUBS R5,R5,R0           ;Subtract R5 by R0, stores value in R5
			;Check Arithemtic
			BVS FUBAR4

    ;+7
            LDR R0,=Const_G         ;Load Register (Say Const_G is R6)
            LDR R0,[R0,#0]          ;Load Value (Say R6 is value of R6 and immediate 0)
			LSLS R0,R0,#24			;Shift Constant
            ADDS R5,R5,R0           ;Adds R0 to conents of R5
		;Check Arithemtic
			BVS FUBAR4
    ;Store G
			ASRS R5,#24				;Shift back to least signficant byte
            LDR R0,=G               ;Load Register (Say G is R5)
            STR R5,[R0,#0]           ;Store Value (Say R5 is equal to R5 and immediate0)
GoToH
;Load P,Q,&R into Registers for H
    ;Load P to R1
            LDR R1,=P
            LDR R1,[R1,#0]
    ;Load Q to R2
            LDR R2,=Q
            LDR R2,[R2,#0]
    ;Load R to R3
            LDR R0,=R
            LDR R3,[R0,#0]
	;Clear R0
			MOVS R0,#0
;Equate H=P-2Q+R-91, store H value in R6 (Should only use R0,R1,R2,R3,R6 ((R7 is used for overflow check)(R0 is used for constant))
	;Store -128 in a register so we can check range later 
            MOVS R7, #128           ;Put 128 into R7
    ;P term, P=R1
            MOVS R6,R1              ;Sets R6 equal to P
    ;Q term,Q=R2
            LSLS R0,R2,#MULT2       ;Multiplies Q by 2, stores value in R0
		;check 
			CMP R0,#127         	;Compare register to 127
			BGT FUBAR2           	;Branch greater than
			CMN R0,R7           	;Compare register to -128
			BLT FUBAR2           	;Branch less than
		;end check 
            SUBS R6,R6,R0           ;Subtracts contents of R6 by R0, stores value in R6
		;check 
            CMP R6,#127             ;Compare register to 127
            BGT FUBAR2              ;Branch greater than
            CMN R6,R7               ;Compare register to -128
            BLT FUBAR2              ;Branch less than
        ;end check 
    ;R term, Q=R3
            ADDS R6,R6,R3           ;Adds R to R6
		;check 
            CMP R6,#127             ;Compare register to 127
            BGT FUBAR2              ;Branch greater than
            CMN R6,R7               ;Compare register to -128
            BLT FUBAR2              ;Branch less than
        ;end check 
    ;-91
            LDR R0,=Const_H         ;Load Register (Say Const_G is R6)
            LDR R0,[R0,#0]          ;Load Value (Say R6 is value of R6 and immediate 0)
            ADDS R6,R6,R0           ;Adds R0 to conents of R6
		;check 
            CMP R6,#127             ;Compare register to 127
            BGT FUBAR2              ;Branch greater than
            CMN R6,R7               ;Compare register to -128
            BLT FUBAR2              ;Branch less than
        ;end check 
    ;Store H
            LDR R0,=H               ;Load Register (Say H is R6)
            STR R6,[R0,#0]           ;Store Value (Say R6 is equal to R6 and immediate0)
GoToResult
;Equate Result=F+G+H, Store Result in R0
			LDR R1,=F
			LDR R1,[R1,#0]
			LDR R2,=G
			LDR R2,[R2,#0]
			LDR R3,=H
			LDR R3,[R3,#0]
    ;Clear R0
            MOVS R0,#0              ;Clears R0 (idk whats in there)
    ;R0+=F
            ADDS R0,R0,R1           ;Adds F to R0
		;check 
            CMP R0,#127             ;Compare register to 127
            BGT FUBAR3              ;Branch greater than
            CMN R0,R7               ;Compare register to -128
            BLT FUBAR3              ;Branch less than
        ;end check
    ;R0+=G
            ADDS R0,R0,R2           ;Adds G to R0
		;check 
            CMP R0,#127             ;Compare register to 127
            BGT FUBAR3              ;Branch greater than
            CMN R0,R7               ;Compare register to -128
            BLT FUBAR3              ;Branch less than
        ;end check
    ;R0+=H
            ADDS R0,R0,R3           ;Adds H to R0
		;check 
            CMP R0,#127             ;Compare register to 127
            BGT FUBAR3               ;Branch greater than
            CMN R0,R7               ;Compare register to -128
            BLT FUBAR3               ;Branch less than
        ;end check
    ;Store Result
            LDR R7,=Result          ;Load Register (Say Result is R0)
            STR R0,[R7,#0]           ;Store Value (Say R0 is equal to R0 and immediate0)

GoToEnd
            NOP
;>>>>>   end main program code <<<<<
;Stay here
            B       .
;---------------------------------------------------------------
;>>>>> begin subroutine code <<<<<
;Store F if FUBAR1
FUBAR1
			MOVS R4,#0   			;Clear R4
			LDR R5,=F				;Load Register (Say F is R4)
			STR R4,[R5,#0]			;Store Value (Say R4 is equal to R4 and immediate0)
			;Not sure how this works but it brings you back to the main code using the LR(R14 Register)
			LDR	R0,=GoToG
			MOV R14,R0
			BX LR
;Store H if FUBAR2
FUBAR2
			MOVS R6,#0  			;Clear R6
			LDR R7,=H				;Load Register (Say H is R6)
			STR R6,[R7,#0]			;Store Value (Say R6 is equal to R6 and immediate0)
			;Not sure how this works but it brings you back to the main code using the LR(R14 Register)
			LDR	R0,=GoToResult
			MOV R14,R0
			BX LR
;Store Result if FUBAR3
FUBAR3
			MOVS R0,#0    			;Clear R0
			LDR R0,=Result			;Load Register (Say Result is R0)
			STR R0,[R0,#0]			;Store Value (Say R0 is equal to R0 and immediate0)
			;Not sure how this works but it brings you back to the main code using the LR(R14 Register)
			LDR	R0,=GoToEnd
			MOV R14,R0
			BX LR
;Store G if FUBAR4
FUBAR4
			MOVS R5,#0				;Clear R5
			LDR R6,=G				;Load Register (Say G is R5)
			STR R5,[R6,#0]			;Store Value (Say R5 is equal to R and immediat0 )
			;Not sure how this works but it brings you back to the main code using the LR(R14 Register)
			LDR	R0,=GoToH
			MOV R14,R0
			BX LR
negativeP
			RSBS R1,R1,#0			;Make it positive
			LSLS R5,R1,#MULT4		;MULT4
			ADDS R5,R5,R1			;MULT5
			RSBS R5,R5,#0			;Make negative
			BVS FUBAR3				;Check for Overflow
			;Not sure how this works but it brings you back to the main code using the LR(R14 Register)
			LDR	R7,=G1
			MOV R14,R7
			BX LR
negativeQ
			RSBS R2,R2,#0			;Make it positive
			LSLS R0,R2,#MULT4		;MULT4
			RSBS R0,R0,#0			;Make negative
			BVS FUBAR3				;Check for Overflow
			;Not sure how this works but it brings you back to the main code using the LR(R14 Register)
			LDR	R7,=G2
			MOV R14,R7
			BX LR
negativeR
			RSBS R3,R3,#0			;Make it positive
			LSLS R0,R2,#MULT2		;MULT4
			RSBS R0,R0,#0			;Make negative
			BVS FUBAR3				;Check for Overflow
			;Not sure how this works but it brings you back to the main code using the LR(R14 Register)
			LDR	R0,=G3
			MOV R14,R7
			BX LR
;Check Carry of G
;>>>>>   end subroutine code <<<<<
            ALIGN
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
Const_F     DCD     51     ;Constant F
Const_G     DCD     7      ;Constant G
Const_H     DCD     -91    ;Constant H
;>>>>>   end constants here <<<<<
;****************************************************************
            AREA    |.ARM.__at_0x1FFFE000|,DATA,READWRITE,ALIGN=3
            EXPORT  __initial_sp
;Allocate system stack
            IF      :LNOT::DEF:SSTACK_SIZE
SSTACK_SIZE EQU     0x00000100
            ENDIF
Stack_Mem   SPACE   SSTACK_SIZE
__initial_sp
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
F       SPACE       4;Variable F, Word
G       SPACE       4;Variable G, Word
H       SPACE       4;Variable H, Word
P       SPACE       4;Variable P, Word
Q       SPACE       4;Variable Q, Word
R       SPACE       4;Variable R, Word
Result  SPACE       4;Variable Result, Word
;>>>>>   end variables here <<<<<
            END