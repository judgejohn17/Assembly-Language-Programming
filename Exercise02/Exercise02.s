;TTL CMPE 250 Exercise Two
;****************************************************************
;This program solves a function, given in decimal, in hexadecimal 
;f(x)=-5+62-(9/4)-(7*9)+58+17
;Name:  <John Judge>
;Date:  <2/4/16>
;Class:  CMPE-250
;Section:  <Section L2, Thursday, 2:00pm-3:50pm>
;---------------------------------------------------------------

;****************************************************************
;Assembler directives
            THUMB
            OPT    64  ;Turn on listing macro expansions
;****************************************************************
;EQUates 
DIV4    EQU         2
MULT8   EQU         3
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
        NOP
;R0 <- (-5+62)
        MOVS R2, #5     ;Store Decimal-5 in R2
        RSBS R1,R2,#0           ;Negate Decimal-5 store value in R1
        MOVS R2, #62     ;Store Decimal-62 in R2
        ADDS R0,R1,R2           ;Adds contents of R2 and R1 together and stores value in R0
;R0 <- (R0-(9/4))		
        MOVS R1, #9			;Store decimal-9 in R1
        ASRS R1,R1,#DIV4	;Divide the contents of R1 by 4 and store value in R1
        SUBS R0,R0,R1		;Subtract contents of R0 by R1 and store value in R0
;R0 <- (R0-(7x9))
        MOVS R1, #7			;Store decimal-7 in R1
        LSLS R2,R1,#MULT8	;Multiply the content of R1 by 8 and store value in R1
        ADDS R1,R2,R1		;Add contents of R1 and R2 together and store in R1
        SUBS R0,R0,R1		;Subtract contents of R0 by R1 and store value in R0
;R0 <- (R0+58)
        MOVS R1, #58		;Store decimal-58 in R1
        ADDS R0,R0,R1		;Add contents of R0 and R1 together and store vlaue in R0
;R0 <- (R0+17)
        MOVS R1, #17		;Store decimal-17 in R1
        ADDS R0,R0,R1		;Add contents of R0 and R1 together and store value in R0. This is the final value
;>>>>>   end main program code <<<<<
;Stay here
 x            B       .
;---------------------------------------------------------------
;>>>>> begin subroutine code <<<<<
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
;>>>>>   end variables here <<<<<
            END