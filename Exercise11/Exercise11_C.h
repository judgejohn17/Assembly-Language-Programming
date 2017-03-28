/*********************************************************************/
/* Lab Exercise Eleven header file                                   */
/* Tests mixed C and assembly language programming to add 128-bit    */
/* unsigned numbers.  Prompts user to enter two numbers in hex       */
/* format to add, computes the result, and prints it.                */
/* Name:  R. W. Melton                                               */
/* Date:  November 9, 2015                                           */
/* Class:  CMPE 250                                                  */
/* Section:  All sections                                            */
/*********************************************************************/
typedef int Int32;
typedef unsigned int UInt32;
typedef char Int8;
typedef unsigned char UInt8;

typedef union {
  UInt32 Word[4];
  UInt8  Byte[16];
} UInt128;

/* assembly language subroutines */
int AddIntMultiU (UInt32 *Sum, UInt32 *Augend, UInt32 *Addend, 
                  int NumWords);
void GetStringSB (char String[], int StringBufferCapacity);
void PutStringSB (char String[], int StringBufferCapacity);
void Init_UART0_IRQ (void);
void Startup (void);

/* C subroutines */
/*int GetHexIntMulti (UInt32 *Number, int NumWords);
void PutHexIntMulti (UInt32 *Number, int NumWords);
int main (void);*/
int Reset_Handler (void) __attribute__((alias("main")));
