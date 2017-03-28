/*********************************************************************/
/* Lab Exercise Eleven                                               */
/* C source file for Exercise 11                                     */
/* Name:  John Judge                                                 */
/* Date:  4/25/16                                                    */
/* Class:  CMPE 250                                                  */
/* Section:  Thursdays 2:00pm-3:50pm                                 */
/*********************************************************************/
#include "Exercise11_C.h"  

#define FALSE      (0)
#define TRUE       (1)
#define MAX_HEX_STRING ((sizeof(UInt128) << 1) + 1)
#define MAX_STRING (79)
#define NUMBER_BITS (128)
#define NUMBER_WORDS (4)
#define NUM_CHARS ((NUMBER_WORDS * 8) + 1)

//Determines length of string
int length(char String[]) {
  unsigned int i;
  unsigned int Counter = 0;
	
  for(i = 0; i < MAX_STRING; i++) {
    if (String[i] != 0) {
			Counter ++;
    }
    else {
      return Counter;   
    }
  }
  return Counter;
}

//Converts to ascii 
int ConvertsToAscii(int numericRep) {
  if(numericRep >= 0 && numericRep <= 9) {
    return numericRep + '0'; 
  }
  else {
    return numericRep + 55;
  }
}

/*********************************************************************/
/* Gets user string input of hex representation of an multiword      */
/* unsigned number of NumWords words, and converts it to a binary    */
/* unsigned NumWords-word number.                                    */
/* If user input is invalid, returns 1; otherwise returns 0.         */  
/* Calls:  GetStringSB                                               */
/*********************************************************************/
int GetHexIntMulti (UInt32 *Number, int NumWords) {
	unsigned int i;
    int BinaryRepresentation;
    char input[MAX_HEX_STRING];				//Memory location to take user input 
    int j = 0;
	int ConvertoLowerAscii = 87;
	int ConvertoUpperAscii = 55;
	int ConvertoNumberAscii = '0';
    
    

    
    //for (i = 0; i <MAX_STRING; i++){
            //input[i] = 0; //Store null for each place in string
    //}
	GetStringSB(input, MAX_HEX_STRING );
	
	if(NumWords * 8 != length(input)) {	//Length of string is not okay
		int john = length(input); // for de;lnklrmgfj
		return 1;
    }	
	for (i = length(input); i > 0; i=i-2) {
        char MSnibble = input[i-2];
        char LSnibble = input[i-1]; 
		if(LSnibble >= 'a' && LSnibble <= 'f') {	//Convert to lowercase ascii
		  LSnibble = LSnibble - ConvertoLowerAscii;	
		}	
		else if(LSnibble >= 'A' && LSnibble <= 'F') {	//convert to uppercase ascii
		  LSnibble = LSnibble - ConvertoUpperAscii;
		}
		else if(LSnibble >= 0x30 && LSnibble <= 0x39) {	//convert to number ascii
			LSnibble = LSnibble - ConvertoNumberAscii;
		}
		else {//If something else came in 
		  return 1;
		}
        if(MSnibble >= 'a' && MSnibble <= 'f') {	//Convert to lowercase ascii
		  MSnibble = MSnibble - ConvertoLowerAscii;	
		}	
		else if(MSnibble >= 'A' && MSnibble <= 'F') {	//convert to uppercase ascii
		  MSnibble = MSnibble - ConvertoUpperAscii;
		}
		else if(MSnibble >= 0x30 && MSnibble <= 0x39) {	//convert to number ascii
			MSnibble = MSnibble - ConvertoNumberAscii;
		}
		else {//If something else came in 
		  return 1;
		}
        MSnibble = MSnibble << 4;
        BinaryRepresentation = MSnibble + LSnibble; 
        ((UInt8 *)Number)[j] = BinaryRepresentation; 
        j++;
	}

	return 0;	//If result is valid return 0
} 
/*********************************************************************/
/* Getsan n-word unsigned number fporm user typer in hexadecimal     */
/* and converts it to a binary unsigned NumWords-word number.   */
/* Parameters:														 */
/*				*Number: Number to n-word usnigned number          */		 
/*				NumWords:	 Number's vaslue					     */
/*                                                                   */  
/* Calls:  GetStringSB                                               */
/*********************************************************************/
void PutHexIntMulti (UInt32 *Number, int NumWords) {
    
	int Counter = 0;
	int NumberOfBytes = (NumWords * 4);
	char ResultString[NUM_CHARS];
	unsigned int i;
	
	for(i = NumberOfBytes; i > 0; i--) {		//Read each byte of each words
		char byteValue = ((UInt8 *) Number) [i-1];
		char leastSigByteMsk = 0x0F;
		char leastSigChar = byteValue & leastSigByteMsk; 
		char mostSigByteMsk = 0xF0;
		char mostSigChar = (byteValue & mostSigByteMsk) >> 4;
		mostSigChar = ConvertsToAscii(mostSigChar);		//Convert most significant char to ascii 
		leastSigChar = ConvertsToAscii(leastSigChar);	//Convert least significant char to ascii
		ResultString[Counter] = mostSigChar;
		Counter++;								//Increment offset
		ResultString[Counter] = leastSigChar;
		Counter++;								//Increment offset 
	}
	ResultString[Counter] = 0;				//null terminate
	PutStringSB(ResultString, MAX_STRING);	//Print to console 
}

int main (void) {
    UInt128 value1;
    UInt128 value2;
    UInt128 resultAddress;
	int additionStatus;
    int result; 
    
    __asm("CPSID I");  // mask interrupts 
    Startup ();
    Init_UART0_IRQ ();
    __asm("CPSIE I");
    
	result = 0;
	for(;;) {
        PutStringSB(" Enter first 128 bit hex number: 0x", MAX_STRING);
        result = GetHexIntMulti(value1.Word, NUMBER_WORDS);
    
        while(result != 0) {
          PutStringSB("\r\nInvalid number--try again:       0x", MAX_STRING);
          result = GetHexIntMulti(value1.Word, NUMBER_WORDS);
        }
        PutStringSB("\r\nEnter 128-bit hex number to add: 0x", MAX_STRING);
        result = GetHexIntMulti(value2.Word, NUMBER_WORDS);
        while(result != 0) {
          PutStringSB("\r\nInvalid number--try again:       0x", MAX_STRING);
          result = GetHexIntMulti(value2.Word, NUMBER_WORDS);
        }
        additionStatus = AddIntMultiU(resultAddress.Word, value1.Word, value2.Word, NUMBER_WORDS);
        PutStringSB("\r\n", MAX_STRING);
        PutStringSB("                            Sum: 0x", MAX_STRING);
        if(additionStatus == 0) {
            PutHexIntMulti(resultAddress.Word, NUMBER_WORDS);
        }
        else {
			PutStringSB("OVERFLOW", MAX_STRING);	
        }
        PutStringSB("\r\n", 2);
	}

} 
