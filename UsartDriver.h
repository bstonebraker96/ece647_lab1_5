// UsartDriver.h
#ifndef UsartDriver_h
#define UsartDriver_h

#include <stm32f4xx.h>

#define BUFFER_SIZE 64

// Pseudo object for holding input and output data 
// Mostly used internally to USART code. 
typedef struct { unsigned char Buffer[BUFFER_SIZE]; 
				int InPtr, OutPtr;}
				SerialBuffer;

// Initialization code for setting empty buffers.
void SerialBufferInit( SerialBuffer *buffer )
{
	buffer->InPtr = 0;
	buffer->OutPtr = 0;
} // End of SerialBufferInit

// Inserts a character in to the buffer, 
// returns true if successful, and a false if an overflow occurred.
int SerialBufferInsert( SerialBuffer *buffer, unsigned char c )
{
	// Place data in buffer.
	buffer->Buffer[buffer->InPtr] = c;
	// advance pointer.
	buffer->InPtr++;
	// wrap if beyond buffer.
	if( buffer->InPtr == BUFFER_SIZE )
		buffer->InPtr = 0;
	// if after the wrap,
	if( buffer->InPtr == buffer->OutPtr )  // the pointers
		return 0;                          // match up, we had overflow.
	return 1;
} // End of SerialBufferInsert

// Extracts a character from the buffer,
// returns a true if there was a character in the buffer, 
// and false if buffer empty.
int SerialBufferExtract( SerialBuffer *buffer, unsigned char *c )
{
	if( buffer->InPtr == buffer->OutPtr )  // if the pointers match up, 
		return 0;                          // we have no data, so return false.
	// else pull data out and advance pointer
	*c = buffer->Buffer[buffer->OutPtr];
	// and advance pointer
	buffer->OutPtr++;
	// wrap if beyond buffer.
	if( buffer->OutPtr == BUFFER_SIZE )
		buffer->OutPtr = 0;
	return 1;
} // End of SerialBufferExtract

// Declaration of USART buffers.
SerialBuffer Serial_In, Serial_Out;

// The next four functions turn on and off the 
// Transmit and Receive interrupts.
void Usart_TxInterruptEnable( void )
{
	 uint32_t tmpreg = 0;
	
	 // Enable Tx Empty Interrupt in CR1 of USART3 
	 tmpreg  = USART3->CR1;
	tmpreg |= 0x0080; // Tx Interrupt enabled.
	USART3->CR1 = tmpreg;

} // End of Usart_TxInterruptEnable

void Usart_TxInterruptDisable( void )
{
	 uint32_t tmpreg = 0;
	
	 // Enable Tx Empty Interrupt in CR1 of USART3 
	 tmpreg  = USART3->CR1;
	tmpreg &= ~(0x0080); // Tx Interrupt enabled.
	USART3->CR1 = tmpreg;

} // End of Usart_TxInterruptDisable

void Usart_RcvInterruptEnable( void )
{
	 uint32_t tmpreg = 0;

	 // Enable Rcv Not Empty Interrupt in CR1 of USART3 
	 tmpreg  = USART3->CR1;
	tmpreg |= 0x0020; // Rcv Interrupt enabled.
	USART3->CR1 = tmpreg;

} // End of Usart_RcvIEnable

void Usart_RcvInterruptDisable( void )
{
	uint32_t tmpreg = 0;

	// Enable Rcv Not Empty Interrupt in CR1 of USART3 
	tmpreg  = USART3->CR1;
	tmpreg &= ~(0x0020); // Rcv Interrupt enabled.
	USART3->CR1 = tmpreg;

} // End of Usart_RcvInterruptDisable


// The following is the interrupt service routine for
// USART3.
void USART3_IRQHandler( void )
{
	unsigned char ch;

	// Check for data coming in.
	if( USART3->SR & USART_SR_RXNE ) // Receive 
	{
		// Read data in (DR) and insert into the input buffer.
		SerialBufferInsert( &Serial_In, USART3->DR );
	} // End of Receive data test.

	// Check if transmitter free,
	if( USART3->SR & USART_SR_TXE )
	{
		// then if there is data in the out buffer,
		if( SerialBufferExtract( &Serial_Out, &ch ) )
		{
			// Transmit data by placing it in DR
			USART3->DR = ch;
		}
		else // if no data in buffer.
		{
			// Disable Tx Interrupt.
			Usart_TxInterruptDisable();
		} // 			
	} // end of transmitter free test	
} // end of USART3_IRQHandler

// This function is called by the main program at the start  to properly configure all the registers in the USART system
// and establish the interrupt system.
void Usart_Init( int SystemClock, int BaudRate )
{
	// Clock enabled for SPI2 and GPIOB.
	RCC->APB1ENR  |= RCC_APB1ENR_USART3EN;
	RCC->AHB1ENR  |= RCC_AHB1ENR_GPIODEN;
	GPIOD->MODER  |=  (0x028a0000);     // PD 8, 9, 11 & 12 to 
	GPIOD->MODER  &= ~(0x01450000);     // Alternate Function
	GPIOD->AFR[1] |=  (0x00077077);    // Set PD 8, 9, 11 and 12's Alternate
	GPIOD->AFR[1] &= ~(0x00088088);    // Function to Usart3 (AF7).

	// Set CR1 for USART3 
	USART3->CR1 |= 0x200c;	// 16 oversample, Usart Enabled, 
							// Transmitter and Receiver enabled.

	// Set CR2 for USART3 
	USART3->CR2 |= 0x3000; // 2 stop bits, 

	// Initialize buffers 
	SerialBufferInit( &Serial_In  );
	SerialBufferInit( &Serial_Out );

	NVIC_EnableIRQ( USART3_IRQn );  // Enable USART3 interrupts.

	{ // Scope for computing Baud Rate values.
		float Divisor;
		int IntDivisor, FracDivisor;
		Divisor = (float) ( SystemClock ) / (float) ( 64 * BaudRate);
		IntDivisor = (int) Divisor;
		Divisor -= (float)IntDivisor;
		FracDivisor = (int)( 16.0F * Divisor + 0.5F );
		USART3->BRR = ( IntDivisor << 4 ) + FracDivisor ; // 
	}
	// Start up interrupts for receive,
	Usart_RcvInterruptEnable();

} // End of Usart_Init

// This function is called by the main program 
// when a character is to be transmitted.
void Usart_Transmit( unsigned char out )
{
	// Place character into buffer.
	SerialBufferInsert( &Serial_Out, out );
	// Activate transmitter interrupt.
	Usart_TxInterruptEnable();
	return;
} // End of Usart_Transmit

// This function is called by the main program  on a regular basis, and if a character has been 
// received, it loads the character received into *in, and returns a true, 
// otherwise it returns a false.
int Usart_Receive( unsigned char *in )
{
	return SerialBufferExtract( &Serial_In, in );
} // End of Usart_Receive

// The following three functions support sending data out to a terminal.
// This first simply transmit a string of characters.
void Usart_Send_String( unsigned char *out )
{
    // loop until reaching a null character.
    while( *out )
	    Usart_Transmit( *out++ ); // Send character and move to next 
} // End of Usart_Send_String

// Transmits a number as a string of ascii characters in a decimal format.
void Usart_Send_Decimal( int number )
{
    // if greater than 0 to 9. 
    if( number > 9 )
	   Usart_Send_Decimal( number/10 ); // move to next character.
	// Send out last digit as ascii character.
	Usart_Transmit( '0' + number % 10 );
	
} // End of Usart_Send_Decimal


// Transmits a number as a string of ascii characters in a hexadecimal format.
void Usart_Send_Hexadecimal( int number )
{
    // if greater than 0 to F
    if( number > 15 )
	   Usart_Send_Hexadecimal( number / 16 ); // move to next character.
    // Pull out bottom digit.
	number = number % 16;
	// if in the range of A to F
	if( number > 9 )
	   Usart_Transmit( 'A' + number - 10 );
	else // if in the range 0 to 9
	   Usart_Transmit( '0' + number );
	   
} // End of Usart_Send_Hexadecimal

// Support function for transmitting a floating point number.
void Usart_Send_Fractional( float number, int FractionalDigits )
{
	// Check if more digits need to be transmitted.
	if( FractionalDigits )
  {	
		// Pull next digit up to integer part.
		number *= 10.0f;
		// Send out digit.
    Usart_Transmit( '0' + ( ( (int) number) % 10 ) );
		// Move on to remaining factional.
		Usart_Send_Fractional( number - (int) number, 
		                       FractionalDigits - 1 );
	}
	return;
} // End of Usart_Send_Fractional

// Transmits a floating point number as a string of ascii 
// characters through the USART.
void Usart_Send_Float( float number, int FractionalDigits )
{
  if( number < 0.0f )
	{
		Usart_Transmit( '-' );
		number = -number;
	} // end of negative number handling.
	
	// if less than 1, 
  if( number >= 1.0f )
	    Usart_Send_Decimal( (int) number ); // send out integer part.
  else                                    // else 
      Usart_Transmit( '0' );		          // print out leading zero
	
  Usart_Transmit( '.' ); // Send decimal point.
	// then send fractional part.
  Usart_Send_Fractional( number - (int) number, FractionalDigits );
	
} // End of Usart_Send_Float

#endif
