// ADC Support.
int Channel2, Channel1; 
const float b0 = 0.1, b1 = 0.2, b2 = 0.1,a1 = -1.0, a2 = 0.5;

float IIR2_y[2] = { 0.0 };

float IIR_Implement2(float x){
	float y; // output
	// Implement difference equation
	y = b0 * x + IIR2_y[0];
	IIR2_y[0] = b1 * x - a1 * y + IIR2_y[1];
	IIR2_y[1] = b2 * x - a2 * y;
	return y;
} 
// IIR_Implement2
// Compute output using Transposed 

const int LED_Indicator = 2;
// DAC service.
void DAC_Data_Send(  int Ch1, int Ch2 )
{
	  DAC->DHR12RD = (Ch2<<16) | Ch1;  // Send data over to DAC

    DAC->SWTRIGR = 3;  // Software trigger DAC
	  
}

// ADC_IRQHandler Function – Only called when conversion is complete.
void ADC_IRQHandler(void){
  LED_On( LED_Indicator );   // Set output pin showing Interrupt.

 	Channel1 = ADC1->DR - 2048;  // Pull Data from ADC, channel 1,
  Channel2 = ADC2->DR - 2048;  // channel 2.

	// Filtering could be added here to change Channel1 and Channel2
  //Channel1 = (int) IIR_Implement2((float)Channel1); 
// Compute output using Transposed 
	DAC_Data_Send(  Channel1 + 2048, Channel2 + 2048 );

	LED_Off( LED_Indicator ); // Turn off Interrupt Flag.
  
} // End of ADC_IRQHandler

// init_adc Function – Sets up ADC, Dac and Timer.
void Adc_Dac_Init( )
{
   RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN;  // Clock enabled for ADC1. 
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable GPIOA clock  
                  	// Enable DAC and Timer 2, 5 clock  
                	   
   GPIOA->MODER |= (0x00000ff0);        // PA.2-5 as Analog              
	
	 // Set CCR for ADC 
	 ADC->CCR |= 0x030f02; // Maximum Prescaler and Max delay between samples
	                       // Dual simultaneous mode.

	 // Set CR1 for ADC 1 and 2 
	 ADC1->CR1 |= 0x000820; // Enable Discontinuous and EOC interrupt. 
   ADC2->CR1 |= 0x000800; // Discontinuous 
	
	 // Set CR2 for ADC 1 and 2
	 ADC1->CR2 |= 0x16000401; // Set Trigger to Tim2, enable EOCS and turn on ADC1 
   ADC2->CR2 |= 0x00000001; // Turn on ADC2

   // Set SMPR2 for ADC 1 and 2 
	 ADC1->SMPR2 |= 0x000000f8; // Set sampling to maximum 
   ADC2->SMPR2 |= 0x000000f8; // Set sampling to maximum 
   
   // Set SQR1 for ADC1 
	 ADC1->SQR1 = 0x00000000; // Set for 1 Conversions  
   ADC2->SQR1 = 0x00000000; // Set for 1 Conversions  

   // Set SQR3 for ADC1 
	 ADC1->SQR3 = 0x00000002; // Set Initial sequence to channels 1 then 2  
	 ADC2->SQR3 = 0x00000003; // Set Initial sequence to channels 1 then 2  
   
   NVIC_EnableIRQ( ADC_IRQn );  // Enable ADC interrupts.

   // Enable both DAC's and set them to trigger on Tmr4.  
   RCC->APB1ENR	|= RCC_APB1ENR_DACEN;  
   DAC->CR |=   DAC_CR_EN1   
	            | DAC_CR_EN2;  
	 
   RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;  

   RCC->APB1ENR	 |= RCC_APB1ENR_TIM2EN;  

}

// This macro will start the adc, using the software trigger.
#define Fire_Adc() (ADC1->CR2 |= ADC_CR2_SWSTART)

