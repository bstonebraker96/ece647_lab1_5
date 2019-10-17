/*----------------------------------------------------------------------------
 * Name:    Blinky.c
 * Purpose: LED Flasher
 * Note(s): 
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2011 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stm32f4xx.h>
#include "LED.h"
#include "UsartDriver.h"
#include "string.h"
#include "AdcDriver.h"

enum RATE 
{
  OneHz,
  HalfHz
} e_toggleRate = OneHz;

enum LEDSTATUS
{
  On,
	Off
} LED_One = Off;
enum BUTTONPRESS
{
	Pressed,
	Released,
	Unheld
} e_button1 = Unheld;

volatile uint32_t msTicks;                      /* counts 1ms timeTicks       */
uint32_t old_msTicks = 0;
uint8_t TimerFired = 0;
/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) {
  TimerFired = 1;
  msTicks++;
}

/*----------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *----------------------------------------------------------------------------*/
void Delay (uint32_t dlyTicks) {                                              
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
}


/*----------------------------------------------------------------------------
  Function that initializes Button pins
 *----------------------------------------------------------------------------*/
void BTN_Init(void) {

  RCC->AHB1ENR  |= ((1UL <<  0) );              /* Enable GPIOA clock         */

  GPIOA->MODER    &= ~((3UL << 2*0)  );         /* PA.0 is input              */
  GPIOA->OSPEEDR  &= ~((3UL << 2*0)  );         /* PA.0 is 50MHz Fast Speed   */
  GPIOA->OSPEEDR  |=  ((2UL << 2*0)  ); 
  GPIOA->PUPDR    &= ~((3UL << 2*0)  );         /* PA.0 is no Pull up         */
}

/*----------------------------------------------------------------------------
  Function that read Button pins
 *----------------------------------------------------------------------------*/
uint32_t BTN_Get(void) {

 return (GPIOA->IDR & (1UL << 0));
}

void WaitOnTimer(void) {
  while(!TimerFired);
  TimerFired = 0;
  return;
}

void UARTSendChangeStatus(void) 
{
	Usart_Send_String("Change Rate\n\r");
}
	
/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
  //int32_t num = -1; 
  //int32_t dir =  1;
  uint32_t btns = 0;
	uint8_t button_debounce = 0;
  SystemCoreClockUpdate();                      /* Get Core Clock Frequency   */
  if (SysTick_Config(SystemCoreClock / 10000)) { /* SysTick 100 usec interrupts  */
    while (1);                                  /* Capture error              */
  }

  LED_Init();
  BTN_Init();
  Usart_Init(SystemCoreClock, 9600);
	LED_Off(1);
	LED_Off(2);
	LED_Off(3);
	LED_Off(4);
	//uint16_t bigboy = 0;
	Adc_Dac_Init();//Setup ADC and DAC
  while(1) 
  {                                    /* Loop forever               */

		Fire_Adc();

    WaitOnTimer();
    

  }//end of superloop
  
}

