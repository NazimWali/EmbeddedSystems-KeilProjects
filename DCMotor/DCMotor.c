// Program to control DC Motor using DAC, SPI, and PWM
// Nazim Wali
// Embedded Systems Hands-On Exercise #2: DC Motor Control
// DUE: 3/26/21
 
#include "stm32f4xx.h"
#include <stdio.h>

void delayMs(int n);
void OB_init(void);						//Onboard DAC initialization
void SPI1_init(void);					//SPI1 DAC initialization
void DAC_write(short data);
void PWM_init(void);					//PWM initialization

int main (void) {
    int result;
		int OB  = 0;							//Control Variables to
		int SPI = 0;							//Choose which loop to execute
		int PWM = 1;

    /* set up pin PA1 for analog input */
    RCC->AHB1ENR |=  1;	            /* enable GPIOA clock */
    GPIOA->MODER |=  0xC;           /* PA1 analog */

    /* setup ADC1 */
    RCC->APB2ENR |= 0x00000100;     /* enable ADC1 clock */
    ADC1->CR2 = 0;                  /* SW trigger */
    ADC1->SQR3 = 1;                 /* conversion sequence starts at ch 1 */
    ADC1->SQR1 = 0;                 /* conversion sequence length 1 */
    ADC1->CR2 |= 1;                 /* enable ADC1 */
	
		////////////////////////////////////////////
		// Loop to execute when using onboard DAC //
		////////////////////////////////////////////
	
    while ((OB == 1) && (SPI == 0) && (PWM == 0)) {
				OB_init();											//initalize ports for onboard DAC
			
				while(1){
						ADC1->CR2 |= 0x40000000;        /* start a conversion */
						while(!(ADC1->SR & 2)) {}       /* wait for conv complete */
						result = ADC1->DR;              /* read conversion result */
						DAC->DHR12R1 = result;					
				}
					
    }
		
		////////////////////////////////////////
		// Loop to execute when using SPI DAC //
		////////////////////////////////////////
		
		while ((OB == 0) && (SPI == 1) && (PWM == 0)){
				SPI1_init();											//initialize ports for SPI DAC
				
				while(1){
						ADC1->CR2 |= 0x40000000;        /* start a conversion */
						while(!(ADC1->SR & 2)) {}       /* wait for conv complete */
						result = ADC1->DR;              /* read conversion result */			
						
						short i = result;
						DAC_write(i);      /* write the letter through SPI1 */
				}
		
		}
		
		////////////////////////////////////
		// Loop to execute when using PWM //
		////////////////////////////////////
		
		while ((OB == 0) && (SPI == 0) && (PWM == 1)){
				PWM_init();													//initialize ports for PWM
			
				while(1){
						ADC1->CR2 |= 0x40000000;        /* start a conversion */
						while(!(ADC1->SR & 2)) {}       /* wait for conv complete */
						result = ADC1->DR;              /* read conversion result */
					
						TIM8->CCR1 = result;
						delayMs(50);
				}
		}
}

		//////////////////////
		//	Onboard DAC port intialization call

void OB_init(){
		GPIOA->MODER |=  0x00000300;    /* PA4 analog */
    /* setup DAC */
    RCC->APB1ENR |= 1 << 29;        /* enable DAC clock */
    DAC->CR |= 1;                   /* enable DAC */
}

		//////////////////////
		// SPI DAC port initialization call

/* enable SPI1 and associated GPIO pins */
void SPI1_init(void) {
    RCC->AHB1ENR |= 1;          /* enable GPIOA clock */
    RCC->APB2ENR |= 0x1000;     /* enable SPI1 clock */

    /* PORTA 5, 7 for SPI1 MOSI and SCLK */
    GPIOA->MODER &= ~0x0000CC00;    /* clear pin mode */
    GPIOA->MODER |=  0x00008800;    /* set pin alternate mode */
    GPIOA->AFR[0] &= ~0xF0F00000;   /* clear alt mode */
    GPIOA->AFR[0] |=  0x50500000;   /* set alt mode SPI1 */

    /* PORTA4 as GPIO output for SPI slave select */
    GPIOA->MODER &= ~0x00000300;    /* clear pin mode */
    GPIOA->MODER |=  0x00000100;    /* set pin output mode */
    
    SPI1->CR1 = 0x31C;		   /* set the Baud rate, 8-bit data frame */
    SPI1->CR2 = 0;
    SPI1->CR1 |= 0x40;              /* enable SPI1 module */
}

/* This function enables slave select, writes one byte to SPI1,
   wait for transmit complete and deassert slave select. */
void DAC_write(short data) {
    while (!(SPI1->SR & 2)) {}      /* wait until Transfer buffer Empty */
    GPIOA->BSRR = 0x00100000;       /* deassert slave select */
    SPI1->DR = 0x9000 | ((data << 2) & 0x0FFF);  /* write command and data */
    while (!(SPI1->SR & 2)) {}      /* wait until Transfer buffer Empty */
    while (SPI1->SR & 0x80) {}      /* wait for transmission done */
    GPIOA->BSRR = 0x00000010;       /* assert slave select */
}

		//////////////////////
		// PWM port initialization call

void PWM_init(){
		RCC->AHB1ENR |= 1;              /* enable GPIOA clock */
    
    GPIOA->AFR[0] |= 0x00300000;    /* PA5 pin for TIM8 */
    GPIOA->MODER |=  0x00000800;
    
    /* setup TIM8 */
    RCC->APB2ENR |= 2;              /* enable TIM8 clock */
    TIM8->PSC = 10 - 1;             /* divided by 10 */
    TIM8->ARR = 4000 - 1;          /* divided by 26667 */
    TIM8->CNT = 0;
    TIM8->CCMR1 = 0x0068;           /* PWM mode */
    TIM8->CCER = 4;                 /* enable PWM Ch1N */
    TIM8->BDTR |= 0x8000;           /* enable output */
    TIM8->CR1 = 1;                  /* enable timer */

}

		//////////////////////

/* 16 MHz SYSCLK */
void delayMs(int n) {
    int i;

    /* Configure SysTick */
    SysTick->LOAD = 16000;  /* reload with number of clocks per millisecond */
    SysTick->VAL = 0;       /* clear current value register */
    SysTick->CTRL = 0x5;    /* Enable the timer */

    for(i = 0; i < n; i++) {
        while((SysTick->CTRL & 0x10000) == 0) /* wait until the COUNTFLAG is set */
            { }
    }
    SysTick->CTRL = 0;      /* Stop the timer (Enable = 0) */
}
