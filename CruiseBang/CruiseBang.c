// DC Motor Control using PWM with RPM Display
// Nazim Wali
// Embedded Systems Hands-On Exercise #3: RPM Shaft Encoder
// DUE: 4/25/21
 
#include "stm32f4xx.h"
#include <stdio.h>

#define RS 0x20     /* PA5 mask for reg select */
#define EN 0x80     /* PA7 mask for enable */

void delayMs(int n);
void PWM_init(void);					//PWM initialization
void LCD_nibble_write(char data, unsigned char control);
void LCD_command(unsigned char command);
void LCD_data(char data);
void LCD_init(void);
void PORTS_init(void);
void Freq_init(void);
void ADC_init(void);
void Cruise_init(void);

int main (void) {
    
		//result from ADC fed to PWM current driver
		int result;
		
		//variables used to determine and display RPMs
		int last;
    int current;
		float rpm;	
		float period;
		float frequency;
		char conv[7];
	
		//variables used to determine and display the RPMs during Cruise Mode
		int last2;
		int current2;
		float actual;
		float period2;
		float frequency2;
		char actualdisp[7];
		
		int first = 1;
		float desired = 0;									//Used to determine power supplied to motor during automated speed control 
		char cruisemode[7];									//Used for debugging; displayed the power being supplied by PWM
		
		ADC_init();
		Freq_init();												//initialize ports for RPM Timer/Counter
		PWM_init();													//initialize ports for PWM
		LCD_init();													//initialize ports for LCD
		Cruise_init();
		
		while(1){
				
				LCD_data('C');												//Print "Cruise:" onto LCD
				LCD_data('r');	
				LCD_data('u');
				LCD_data('i');
				LCD_data('s');
				LCD_data('e');
				LCD_data(':');
			
				if(GPIOC->IDR & 0x00000001){					//If Switch is off, display Cruise Mode: "Off"
						LCD_data('O');
						LCD_data('f');
						LCD_data('f');
			  }
				else{																	//Else, display Cruise: "ON"
						LCD_data('O');
						LCD_data('n');

						//This bit of code was used to debug and verify the automated speed control of the Cruise control
						/*
						sprintf(cruisemode, "%f", desired);
				
						for(int i = 0; i <4; i++){
								LCD_data(cruisemode[i]);
						}
						*/
				}
			
				LCD_command(0xC0);										//Start print to second line of LCD
			
				LCD_data('R');												//Display "RPMs:"
				LCD_data('P');
				LCD_data('M');
				LCD_data('s');
				LCD_data(':');		
					
				//If Switch is Off (Cruse Mode OFF), then read/update result from ADC
				//first initially is set to 1, so for the first iteration, the ADC conversion and result update will always execute
				if(first | GPIOC->IDR & 0x00000001){
					
						ADC1->CR2 |= 0x40000000;        /* start a conversion */
						while(!(ADC1->SR & 2)) {}       /* wait for conv complete */
						result = ADC1->DR;              /* read conversion result */
						desired = result;								//save result in a variable used for automated speed control
			  }
				
				//first is set to 0, so execution of above loop is dependent solely on Cruise mode toggling
				first = 0;
				
				//If Cruise Mode is OFF, measure, determine, and display the RPMs
				if((GPIOC->IDR & 0x00000001)){
					
						TIM8->CCR1 = result;				//feed result from ADC to PWM to supply voltage to motor
						delayMs(50);
					
						while (!(TIM3->SR & 2)) {}  /* wait until initial input edge is captured */	
						
						last = TIM3->CCR1; 					//read initial captured counter value
							
						while (!(TIM3->SR & 2)) {}  /* wait until next input edge is captured */
						
						current = TIM3->CCR1;       /* read next captured counter value */
						period = current - last;    /* calculate the period */

						frequency = 100000.0f /(period*20);			//calculate frequency
						rpm = frequency*60;											//calculate RPMs
						
						sprintf(conv, "%f", rpm);								//Convert RPMs to Char array
						
						for(int i = 0; i <4; i++){							//Display contents of array
								LCD_data(conv[i]);
						}
				
				}
				else{//else begin new rpms measruement and initiate feedback for automated speed control
				
							//A new RPMs is measured called 'actual' that is compared to the variable 'rpm', which in Cruise mode is used as a setpoint
							//The variable 'rpm' from above is not updated when Cruise Mode is on. Only the variable 'actual' is updated here
							while (!(TIM3->SR & 2)) {}  /* wait until initial input edge is captured */	
				
							last2 = TIM3->CCR1; 				//read initial captured counter value
								
							while (!(TIM3->SR & 2)) {}  /* wait until next input edge is captured */
							
							current2 = TIM3->CCR1;       /* read next captured counter value */
							period2 = current2 - last2;    /* calculate the period */

							frequency2 = 100000.0f /(period2*20);			//calculate frequency
							actual = frequency2*60;										//calculate RPMs
							
							sprintf(actualdisp, "%f", actual);				
							
							for(int i = 0; i <4; i++){								//display the 'actual' rpms
									LCD_data(actualdisp[i]);
							}
					
							//compare 'actual' to the setpoint 'rpms' to see whether the power to the motor should increase/decrease
							if(actual < (rpm-100)){
										TIM8->CCR1 = desired+=5;
							}
							else if(actual > (rpm+100)){
										TIM8->CCR1 = desired-=5;
							}
							
							//Bit of Code used to debug and verify the automated speed control of the Cruise Mode
							//This displays the setpoint 'rpm' next to the 'actual' rpms to see how the controller tries to maintain the setpoint rpms
							/*
							LCD_data(' ');
							
							sprintf(conv, "%f", rpm);
							for(int i = 0; i <4; i++){
								LCD_data(conv[i]);
							}
							*/
							
				}
				
				delayMs(300);
				LCD_command(1);

		}

}

void ADC_init(){
		/* set up pin PA1 for analog input */
    RCC->AHB1ENR |=  1;	            /* enable GPIOA clock */
    GPIOA->MODER |=  0xC;           /* PA1 analog */

    /* setup ADC1 */
    RCC->APB2ENR |= 0x00000100;     /* enable ADC1 clock */
    ADC1->CR2 = 0;                  /* SW trigger */
    ADC1->SQR3 = 1;                 /* conversion sequence starts at ch 1 */
    ADC1->SQR1 = 0;                 /* conversion sequence length 1 */
    ADC1->CR2 |= 1;                 /* enable ADC1 */
}

		//////////////////////////////////
		// PWM port initialization call //
		//////////////////////////////////

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

void Freq_init(void){

		// configure PA6 as input of TIM3 CH1
    RCC->AHB1ENR |=  1;             /* enable GPIOA clock */
    GPIOA->MODER &= ~0x00003000;    /* clear pin mode */
    GPIOA->MODER |=  0x00002000;    /* set pin to alternate function */
    GPIOA->AFR[0] &= ~0x0F000000;   /* clear pin AF bits */
    GPIOA->AFR[0] |= 0x02000000;    /* set pin to AF2 for TIM3 CH1 */

    // configure TIM3 to do input capture with prescaler ...
    RCC->APB1ENR |= 2;              /* enable TIM3 clock */
    TIM3->PSC = 160 - 1;          /* divided by 16000 */
    TIM3->CCMR1 = 0x41;             /* set CH1 to capture at every edge */
    TIM3->CCER = 1;                 /* enable CH 1 capture rising edge */
    TIM3->CR1 = 1;                  /* enable TIM3 */

}

		//////////////////////

void Cruise_init(void){
		RCC->AHB1ENR |= 0x04;
		
		GPIOC->MODER &= ~0x00000003;
		GPIOC->MODER |= 0x00000000;
		GPIOC->PUPDR = 0x00000001;

}

/* initialize GPIOB/C then initialize LCD controller */
void LCD_init(void) {
    PORTS_init();

    delayMs(20);                /* LCD controller reset sequence */
    LCD_nibble_write(0x30, 0);
    delayMs(5);
    LCD_nibble_write(0x30, 0);
    delayMs(1);
    LCD_nibble_write(0x30, 0);
    delayMs(1);

    LCD_nibble_write(0x20, 0);  /* use 4-bit data mode */
    delayMs(1);
    LCD_command(0x28);          /* set 4-bit data, 2-line, 5x7 font */
    LCD_command(0x06);          /* move cursor right */
    LCD_command(0x01);          /* clear screen, move cursor to home */
    LCD_command(0x0F);          /* turn on display, cursor blinking */
}

void PORTS_init(void) {
    RCC->AHB1ENR |=  0x06;          /* enable GPIOB/C clock */
		RCC->AHB1ENR |=  1;             /* enable GPIOA clock */

    /* PORTC 5 for LCD R/S */
    /* PORTC 7 for LCD EN */
    GPIOC->MODER &= ~0x0000CC00;    /* clear pin mode */
    GPIOC->MODER |=  0x00004400;    /* set pin output mode */
    GPIOC->BSRR = 0x00800000;       /* turn off EN */

    /* PB4-PB7 for LCD D4-D7, respectively. */
    GPIOB->MODER &= ~0x0000FF00;    /* clear pin mode */
    GPIOB->MODER |=  0x00005500;    /* set pin output mode */
}

void LCD_nibble_write(char data, unsigned char control) {
    /* populate data bits */
    GPIOB->BSRR = 0x00F00000;       /* clear data bits */
    GPIOB->BSRR = data & 0xF0;      /* set data bits */

    /* set R/S bit */
    if (control & RS)
        GPIOC->BSRR = RS;
    else
        GPIOC->BSRR = RS << 16;

    /* pulse E */
    GPIOC->BSRR = EN;
    delayMs(0);
    GPIOC->BSRR = EN << 16;
}

void LCD_command(unsigned char command) {
    LCD_nibble_write(command & 0xF0, 0);    /* upper nibble first */
    LCD_nibble_write(command << 4, 0);      /* then lower nibble */

    if (command < 4)
        delayMs(2);         /* command 1 and 2 needs up to 1.64ms */
    else
        delayMs(1);         /* all others 40 us */
}

void LCD_data(char data) {
    LCD_nibble_write(data & 0xF0, RS);      /* upper nibble first */
    LCD_nibble_write(data << 4, RS);        /* then lower nibble */

    delayMs(1);
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
