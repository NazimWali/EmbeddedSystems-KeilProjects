// Program to Determine Strain on Pipe Configuration
// Nazim Wali & Nick Deoki
// Senior Design I

#include "stm32f4xx.h"
#include <stdio.h>

#define RS 0x20     /* PA5 mask for reg select */
#define EN 0x80     /* PA7 mask for enable */

void delayMs(int n);
void LCD_nibble_write(char data, unsigned char control);
void LCD_command(unsigned char command);
void LCD_data(char data);
void LCD_init(void);
void PORTS_init(void);

int main (void) {
    int result;
		double voltage;
		double degr;
		char conv[4];
		char conv2[5];
		
		/* initialize LCD controller */
    LCD_init();

    /* set up pin PA1 for analog input */
    RCC->AHB1ENR |=  1;	            /* enable GPIOA clock */
    GPIOA->MODER |=  0xC;           /* PA1 analog */

    /* setup ADC1 */
    RCC->APB2ENR |= 0x00000100;     /* enable ADC1 clock */
    ADC1->CR2 = 0;                  /* SW trigger */
    ADC1->SQR3 = 1;                 /* conversion sequence starts at ch 1 */
    ADC1->SQR1 = 0;                 /* conversion sequence length 1 */
    ADC1->CR2 |= 1;                 /* enable ADC1 */
	
    while (1) {
        ADC1->CR2 |= 0x40000000;        /* start a conversion */
        while(!(ADC1->SR & 2)) {}       /* wait for conv complete */
        result = ADC1->DR;              /* read conversion result */
				
				voltage = (double)result/4095*3.3;		//Equation to converrt to Voltage units
				sprintf(conv, "%f", voltage);					//Use sprintf to store int results in a char array
				
				LCD_data('V');
				LCD_data('o');
				LCD_data('l');
				LCD_data('t');
				LCD_data('a');
				LCD_data('g');
				LCD_data('e');
				LCD_data(':');
				LCD_data(' ');
					
				for(int i = 0; i < 7; i++){						//For loop to output char array contents to LCD
					LCD_data(conv[i]);
					//delayMs(100);
				}
				
				degr = (double)(220)*(1/((3.3/voltage)-1));				//Equation for Degrees
				sprintf(conv2, "%f", degr);						//Store in char array
				
				LCD_command(0xC0);										//Start print to second line of LCD
				
				LCD_data('D');
				LCD_data('e');
				LCD_data('g');
				LCD_data('r');
				LCD_data('e');
				LCD_data('e');
				LCD_data('s');
				LCD_data(':');
				LCD_data(' ');
				
				for(int i = 0; i < 7; i++){						//Print contents of char array
					LCD_data(conv2[i]);
					//delayMs(100);
				}
				
				delayMs(1500);
					
        /* clear LCD display */
        LCD_command(1);
        //delayMs(1000);	
				
    }
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
	
		RCC->AHB1ENR |=  1;                	/* enable GPIOA clock */

    /* PORTA 5 for LCD R/S */
    /* PORTA 7 for LCD EN */
    GPIOA->MODER &= ~0x0000CC00;    /* clear pin mode */
    GPIOA->MODER |=  0x00004400;    /* set pin output mode */
    GPIOA->BSRR = 0x00800000;       /* turn off EN */

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
        GPIOA->BSRR = RS;
    else
        GPIOA->BSRR = RS << 16;

    /* pulse E */
    GPIOA->BSRR = EN;
    delayMs(0);
    GPIOA->BSRR = EN << 16;
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