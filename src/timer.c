#include "timer.h"
#include "var_global.h"

volatile uint32_t keyboard_counter = 0;
volatile uint8_t fuck = 0;
volatile uint8_t count_tx = 0;


// -----------------------------------------------------------------------------
// Função Timer_Init
//--------------------------------
// Inicializa Timer 0 no modo A (32 bits) e com interrupção a cada 200 us.
// -----------------------------------------------------------------------------
void Timer_Init(void)
{
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0;

    while ((SYSCTL_RCGCTIMER_R & SYSCTL_RCGCTIMER_R0) != SYSCTL_RCGCTIMER_R0){};

    TIMER0_CTL_R &= ~0x01;
    TIMER0_CFG_R = 0x00;
    TIMER0_TAMR_R &= ~0x03;
    TIMER0_TAMR_R |= 0x02;
    TIMER0_TAILR_R = 0x00003E80; //=200us * 80e6hz
    TIMER0_ICR_R |= 0x01;
    TIMER0_IMR_R |= 0x01;
    NVIC_PRI4_R &= ~0xE0000000;
    NVIC_EN0_R |= 0x80000;
    TIMER0_CTL_R |= 0x01;
}

// -----------------------------------------------------------------------------
// Função Timer0A_Handler
//--------------------------------
// Interrupção gerada pelo Timer 0
// -----------------------------------------------------------------------------
void Timer0A_Handler(void)
{
		TIMER0_ICR_R |= 0x01;
		if(fuck){
			if (count_tx>50){Uart_Transmit();count_tx=0;}
			else{count_tx++;}
		}
		keyboard_counter++;
}
