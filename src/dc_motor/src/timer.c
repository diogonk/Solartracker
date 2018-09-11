#include "timer.h"

volatile uint32_t   timer_counter = 0;
volatile uint32_t   pwm_counter = 0;
volatile uint8_t    pwm_bit = 0;
volatile uint32_t   keyboard_counter = 0;

void Timer_Init(void) 
{
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0;

    while((SYSCTL_RCGCTIMER_R & SYSCTL_RCGCTIMER_R0) != SYSCTL_RCGCTIMER_R0) {};

    TIMER0_CTL_R    &= ~0x01;

    TIMER0_CFG_R    = 0x00;
    
    TIMER0_TAMR_R   &= ~0x03;
    TIMER0_TAMR_R   |= 0x02;

    //TIMER0_TAILR_R  = 0x00013880; //1ms

    //TIMER0_TAILR_R  = 0x00013880; //1ms

    TIMER0_TAILR_R  = 0x00003E80;   //200us

    TIMER0_ICR_R    |= 0x01;

    TIMER0_IMR_R    |= 0x01;

    NVIC_PRI4_R     &= ~0xE0000000;

    NVIC_EN0_R      |= 0x80000;

    TIMER0_CTL_R    |= 0x01;
}

void Timer0A_Handler(void) 
{
    TIMER0_ICR_R |= 0x01;

    //Incrementa pwm_counter e coloca pra zero se passar de 10 (2ms/500hz)
    if(++pwm_counter >= 10)
        pwm_counter = 0;

    //Coloca o bit pwm como 0 ou 1 dependendo se a contagem 
    //for maior do que a velocidade atual do motor desejada
    pwm_bit = (pwm_counter >= (motor_speed/10));

    keyboard_counter++;

    Motor_Control();
}
