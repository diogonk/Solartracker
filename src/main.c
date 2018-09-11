
#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "utils.h"
#include "adc.h"
#include "uart.h"

#define GPIO_PORTN1             (*((volatile uint32_t *)0x40064008))
#define GPIO_PORTK1             (*((volatile uint32_t *)0x40061008))
#define GPIO_PORTK0             (*((volatile uint32_t *)0x40061004))


void PLL_Init(void);
void SysTick_Init(void);


void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical(void);     // previous I bit, disable interrupts
void WaitForInterrupt(void);  // low power mode

uint32_t potentiometers[2];
uint32_t ldr_inputs[4];

int16_t servoh = 100;
int16_t servov = 100;

int16_t servoh_t = 100;
int16_t servov_t = 100;

uint16_t dutycicle_v = 0;
uint16_t dutycicle_h = 0;

uint32_t polling_time = 0;
uint32_t dif_tol = 0;

uint16_t pwm50hz = 4000;

// data returned from ADC0_InSeq2() by reference
// ADCvalue[0] is second result (default: ADC8 (PE5)) 0 to 4095
// ADCvalue[1] is first result  (default: ADC9 (PE4)) 0 to 4095

// This debug function initializes Timer0A to request interrupts
// at a 10 Hz frequency.  It is similar to FreqMeasure.c.
void Timer0A_PWM_init(void){
  DisableInterrupts();
  // **** general initialization ****
                                   // activate timer0
  SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0;
                                   // allow time for clock to stabilize
  while((SYSCTL_PRTIMER_R&SYSCTL_PRTIMER_R0) == 0){};
  TIMER0_CTL_R &= ~TIMER_CTL_TAEN; // disable timer0A during setup
  TIMER0_CTL_R &= ~TIMER_CTL_TBEN; // disable timer0A during setup
  TIMER0_CFG_R = TIMER_CFG_16_BIT; // configure for 16-bit timer mode
  TIMER0_CC_R &= ~TIMER_CC_ALTCLK; // timer0 clocked from system clock
  // **** timer0A initialization ****
                                   // configure for periodic mode
  TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
  TIMER0_TAPR_R = 0;             // prescale value for 10us
  TIMER0_TAILR_R = 399;          // start value for 10 Hz interrupts VECTOR 35, INT 19
  TIMER0_IMR_R |= TIMER_IMR_TATOIM;// enable timeout (rollover)  interrupt
  TIMER0_ICR_R = TIMER_ICR_TATOCINT;// clear timer0A timeout flag
  TIMER0_CTL_R |= TIMER_CTL_TAEN;  // enable timer0A 16-b, periodic, interrupts

  TIMER0_TBMR_R = TIMER_TAMR_TAMR_PERIOD;
  TIMER0_TBPR_R = 199;             // prescale value for 10us
  TIMER0_TBILR_R = 79;          // start value for 10 Hz interrupts VECTOR 36 INT 20
  TIMER0_IMR_R |= TIMER_IMR_TBTOIM;// enable timeout (rollover) interrupt
  TIMER0_ICR_R = TIMER_ICR_TBTOCINT;// clear timer0A timeout flag
  TIMER0_CTL_R |= TIMER_CTL_TBEN;  // enable timer0A 16-b, periodic, interrupts
  // **** interrupt initialization ****
                                   // Timer0A=priority 2
  NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0x40000000; // top 3 bits
  NVIC_PRI5_R = (NVIC_PRI5_R&~0xE0)|0x60; // PRI5[7:5] = PRIORITY 3 
  
  NVIC_EN0_R = 0x00180000;         // enable interrupt 19 AND 20 in NVIC
}

void Timer0B_Handler(void){
  TIMER0_ICR_R = TIMER_ICR_TBTOCINT;    // acknowledge timer0A timeout
  ADC0_InSeq2(potentiometers);                // takes 3.4 usec to run
	ADC0_InSeq1(ldr_inputs);                // takes 3.4 usec to run
  /*
  uint32_t rt = ldr_inputs[3] top right
  uint32_t lt = ldr_inputs[2] top left
  uint32_t rb = ldr_inputs[1] botton right
  uint32_t lb = ldr_inputs[0] botton left
  */

  uint32_t dtime = (potentiometers[0]+20) / 20; // read potentiometers
  int32_t tol = potentiometers[1] / 4;
  if (polling_time> dtime)
  {
    polling_time = 0;
    int32_t avt = (ldr_inputs[3] + ldr_inputs[2]) / 2; // average value top
    int32_t avd = (ldr_inputs[1] + ldr_inputs[0]) / 2; // average value down
    int32_t avl = (ldr_inputs[2] + ldr_inputs[0]) / 2; // average value left
    int32_t avr = (ldr_inputs[3] + ldr_inputs[1]) / 2; // average value right

    int32_t dvert = avt - avd; // check the difference beetwen up and down
    int32_t dhoriz = avl - avr;// check the difference beetwen left and rigt

    if (-1 * tol > dvert || dvert > tol) // check if the diffirence is in the tolerance else change vertical angle
    {
      if (avt > avd)
      {
        servov++;
        if (servov > 0x105)
        {
          servov = 0x105;
        }
      }
      else if (avt < avd)
      {
        servov--;
        if (servov < 0x10)
        {
          servov = 0x10;
        }
      }
    }
    if (-1 * tol > dhoriz || dhoriz > tol) // check if the diffirence is in the tolerance else change horizontal angle
    {
      if (avl > avr)
      {
        servoh--;
        if (servoh < 0x10)
        {
          servoh = 0x10;
        }
      }
      else if (avl < avr)
      {
        servoh++;
        if (servoh > 0x105)
        {
          servoh = 0x105;
        }
      }
    }
  }
  polling_time++;
}

void Timer0A_Handler(void){
  TIMER0_ICR_R = TIMER_ICR_TATOCINT;    // acknowledge timer0A timeout
  if (dutycicle_v < (100+servov)) {
		GPIO_PORTK0 |= 0x01;
	}
  else{ 
		GPIO_PORTK0 &= ~0x01;
	}
  if (dutycicle_h < (100+servoh)) {
		GPIO_PORTK1 |= 0x02;
	}
  else{ 
		GPIO_PORTK1 &= ~0x02;
	}
  dutycicle_v++;
  dutycicle_h++;
  if(dutycicle_v>pwm50hz){dutycicle_v = 0;}
  if(dutycicle_h>pwm50hz){dutycicle_h = 0;}
}




int main(void){
  
	PLL_Init();                      // 120 MHZ
																		// activate clock for Port N
	ADC0_Init();                      // initialize ADC0, software trigger, PE3/AIN0|PE2/AIN1|PE1/AIN2|PE0/AIN3|PD1/AIN14|PD0/AIN15
	Timer0A_PWM_init();               // set up Timer0A for 50Hz interrupts PWM
	
	Uart_Init();
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12 + SYSCTL_RCGCGPIO_R9;
                                   // allow time for clock to stabilize
  while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};
  while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R9) == 0){};

  GPIO_PORTN_DIR_R |= 0x02;         // make PN1 out (PN1 built-in LED1)
  GPIO_PORTN_AFSEL_R &= ~0x02;      // disable alt funct on PN1
  GPIO_PORTN_DEN_R |= 0x02;         // enable digital I/O on PN1
                                    // configure PN1 as GPIO
  GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF0F)+0x00000000;
  GPIO_PORTN_AMSEL_R &= ~0x02;     // disable analog functionality on PN1


  GPIO_PORTK_DIR_R |= 0x03;        // make PK1 AND PK0 out
  GPIO_PORTK_AFSEL_R &= ~0x03;     // disable alt funct on PK1 AND PK0
  GPIO_PORTK_DEN_R |= 0x03;        // enable digital I/O on PK1 AND PK0
                                   // configure PK1 AND PK0 as GPIO
  GPIO_PORTK_PCTL_R = (GPIO_PORTK_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTK_AMSEL_R &= ~0x03;     // disable analog functionality on PK1 AND PK0

  EnableInterrupts();
  while(1){
    WaitForInterrupt();
  }
}
