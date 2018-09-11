// uart.c
// Desenvolvido para a placa EK-TM4C1294XL
// Marcelo Fernandes e Bruno Colombo

#include "uart.h"


void Uart_Init(void)
{
	                                        // activate clock for Port A
		SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
    //Habilita clock no módulo UART0
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;

    //Espera até a UART0 estar pronta
    while((SYSCTL_RCGCUART_R & SYSCTL_RCGCUART_R0) != SYSCTL_RCGCUART_R0) {}

    //Desabilita UART0 antes de fazer alterações
    UART0_CTL_R &= ~UART_CTL_UARTEN;

    //BRD  = BRDI + BRDF = UARTSysClk / (ClkDiv * Baud Rate)
    //9600 = BRDI + BRDF = 80e6 / (16 * 9600) = 520,833...
    //BRDI = 520; BRDF = 2^6 * 0,833... = 53,3 ~= 53
    UART0_IBRD_R = 520;
    UART0_FBRD_R = 53;
    UART0_LCRH_R = UART_LCRH_WLEN_8;
    UART0_CC_R = UART_CC_CS_SYSCLK;
    UART0_CTL_R |= (UART_CTL_UARTEN|UART_CTL_RXE|UART_CTL_TXE);
		GPIO_PORTA_AFSEL_R |= 0x03;           // habilita funcao anternativa PA1 E PA0
		GPIO_PORTA_DEN_R |= 0x03;             // habilita DIGITAL I/O EM PA1 E PA0
																					// CONFIG PA0-1 COMO UART
		GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFFFFFF00)+0x00000011;
		GPIO_PORTA_AMSEL_R &= ~0x03;          // DESABILITA FUNCAO ANALOGICA
}

void Uart_Transmit(uint32_t value)
{
	while((UART0_FR_R&UART_FR_TXFF) != 0); //VERIFICA SE PILHAS DE ENVIO ESTA VAZIA
	//uint32_t value;
	//value = le_adc();
	UART0_DR_R |= value;//COPIA PARA PILHA DE ENVIO DA UART
}
