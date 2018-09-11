// gpio.c
// Desenvolvido para a placa EK-TM4C1294XL
// Inicializa as portas J e N
// Prof. Guilherme Peron


#include <stdint.h>

#include "tm4c1294ncpdt.h"

#define GPIO_PORT_A 0x0001
#define GPIO_PORT_B 0x0002
#define GPIO_PORT_C 0x0004
#define GPIO_PORT_D 0x0008
#define GPIO_PORT_E 0x0010
#define GPIO_PORT_F 0x0020
#define GPIO_PORT_G 0x0040
#define GPIO_PORT_H 0x0080
#define GPIO_PORT_J 0x0100
#define GPIO_PORT_K 0x0200
#define GPIO_PORT_L 0x0400
#define GPIO_PORT_M 0x0800
#define GPIO_PORT_N 0x1000

#define GPIO_PORTS (GPIO_PORT_D|GPIO_PORT_F|GPIO_PORT_J|\
					GPIO_PORT_K|GPIO_PORT_M|GPIO_PORT_N)

#define GPIO_PORTD_KB_ROWS 		(*((volatile uint32_t *)0x4005B03C))
#define GPIO_PORTF_DC_MOTOR		(*((volatile uint32_t *)0x4005D018))
#define GPIO_PORTJ_BOTOES 		(*((volatile uint32_t *)0x4006003C))
#define GPIO_PORTK_LCD_DATA		(*((volatile uint32_t *)0x400613FC))
#define GPIO_PORTM_LCD_CTRL 	(*((volatile uint32_t *)0x4006301C))
#define GPIO_PORTM_KB_COLUMNS 	(*((volatile uint32_t *)0x400631E0))
#define GPIO_PORTN_LEDS   		(*((volatile uint32_t *)0x4006403C))


// -------------------------------------------------------------------------------
// Função GPIO_Init
// Inicializa os ports J e N
// Parâmetro de entrada: Não tem
// Parâmetro de saída: Não tem
void GPIO_Init(void)
{
	//1a. Ativar o clock para a porta setando o bit correspondente no registrador RCGCGPIO
	SYSCTL_RCGCGPIO_R |= GPIO_PORTS;

	//1b.   após isso verificar no PRGPIO se a porta está pronta para uso.
  	while((SYSCTL_PRGPIO_R & (GPIO_PORTS) ) != (GPIO_PORTS) ){};
	
	// 2. Destravar a porta somente se for o pino PD7 e PE7
		
	// 3. Limpar o AMSEL para desabilitar a analógica
	GPIO_PORTD_AHB_AMSEL_R	= 0x00;
	GPIO_PORTF_AHB_AMSEL_R	= 0x00;
	GPIO_PORTJ_AHB_AMSEL_R 	= 0x00;
	GPIO_PORTK_AMSEL_R 		= 0x00;
	GPIO_PORTM_AMSEL_R		= 0x00;
	GPIO_PORTN_AMSEL_R 		= 0x00;
		
	// 4. Limpar PCTL para selecionar o GPI
	GPIO_PORTD_AHB_PCTL_R	= 0x00;
	GPIO_PORTF_AHB_PCTL_R	= 0x00;
	GPIO_PORTK_PCTL_R		= 0x00;
	GPIO_PORTJ_AHB_PCTL_R 	= 0x00;
	GPIO_PORTK_PCTL_R		= 0x00;
	GPIO_PORTM_PCTL_R		= 0x00;
	GPIO_PORTN_PCTL_R 		= 0x00;

	// 5. DIR para 0 se for entrada, 1 se for saída
	GPIO_PORTD_AHB_DIR_R	= 0x00;
	GPIO_PORTF_AHB_DIR_R	= 0x06;
	GPIO_PORTJ_AHB_DIR_R 	= 0x00;
	GPIO_PORTK_DIR_R		= 0xFF;
	GPIO_PORTM_DIR_R		= 0x7F;
	GPIO_PORTN_DIR_R 		= 0x03;
		
	// 6. Limpar os bits AFSEL para 0 para selecionar GPIO sem função alternativa
	GPIO_PORTD_AHB_AFSEL_R	= 0x00;
	GPIO_PORTF_AHB_AFSEL_R	= 0x00;
	GPIO_PORTJ_AHB_AFSEL_R 	= 0x00;
	GPIO_PORTK_AFSEL_R		= 0x00;
	GPIO_PORTM_AFSEL_R		= 0x00;
	GPIO_PORTN_AFSEL_R 		= 0x00;

		
	// 7. Setar os bits de DEN para habilitar I/O digital
	GPIO_PORTD_AHB_DEN_R	= 0x0F;
	GPIO_PORTF_AHB_DEN_R	= 0x06;
	GPIO_PORTJ_AHB_DEN_R 	= 0x03;
	GPIO_PORTK_DEN_R		= 0xFF;
	GPIO_PORTM_DEN_R		= 0x7F; 
	GPIO_PORTN_DEN_R 		= 0x03;

	
	// 8. Habilitar resistor de pull-up interno, setar PUR para 1
	GPIO_PORTJ_AHB_PUR_R 	= 0x03;
	GPIO_PORTD_AHB_PUR_R	= 0x0F;

}	

uint32_t PortD_Input(void)
{
	return GPIO_PORTD_KB_ROWS;
}
// -------------------------------------------------------------------------------
// Função PortJ_Input
// Lê os valores de entrada do port J
// Parâmetro de entrada: Não tem
// Parâmetro de saída: o valor da leitura do port
uint32_t PortJ_Input(void)
{
	return GPIO_PORTJ_BOTOES;
}


void PortF_Output_Dc_Motor(uint32_t valor)
{
	GPIO_PORTF_DC_MOTOR = valor;
}

void PortK_Output(uint32_t valor)
{
	GPIO_PORTK_LCD_DATA = valor;
}

void PortM_Output(uint32_t valor)
{
	GPIO_PORTM_LCD_CTRL = valor;
}

void PortM_OutputKeyboard(uint32_t valor)
{
	GPIO_PORTM_KB_COLUMNS = valor;
}

// -------------------------------------------------------------------------------
// Função PortN_Output
// Escreve os valores no port N
// Parâmetro de entrada: Valor a ser escrito
// Parâmetro de saída: não tem
void PortN_Output(uint32_t valor)
{
	//Ponteiro para o valor dos bits com leitura amigável
	GPIO_PORTN_LEDS = valor;
}




