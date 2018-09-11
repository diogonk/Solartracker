// motor_input.c
// Desenvolvido para a placa EK-TM4C1294XL
// Recebe e armazena dados do teclado que alteram o funcionamento do motor DC.
// Marcelo Fernandes e Bruno Colombo

#define LIB_LCD 2

#include <stdint.h>
#include "dc_motor.h"

volatile uint32_t motor_speed = 0;
volatile uint32_t motor_direction = 0;


void Motor_Process(int input)
{
    if(input == 0)
    {
        motor_speed = 0;
        Motor_DisplayStopped();
        return;
    }
    else if (input >= 1 && input <= 7)
    {
        motor_speed = 30 + (input*10);
        Motor_DisplayRunning();
        return;
    }
    else if (input == '*')
    {
        motor_direction = 0;
        Motor_DisplayRunning();
        return;
    }
    else if (input == '#')
    {
        motor_direction = 1;
        Motor_DisplayRunning();
        return;
    }
}

void Motor_Control(void) 
{
    uint8_t motor_output = 0x00;
    
    if(motor_speed) 
    {
        //Coloca 1 no pino 1 ou 2 dependendo da direção
        motor_output |= 1 << (1 + motor_direction);

        //Coloca PWM no pino 2 ou 1 dependendo da direção
        if(pwm_bit)
            motor_output |= 1 << (2 - motor_direction);
    }

    PortF_Output_Dc_Motor(motor_output);
}

uint8_t motor_strings[][17] = 
{
    {"MOTOR PARADO    "},
    {"VELOCIDADE: XXX%"},
    {"ROTACAO: X      "},
    {"                "}
};

void Motor_DisplayStopped(void) {
    LCD_PositionCursor(0, 0);
    LCD_PushCustomString(0, ((uint32_t)motor_strings));
    
    LCD_PositionCursor(1, 0);
    LCD_PushCustomString(3, ((uint32_t)motor_strings));
}

void Motor_DisplayRunning(void) {

    if(motor_speed > 0) 
    {
        LCD_PositionCursor(0, 0);
        LCD_PushCustomString(1, ((uint32_t)motor_strings));
        
        LCD_PositionCursor(0, 12);
        
        //Se o 3o dígito (motor_speed/100) for 1, manda '1'; senão manda um espaço vazio
        LCD_PushChar(((motor_speed / 100) == 1) ? '1' : ' ');

        //Manda o 2o dígito ((motor_speed / 10) % 10) em ascii (+48)
        LCD_PushChar(((motor_speed / 10) % 10) + 48);

        //Manda o 3o dígito (motor_speed % 10) em ascii (+48)
        LCD_PushChar((motor_speed % 10) + 48);
    }
    
    LCD_PositionCursor(1, 0);
    LCD_PushCustomString(2, ((uint32_t)motor_strings));
    
    //Escolhe entre o caractere "->" ou o próximo "<-" se motor_direction for 1
    LCD_PositionCursor(1, 9);
    LCD_PushChar(0x7E + motor_direction);
}

