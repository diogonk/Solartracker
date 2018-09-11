#include <stdint.h>

void LCD_Init(void);

void LCD_PushString(uint32_t posicao);

void LCD_PushCustomString(uint32_t posicao, uint32_t endereco);

void LCD_PositionCursor(uint8_t line, uint8_t column);

void LCD_PushConfig(uint32_t data);

void LCD_PushChar(uint32_t character);
