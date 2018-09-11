#if LIB_UTILS != 2
      #warning "Versão do arquivo LCD errada! Verifique"
#endif

#include <stdint.h>

void SysTick_Wait1us(int time_in_us);

void SysTick_Wait1ms(int time_in_ms);
