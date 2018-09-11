#if LIB_KB != 2
      #warning "Versão do arquivo teclado errada! Verifique"
#endif

void Keyboard_Init(void);

uint8_t Keyboard_Poll(void);

