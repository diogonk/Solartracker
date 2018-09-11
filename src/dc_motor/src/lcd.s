;------------Constantes------------
DELAY_SMALL EQU 100
DELAY_BIG   EQU 4
STRING_SIZE EQU 17

;------------Área de Código------------
;Tudo abaixo da diretiva a seguir será armazenado na memória de código

        AREA    |.text|, CODE, READONLY, ALIGN=2
        THUMB
            
        EXPORT  LCD_Init
        EXPORT  LCD_PushConfig
        EXPORT  LCD_PushString
        EXPORT  LCD_PushChar
        EXPORT  LCD_PushCustomString
        EXPORT  LCD_PositionCursor
         
        IMPORT  SysTick_Wait1ms
        IMPORT  SysTick_Wait1us
        IMPORT  PortJ_Input
        IMPORT  PortK_Output
        IMPORT  PortM_Output
                ;Strings sempre de 16 caracteres
stringArray =   "UTFPR           "      ,0,\
                "2018            "      ,0,\
                "BRUNO E MARCELO "      ,0,\
                "EQUIPE N",223," 8     ",0,\
                "STRING INVÁLIDA "      ,0   
               

        ALIGN
            
;------------LCD_Init------------
; Configura o sistema para utilizar o LCD.
; Tabela: Pág 24 - https://goo.gl/HmbeUC
; Entrada: Nenhum
; Saída: Nenhum
; Modifica: R0, R1
LCD_Init
    PUSH {R0, LR}
    
    MOV R0, #2_00000001     ;(0 0 0 0|0 0 0 R)
    BL  LCD_PushConfig
    MOV R0, #DELAY_BIG
    BL  SysTick_Wait1ms
    
    ;Entry Mode Set 
    MOV R0, #2_00000110     ;(0 0 0 0|0 1 I/D S)
    BL LCD_PushConfig
    
    ;Function Set 
    MOV R0, #2_00111000     ;(0 0 1 DL|N F - -)
    BL  LCD_PushConfig
    
    ;Display on/off control
    MOV R0, #2_00001100     ;(0 0 0 0|1 D C B)
    BL  LCD_PushConfig
    
    ;Return home            ;(0 0 0 0|0 0 1 -)
    MOV R0, #2_00000010
    BL  LCD_PushConfig
    
	MOV  R0, #50
	BL   SysTick_Wait1ms
    
    POP {R0, LR}
    BX  LR

;------------LCD_PushConfig------------
; Envia dados no modo config para o controlador do LCD.
; Entrada: R0 --> Dado a ser enviado
; Saída: Nenhum
; Modifica: R0, R1 (temporariamente), R2 (temporariamente)
LCD_PushConfig
    PUSH {R12, R1, R2, LR}           
    
    MOV R2, R0         
    MOV R0, #2_00000000
    BL  PortM_Output
    MOV R0, R2
    BL  PortK_Output
    MOV R0, #2_00000100
    BL  PortM_Output
    MOV R0, #DELAY_SMALL
    BL  SysTick_Wait1us
    MOV R0, #2_00000000
    BL  PortM_Output
    POP {R12, R1, R2, LR}
    BX  LR
    
    
LCD_PushString
;------------LCD_PushString------------
; Envia uma string para o LCD.
; Entrada: R0 --> Posição no vetor da string a ser enviada
; Saída: Nenhum
; Modifica: -- (apenas mudanças temporárias)
    PUSH {R12, R0, R4, R5, R6, LR}
    
    MOV R6, #STRING_SIZE        
    LDR R4, =stringArray
    MUL R5, R0, R6
    ADD R4, R5

stringCopy                      ;Copia a string byte a byte, colocando-os em R0 e mandando-os para o LCD.
    LDRB R0, [R4], #1           ;Termina o envio quando um '0' é detectado na string.
    CMP R0, #0
    BEQ stringCopy_end 
    BL  LCD_PushChar
    B   stringCopy
stringCopy_end    
    POP {R12, R0, R4, R5, R6, LR}
    BX  LR



LCD_PushCustomString
;------------LCD_PushString------------
; Envia uma string para o LCD.
; Entrada: R0 --> Posição no vetor da string a ser enviada
;          R1 --> Ponteiro para vetor de strings que deve ser lido
; Saída: Nenhum
; Modifica: -- (apenas mudanças temporárias)
    PUSH {R12, R0, R4, R5, R6, LR}
    
    MOV R6, #STRING_SIZE        
    MOV R4, R1
    MUL R5, R0, R6
    ADD R4, R5

customStringCopy                      ;Copia a string byte a byte, colocando-os em R0 e mandando-os para o LCD.
    LDRB R0, [R4], #1           ;Termina o envio quando um '0' é detectado na string.
    CMP R0, #'\0'
    BEQ stringCopy_end 
    BL  LCD_PushChar
    B   stringCopy
customStringCopy_end    
    POP {R12, R0, R4, R5, R6, LR}
    BX  LR

;------------LCD_PushChar------------
; Envia um caractere para o LCD.
; Entrada: R0 --> Caractere a ser enviado
; Saída: Nenhum
; Modifica: -- (apenas mudanças temporárias)
LCD_PushChar
    PUSH {R0, R1, R2, R3, R12, LR}
    MOV R2, R0
    MOV R0, #2_00000000
    BL  PortM_Output
    MOV R0, R2
    BL  PortK_Output
    MOV R0, #2_00000101
    BL  PortM_Output
    MOV R0, #DELAY_SMALL
    BL  SysTick_Wait1us
    MOV R0, #2_00000000
    BL  PortM_Output
    POP {R0, R1, R2, R3, R12, LR}
    BX  LR

;------------LCD_PositionCursor------------
; Modifica posição do cursor de acordo com parâmetros recebidos
;   0x80 + (linha * 0x40) + coluna
; Entrada:  R0 --> Linha
;           R1 --> Coluna

; Saída: Nenhum
LCD_PositionCursor
    PUSH {R0, R1, R2, LR}

    MOV R2, #0x40
    MUL R2, R0

    ADD R2, R1

    ADD R2, #0x80

    MOV R0, R2

    BL LCD_PushConfig

    POP {R0, R1, R2, LR}
    BX  LR


    
    ALIGN                        ;Garante que o fim da seção está alinhada 
    END                          ;Fim do arquivo