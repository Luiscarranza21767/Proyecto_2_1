/*
 * File:   oscilador.c
 * Author: Luis Pablo Carranza
 *
 * Created on 24 de octubre de 2022, 11:37 PM
 */

#include "setupUART.h"
void initUART(void){
    // Configuración velocidad de baud rate
    SPBRG = 25;             // Valor calculado para 9600 baudios
    SPBRGH = 0;
    BAUDCTLbits.BRG16 = 1;
    TXSTAbits.BRGH = 1;     // 16 bits asíncrono
    
    TXSTAbits.SYNC = 0;     // Modo asíncrono
    RCSTAbits.SPEN = 1;     // Habilitar módulo UART
    
    TXSTAbits.TXEN = 1;     // Habilitar la transmisión
    PIR1bits.TXIF = 0;
    
    RCSTAbits.CREN = 1;     // Habilitar la recepción
}
