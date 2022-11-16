/*
 * File:   oscilador.c
 * Author: Luis Pablo Carranza
 *
 * Created on 24 de octubre de 2022, 11:37 PM
 */

#include "setupADC.h"
#define _XTAL_FREQ 1000000

void setup_ADC(void){
    PORTAbits.RA0 = 0;      // Inicia el bit 0 de PORTA en 0
    TRISAbits.TRISA0 = 1;   // RA0 es entrada
    ANSELbits.ANS0 = 1;     // RA0 es analógico
    
    PORTAbits.RA1 = 0;      // Configuración del canal analógico RA1
    TRISAbits.TRISA1 = 1;
    ANSELbits.ANS1 = 1;
    
    PORTAbits.RA2 = 0;      // Configuración del canal analógico RA2
    TRISAbits.TRISA2 = 1;
    ANSELbits.ANS2 = 1;
    
    PORTAbits.RA3 = 0;      // Configuración del canal analógico RA2
    TRISAbits.TRISA3 = 1;
    ANSELbits.ANS3 = 1;
    
    ADCON0bits.ADCS1 = 0;
    ADCON0bits.ADCS0 = 1;   // Fosc/8
    
    ADCON1bits.VCFG1 = 0;   // Ref VSS
    ADCON1bits.VCFG0 = 0;   // Ref VDD
    
    ADCON1bits.ADFM = 0;    // Justificado a la izquierda
    
    ADCON0bits.ADON = 1;    // Habilitar el convertidor ADC
    __delay_us(100);
}