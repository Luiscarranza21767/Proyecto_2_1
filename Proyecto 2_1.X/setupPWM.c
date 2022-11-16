/*
 * File:   oscilador.c
 * Author: Luis Pablo Carranza
 *
 * Created on 24 de octubre de 2022, 11:37 PM
 */

#include "setupPWM.h"
#define PR2PWM 254

void setupPWM(void){
    // Configuración para CCP1
    TRISCbits.TRISC2 = 1;       // CCP1
    PR2 = PR2PWM;               // Periodo de 16.32 ms
    CCP1CON = 0b00001100;       // P1A como PWM
    TMR2IF = 0;                 // bandera de TMR2 apagada
    T2CONbits.T2CKPS = 0b11;    // Prescaler 1:16
    TMR2ON = 1;                 // Habilitar TMR2
    while(!TMR2IF);             //
    TRISCbits.TRISC2 = 0;       // Habilitar la salida del PWM
    
    //Configuración para CCP0
    TRISCbits.TRISC1 = 1;       // CCP0
    CCP2CON = 0b00001100;       // P2A como PWM
    TMR2IF = 0;                 // bandera de TMR2 apagada
    while(!TMR2IF);
    TRISCbits.TRISC1 = 0;       // Habilitar la salida del PWM
}
