/* Universidad del Valle de Guatemala
 IE2023 Programación de Microcontroladores
 Autor: Luis Pablo Carranza
 Compilador: XC8, MPLAB X IDE (v6.00)
 Proyecto: Proyecto de laboratorio 2
 Hardware PIC16F887
 Creado: 3/11/22
 Última Modificación: 15/11/22*/

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT // Oscillator Selection bits (INTOSC 
//oscillator without clock out)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and 
//can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR 
//pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code 
//protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code 
//protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/
//External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-
//Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin 
//has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit 
//(Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits 
//(Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include "oscilador.h"
#include "setupADC.h"
#include "setupPWM.h"

#define _XTAL_FREQ 1000000
#define valTMR0 246
#define PR2PWM 254
void modomanual(void);
void setup(void);
unsigned int mapeo(int valor, int inmin, int inmax, int outmin, int outmax);

unsigned int vPWM;
unsigned int vPWMl;
unsigned int vPWMh;

void main(void) {
    setup();
    setupINTOSC(4);     // Oscilador a 1 MHz
    setup_ADC();
    setupPWM();
    while(1){
        modomanual();        
        __delay_ms(1);
    }
}

void setup(void){
    ANSELH = 0;
    TRISB = 0;
    TRISC = 0; 
    TRISD = 0;
    PORTC = 0;
    PORTD = 0;
}

unsigned int mapeo(int valor, int inmin, int inmax, int outmin,int outmax){
    unsigned int resultado;
    resultado = (((outmax-outmin)*(valor-inmin)/(inmax)) + outmin);
    return resultado;
}

void modomanual(void){
    // Iniciar la conversión ADC
    ADCON0bits.CHS = 0b0000;
    __delay_us(100);
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO == 1); // Revisa si ya terminó la conversión ADC
    ADIF = 0;                   // Apaga la bandera del ADC
    // Hará el mapeo del ADC a valores para el servo
    // mapeo(valor, inmax, outmin, outmax)
    vPWM = mapeo(ADRESH, 0, 255, 63, 125);
    PORTD = vPWM;
    // Obtiene los 2 bits más bajos de vPWM
    vPWMl = vPWM & 0x003;
    // Obtiene los 8 bits más altos de vPWM 
    vPWMh = (vPWM & 0x3FC) >> 2;
    // Carga los bits bajos a CCP1CON <5:4>
    CCP1CONbits.DC1B = vPWMl;
    // Carga los bits altos a CCPR1L
    CCPR1L = vPWMh;
    
    // Cambia a canal analógico 1
    ADCON0bits.CHS = 0b0001;
    __delay_us(100);
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO == 1); // Revisa si ya terminó la conversión ADC
    ADIF = 0;               // Apaga la bandera del ADC
    // Hará el mapeo del ADC a valores para el servo
    vPWM = mapeo(ADRESH, 0, 255, 63, 125);
    // Obtiene los 2 bits más bajos de vPWM
    vPWMl = vPWM & 0x003;
    // Obtiene los 8 bits más altos de vPWM 
    vPWMh = (vPWM & 0x3FC) >> 2;
    // Carga los bits bajos a CCP2CON <5:4>
    CCP2CONbits.DC2B0 = vPWMl & 0x01;
    CCP2CONbits.DC2B1 = ((vPWMl & 0x02) >> 1);
    // Carga los bits altos a CCPR2L
    CCPR2L = vPWMh;
    __delay_ms(10);    
        
}