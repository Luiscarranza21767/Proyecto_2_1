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
#define valTMR0 100
#define PR2PWM 254
void modomanual(void);
void setup(void);
void setupTMR0(void);
void hightime(int tdelay);
void setup_portb(void);
void write_EEPROM(uint8_t address, uint8_t data);
unsigned int mapeo(int valor, int inmin, int inmax, int outmin, int outmax);
uint8_t read_EEPROM(uint8_t address);

unsigned int v1PWM;
unsigned int v2PWM;
unsigned int vPWMl;
unsigned int vPWMh;
unsigned int HIGHpulse0;
unsigned int HIGHpulse1;
int cont;
int modo;
int pos;
int B4Flag;

void __interrupt() isr (void){
    if (INTCONbits.T0IF){
        INTCONbits.T0IF = 0;
        TMR0 = valTMR0;
        PORTCbits.RC0 = 1;
        hightime(HIGHpulse0);
        PORTCbits.RC0 = 0;
        PORTCbits.RC3 = 1;
        hightime(HIGHpulse1);
        PORTCbits.RC3 = 0;
        
    }
    if (INTCONbits.RBIF){             // Revisa si hay interrupción del puerto B
        if (PORTBbits.RB7 == 0)     // Si hay revisa si se presionó RB6
        {
            while(PORTBbits.RB7 == 0);
            if (modo < 1){
                modo = modo + 1;
            }
            else {
                modo = 0;
            }
        }
        if(PORTBbits.RB6 == 0){
            while(PORTBbits.RB6 == 0);
            if (pos < 4){
                pos = pos +1;
            }
            else {
                pos = 1;
            }
        }
        if(PORTBbits.RB5 == 0){
            while(PORTBbits.RB5 == 0);
            if (pos > 1){
                pos = pos - 1;
            }
            else {
                pos = 4;
            }
        }
        if(PORTBbits.RB4 == 0){
            while(PORTBbits.RB4 == 0);
            B4Flag = 1;
        }
        INTCONbits.RBIF = 0;  
    }   
}
void main(void) {
    setup();
    setupINTOSC(4);     // Oscilador a 1 MHz
    setup_ADC();
    setupPWM();
    setupTMR0();
    setup_portb();
    pos = 1;
    B4Flag = 0;
    while(1){
        if(modo == 0){
            modomanual();  
            PORTD = pos;
            if (B4Flag == 1){
                B4Flag = 0;
                int addr;
                if (pos == 1){
                    addr = 0;}
                else if (pos == 2){
                    addr = 4;}
                else if (pos == 3){
                    addr = 8;}
                else if (pos == 4){
                    addr = 12;}
                write_EEPROM(addr, v1PWM);
                write_EEPROM((addr + 1), v2PWM);
                write_EEPROM((addr + 2), HIGHpulse0);
                write_EEPROM((addr + 3), HIGHpulse1);
            }
            __delay_ms(1);
        }
        else if (modo == 1){
            if (B4Flag == 1){
                B4Flag = 0;
                int addr;
                if (pos == 1){
                    addr = 0;}
                else if (pos == 2){
                    addr = 4;}
                else if (pos == 3){
                    addr = 8;}
                else if (pos == 4){
                    addr = 12;}
                v1PWM = read_EEPROM(addr);
                v2PWM = read_EEPROM((addr+1));
                HIGHpulse0 = read_EEPROM((addr+2));
                HIGHpulse1 = read_EEPROM((addr+3));                
            }
            modomanual();
            PORTD = pos;
            __delay_ms(1);
        }
        
    }
}

void setup(void){
    ANSELH = 0;
    TRISB = 0;
    TRISC = 0b10000000; 
    TRISD = 0;
    PORTC = 0;
    PORTD = 0;
}

void setup_portb(void){
    TRISB = 0b11110000;
    INTCONbits.RBIE = 1;    // Habilita interrupción del puerto B
    INTCONbits.RBIF = 0;    // Apaga la bandera de interrupción del puerto B
    IOCB = 0b11110000;      // Habilita la interrupción en cambio
    WPUB = 0b11110000;      // Habilita el Weak Pull-Up en el puerto B
    OPTION_REGbits.nRBPU = 0;   // Deshabilita el bit de RBPU
}

void setupTMR0(void){
    INTCONbits.GIE = 1;         // Habilitar interrupciones globales
    INTCONbits.T0IE = 1;        // Habilitar interrupción de TMR0
    INTCONbits.T0IF = 0;        // Desactivar la bandera de TMR0
    
    OPTION_REGbits.T0CS = 0;    // Fosc/4
    OPTION_REGbits.PSA = 0;     // Prescaler para TMR0
    OPTION_REGbits.PS = 0b100;  // Prescaler 1:32
    TMR0 = valTMR0;                   // Valor inicial del TMR0
    cont = 0;
}

unsigned int mapeo(int valor, int inmin, int inmax, int outmin,int outmax){
    unsigned int resultado;
    resultado = (((outmax-outmin)*(valor-inmin)/(inmax)) + outmin);
    return resultado;
}

void hightime(int tdelay){
    while (tdelay > 0){
        tdelay = tdelay - 1;
    } 
}

void write_EEPROM(uint8_t address, uint8_t data){
    while(WR);
    EEADR = address;
    EEDAT = data;
    EECON1bits.EEPGD = 0;
    EECON1bits.WREN = 1;
    INTCONbits.GIE = 0;
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;
    EECON1bits.WREN = 0;
    
    INTCONbits.GIE = 1;
}

uint8_t read_EEPROM(uint8_t address){
    while(WR || RD);
    EEADR = address;
    EECON1bits.EEPGD = 0;
    EECON1bits.RD = 1;
    return EEDAT;
}

void modomanual(void){
    //PWM con módulo CCP
    // Iniciar la conversión ADC
    ADCON0bits.CHS = 0b0000;
    __delay_us(100);
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO == 1); // Revisa si ya terminó la conversión ADC
    ADIF = 0;                   // Apaga la bandera del ADC
    // Hará el mapeo del ADC a valores para el servo
    // mapeo(valor, inmax, outmin, outmax)
    if (modo == 0){
        v1PWM = mapeo(ADRESH, 0, 255, 63, 125);
    }
    // Obtiene los 2 bits más bajos de vPWM
    vPWMl = v1PWM & 0x003;
    // Obtiene los 8 bits más altos de vPWM 
    vPWMh = (v1PWM & 0x3FC) >> 2;
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
    if (modo == 0){
        v2PWM = mapeo(ADRESH, 0, 255, 63, 125);
    }
    // Obtiene los 2 bits más bajos de vPWM
    vPWMl = v2PWM & 0x003;
    // Obtiene los 8 bits más altos de vPWM 
    vPWMh = (v2PWM & 0x3FC) >> 2;
    // Carga los bits bajos a CCP2CON <5:4>
    CCP2CONbits.DC2B0 = vPWMl & 0x01;
    CCP2CONbits.DC2B1 = ((vPWMl & 0x02) >> 1);
    // Carga los bits altos a CCPR2L
    CCPR2L = vPWMh;  
    
    //PWM con TMR0
    // Cambia a canal analógico 2
    ADCON0bits.CHS = 0b0010;
    __delay_us(100);
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO == 1); // Revisa si ya terminó la conversión ADC
    ADIF = 0; 
    if (modo == 0){
        HIGHpulse0 = mapeo(ADRESH, 0, 255, 9, 27);
    }
    
     // Cambia a canal analógico 3
    ADCON0bits.CHS = 0b0011;
    __delay_us(100);
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO == 1); // Revisa si ya terminó la conversión ADC
    ADIF = 0; 
    if (modo == 0){
        HIGHpulse1 = mapeo(ADRESH, 0, 255, 9, 22);
    }    
    __delay_ms(50);
}