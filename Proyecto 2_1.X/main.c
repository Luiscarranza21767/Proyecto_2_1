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


//******************************************************************************
// Librerías
//******************************************************************************
#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>
#include "oscilador.h"
#include "setupADC.h"
#include "setupPWM.h"
#include "setupUART.h"
#include "OpNum.h"
#include "ReadWrite.h"

//******************************************************************************
// Declaración de constantes
//******************************************************************************
#define _XTAL_FREQ 1000000
#define valTMR0 100
#define PR2PWM 254
//******************************************************************************
// Prototipos de funciones
//******************************************************************************
void controlservos(void);
void setup(void);
void setupTMR0(void);
void hightime(int tdelay);
void setup_portb(void);
void adafruitrec(void);
void LEDpos(void);
void LEDmod(void);
void writepos(void);
void lecpos(void);
void lecturanum(void);
//******************************************************************************
// Declaración de variables
//******************************************************************************
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
char dec;
char cent;
char uni;

//******************************************************************************
// Función de interrupciones
//******************************************************************************
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
            if (modo < 2){
                modo = modo + 1;
            }
            else {
                modo = 0;
            }
            LEDmod();
        }
        if(PORTBbits.RB6 == 0){
            while(PORTBbits.RB6 == 0);
            if (pos < 4){
                pos = pos +1;
            }
            else {
                pos = 1;
            }
            LEDpos();
        }
        if(PORTBbits.RB5 == 0){
            while(PORTBbits.RB5 == 0);
            if (pos > 1){
                pos = pos - 1;
            }
            else {
                pos = 4;
            }
            LEDpos();
        }
        if(PORTBbits.RB4 == 0){
            while(PORTBbits.RB4 == 0);
            B4Flag = 1;
        }
        INTCONbits.RBIF = 0;  
    }   
}
//******************************************************************************
// Función principal
//******************************************************************************
void main(void) {
    setup();
    setupINTOSC(4);     // Oscilador a 1 MHz
    setup_ADC();
    setupPWM();
    setupTMR0();
    setup_portb();
    initUART();
    pos = 1;
    B4Flag = 0;
    PORTDbits.RD0 = 1; // Solo desde el inicio indica que la posición es 1
    while(1){
        if (PIR1bits.RCIF == 1){
            adafruitrec();                
        }
        if(modo != 1){
            controlservos();  
            if (B4Flag == 1){
                B4Flag = 0;
                writepos();
            }
            __delay_ms(1);
        }
        else if (modo == 1){
            if (B4Flag == 1){
                B4Flag = 0;
                lecpos();              
            }
            controlservos();
            __delay_ms(1);
        }
        
    }
}
//******************************************************************************
// Función de configuración inicial de puertos
//******************************************************************************
void setup(void){
    ANSELH = 0;
    TRISB = 0;
    TRISC = 0b10000000; 
    TRISD = 0;
    PORTC = 0;
    PORTD = 0;
}
//******************************************************************************
// Función para configuración del puerto B
//******************************************************************************
void setup_portb(void){
    TRISB = 0b11110000;
    INTCONbits.RBIE = 1;    // Habilita interrupción del puerto B
    INTCONbits.RBIF = 0;    // Apaga la bandera de interrupción del puerto B
    IOCB = 0b11110000;      // Habilita la interrupción en cambio
    WPUB = 0b11110000;      // Habilita el Weak Pull-Up en el puerto B
    OPTION_REGbits.nRBPU = 0;   // Deshabilita el bit de RBPU
}
//******************************************************************************
// Función para configuración de TMR0
//******************************************************************************
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
//******************************************************************************
// Función para realizar delay variable
//******************************************************************************
void hightime(int tdelay){
    while (tdelay > 0){
        tdelay = tdelay - 1;
    } 
}
//******************************************************************************
// Función para conectar con Adafruit
//******************************************************************************
void adafruitrec(void){
    switch (RCREG){
        case 'a':
            PIR1bits.RCIF  = 0;
            while(!RCIF);
            if (RCREG == '1'){
                PIR1bits.RCIF  = 0;
                if (modo < 2){
                    modo = modo + 1;
                }
                else {
                    modo = 0;
                }
                LEDmod();
            }
            break;
        case 'b':
            PIR1bits.RCIF  = 0;
            while(!RCIF);
            if (RCREG == '1' & modo != 1){
                writepos();
            }  
            else if (RCREG == '1' & modo == 1){
                lecpos();
            }
            break;
        case 'c':
            PIR1bits.RCIF  = 0;
            while(!RCIF);
            if (RCREG == '1'){
                pos = 1;
            }
            else if(RCREG == '2'){
                pos = 2;
            }
            else if(RCREG == '3'){
                pos = 3;
            }           
            else if(RCREG == '4'){
                pos = 4;
            }
            LEDpos();
            PIR1bits.RCIF  = 0;  
            break;
            
        case 'd':
            if (modo == 2){
                lecturanum();
                v1PWM = convint(cent, dec, uni);
                v1PWM = mapeo(v1PWM, 0, 180, 63, 125);
            }
            break;
        case 'e':
            if (modo == 2){
                lecturanum();
                v2PWM = convint(cent, dec, uni);
                v2PWM = mapeo(v2PWM, 0, 180, 63, 125);
            }
            break;
        case 'f':
            if (modo == 2){
                lecturanum();
                HIGHpulse0 = convint(cent, dec, uni);
                HIGHpulse0 = mapeo(HIGHpulse0, 0, 180, 9, 26);
            }
            break;
        case 'g':
            if (modo == 2){
                lecturanum();
                HIGHpulse1 = convint(cent, dec, uni);
                HIGHpulse1 = mapeo(HIGHpulse1, 0, 180, 9, 20);
            }
            break;
        default:
            PIR1bits.RCIF = 0;
    }
}
//******************************************************************************
// Funciones para indicadores de posición y modo
//******************************************************************************
void LEDpos(void){
    switch (pos){
        case 1: 
            PORTD = (PORTD & 0xF0) | 0x01;
            break;
        case 2:
            PORTD = (PORTD & 0xF0) | 0x02;
            break;
        case 3:
            PORTD = (PORTD & 0xF0) | 0x04;
            break;
        case 4:
            PORTD = (PORTD & 0xF0) | 0x08;
            break;

    }
}
void LEDmod(void){
    switch (modo){
        case 0: 
            PORTD = (PORTD & 0x0F) | 0x10;
            break;
        case 1:
            PORTD = (PORTD & 0x0F) | 0x20;
            break;
        case 2:
            PORTD = (PORTD & 0x0F) | 0x40;
            break;
            
    }
}
//******************************************************************************
// Funciones para lectura y escritura según la posición
//******************************************************************************
void writepos(void){
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
void lecpos(void){
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
//******************************************************************************
// Función para reconocer el número (caracteres) que devuelve Adafruit
//******************************************************************************
void lecturanum(void){
    PIR1bits.RCIF  = 0;
    while(!RCIF);
    cent = RCREG;
    PIR1bits.RCIF  = 0;
    while(!RCIF);
    dec = RCREG;
    PIR1bits.RCIF  = 0;
    while(!RCIF);
    uni = RCREG;
    PIR1bits.RCIF  = 0; 
}
//******************************************************************************
// Control de servomotores independiente del modo
//******************************************************************************
void controlservos(void){
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
        HIGHpulse0 = mapeo(ADRESH, 0, 255, 9, 26);
    }
    
     // Cambia a canal analógico 3
    ADCON0bits.CHS = 0b0011;
    __delay_us(100);
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO == 1); // Revisa si ya terminó la conversión ADC
    ADIF = 0; 
    if (modo == 0){
        HIGHpulse1 = mapeo(ADRESH, 0, 255, 9, 20);
    }    
    __delay_ms(50);
}