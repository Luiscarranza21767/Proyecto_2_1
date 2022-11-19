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
// Variables para PWM con módulo CCP
unsigned int v1PWM;
unsigned int v2PWM;
unsigned int vPWMl;
unsigned int vPWMh;
// Variables para PWM con TMR0
unsigned int HIGHpulse0;
unsigned int HIGHpulse1;
// Variabels indicadoras
int cont;
int modo;
int pos;
int B4Flag;
// Variables para almacenar valores recibidos
char dec;
char cent;
char uni;

//******************************************************************************
// Función de interrupciones
//******************************************************************************
void __interrupt() isr (void){
    if (INTCONbits.T0IF){       // Si hay interrupción del TMR0 pasaron 20ms
        INTCONbits.T0IF = 0;    // Apaga la bandera
        TMR0 = valTMR0;         // Carga el valor del timer
        PORTCbits.RC0 = 1;      // Enciende uno de los puertos de pwm
        hightime(HIGHpulse0);   // Hace un delay que varía según HIGHpulse0
        PORTCbits.RC0 = 0;      // Apaga el puerto
        PORTCbits.RC3 = 1;      // Enciende el siguiente PWM según el delay 
        hightime(HIGHpulse1);
        PORTCbits.RC3 = 0;
        
    }
    if (INTCONbits.RBIF){           // Revisa si hay interrupción del puerto B
        if (PORTBbits.RB7 == 0)     // Revisa si se presionó RB7 (cambio modo)
        {
            while(PORTBbits.RB7 == 0); // Antirrebotes
            if (modo < 2){          // Si modo <2 lo incrementa
                modo = modo + 1;
            }
            else {                  // De lo contrario reinicia en 0
                modo = 0;
            }
            LEDmod();               // Enciende el led según el modo
        }
        if(PORTBbits.RB6 == 0){     // Revisa si se presionó RB6 (posición +1)
            while(PORTBbits.RB6 == 0);
            if (pos < 4){           // Si pos<4 la incrementa
                pos = pos +1;
            }
            else {                  // De lo contrario reinicia en 1
                pos = 1;
            }
            LEDpos();               // Enciende el led según la posición
        }
        if(PORTBbits.RB5 == 0){     // Revisa si se presionó RB5
            while(PORTBbits.RB5 == 0);
            if (pos > 1){           // Si se presionó decrementa la posición
                pos = pos - 1;
            }
            else {                  // Si llega a 1 regresa a 4
                pos = 4;
            }
            LEDpos();               // Enciende el led de la posición
        }
        if(PORTBbits.RB4 == 0){     // Revisa si se presiona RB4
            while(PORTBbits.RB4 == 0);
            B4Flag = 1;             // Si si enciende la bandera de RB4
        }
        INTCONbits.RBIF = 0;  
    }   
}
//******************************************************************************
// Función principal
//******************************************************************************
void main(void) {
    setup();            // Configura registros PORT y TRIS
    setupINTOSC(4);     // Oscilador a 1 MHz
    setup_ADC();        // Configura e inicia el ADC
    setupPWM();         // Configura el PWM de CCP1 y CCP0  
    setupTMR0();        // Configura el TMR0
    setup_portb();      // Configura puerto B (IoC, WPUB, TRISB)
    initUART();         // Inicia el módulo UART
    pos = 1;            // Indica que la posición inicial por defecto es 1
    B4Flag = 0;         // Inicialmente el botón B4 no está encendido (L/E)
    PORTDbits.RD0 = 1; // Solo desde el inicio indica que la posición es 1
    
    while(1){
        if (PIR1bits.RCIF == 1){    // Si recibe un dato llama a la función
            adafruitrec();                
        }
        if(modo != 1){              // Si modo es distinto de 1 llama al 
            controlservos();        // control de los servos y revisa si se 
            if (B4Flag == 1){       // Presiona el botón B4 para escribir datos
                B4Flag = 0;
                writepos();         // Guarda la posición
            }
            __delay_ms(1);
        }
        else if (modo == 1){        // Si modo es 1 revisa si se presiona B4 
            if (B4Flag == 1){       // Pero en lugar de escribir lee la posición
                B4Flag = 0;
                lecpos();              
            }
            controlservos();        // Llama al control de servos
            __delay_ms(1);
        }
        
    }
}
//******************************************************************************
// Función de configuración inicial de puertos
//******************************************************************************
void setup(void){
    ANSELH = 0;             // No se necesitan más entradas analógicas
    TRISC = 0b10000000;     // El bit 7 de TRISC es entrada
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
    IOCB = 0b11110000;      // Habilita la interrupción en cambio (IoC)
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
        tdelay = tdelay - 1;    // Se mantiene en el loop hasta que Tdelay = 0
    }                           // Se usa para hacer delays variables
}
//******************************************************************************
// Función para conectar con Adafruit
//******************************************************************************
void adafruitrec(void){         
    switch (RCREG){             // Abre un switch case para el dato que recibe
        case 'a':               // Si el dato es una 'a' es el feed de cambio
            PIR1bits.RCIF  = 0; // de modo
            while(!RCIF);       // Mientras no reciba otro dato no hace nada
            if (RCREG == '1'){  // Si recibe un '1' hace la función de 
                PIR1bits.RCIF  = 0; // incremento de modo
                if (modo < 2){
                    modo = modo + 1;
                }
                else {
                    modo = 0;
                }
                LEDmod();
            }
            break;
        case 'b':               // Si el dato que recibe es 'b' es el fee de
            PIR1bits.RCIF  = 0; // Lectura/Escritura
            while(!RCIF);
            if (RCREG == '1' & modo != 1){  // Si modo es distinto de 1 escribe
                writepos();
            }  
            else if (RCREG == '1' & modo == 1){ // Si modo es 1 entonces lee
                lecpos();
            }
            break;
        case 'c':               // Si el dato recibido es una 'c' es el feed
            PIR1bits.RCIF  = 0; // de cambio de posición
            while(!RCIF);
            if (RCREG == '1'){  // Si recibe un '1' es la posición 1
                pos = 1;
            }
            else if(RCREG == '2'){ // Si recibe un '2' es la segunda...
                pos = 2;
            }
            else if(RCREG == '3'){
                pos = 3;
            }           
            else if(RCREG == '4'){
                pos = 4;
            }
            LEDpos();           // Actualiza el led de posición
            PIR1bits.RCIF  = 0; // Apaga bandera de recepción de datos
            break;
            
        case 'd':               // Si recibe una 'd' es el feed para controlar
            if (modo == 2){     // El primer servomotor
                lecturanum();   // Lee el valor que recibe del feed
                v1PWM = convint(cent, dec, uni);    // Lo convierte en un entero
                v1PWM = mapeo(v1PWM, 0, 180, 63, 125);  // de 3 cifras y mapea
            }                   // El valor de la variable para el PWM actualiza
                                // Según el dato recibido del feed
            break;
        case 'e':               // Misma funcionalidad que el caso 'd' pero con 
            if (modo == 2){     // El feed del segundo servo
                lecturanum();
                v2PWM = convint(cent, dec, uni);
                v2PWM = mapeo(v2PWM, 0, 180, 63, 125);
            }
            break;  
        case 'f':               // Misma funcionalidad que el caso 'd' pero con 
            if (modo == 2){     // el feed del tercer servo, por eso el mapeo
                lecturanum();   // es distinto, es un PWM con tmr0
                HIGHpulse0 = convint(cent, dec, uni);
                HIGHpulse0 = mapeo(HIGHpulse0, 0, 180, 9, 26);
            }
            break;
        case 'g':               // Misma funcionalidad que el caso 'f' para el
            if (modo == 2){     // feed del cuarto servo
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
            PORTD = (PORTD & 0xF0) | 0x01;  // Cambia el LED de posición sin
            break;                          // afectar los 4 bits más significa-
        case 2:                             // tivos que son del modo
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
            PORTD = (PORTD & 0x0F) | 0x10;  // Cambia el LED de modo sin afectar
            break;                          // los 4 bits menos significativos
        case 1:                             // que son de la posición
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
    if (pos == 1){          // Si es la primera posición lo direcciona a 0x0  
        addr = 0;}
    else if (pos == 2){     // Si es la segunda lo direcciona a 0x4
        addr = 4;}
    else if (pos == 3){     // Si es la tercera lo direcciona a 0x8
        addr = 8;}
    else if (pos == 4){     // Si es la cuarta lo direcciona a 0xC
        addr = 12;}
    write_EEPROM(addr, v1PWM);              // Escribe en la dirección
    write_EEPROM((addr + 1), v2PWM);        // Escribe en la dirección + 1
    write_EEPROM((addr + 2), HIGHpulse0);   // Escribe en la dirección + 2
    write_EEPROM((addr + 3), HIGHpulse1);   // Escribe en la dirección + 3
    // Si la posición es 2, guarda un dato en la dirección 4, el siguiente en la
    // dirección 5, el siguiente en la 6 y el último en la 7.
}
void lecpos(void){
    int addr;
    if (pos == 1){          // Función igual a la escritura para indicar la 
        addr = 0;}          // Dirección de la posición
    else if (pos == 2){
        addr = 4;}
    else if (pos == 3){
        addr = 8;}
    else if (pos == 4){
        addr = 12;}
    v1PWM = read_EEPROM(addr);          // Lee los valores de la posición uno
    v2PWM = read_EEPROM((addr+1));      // por uno cambiando la dirección en + 1
    HIGHpulse0 = read_EEPROM((addr+2)); // para cada variable
    HIGHpulse1 = read_EEPROM((addr+3));  
}
//******************************************************************************
// Función para reconocer el número (caracteres) que devuelve Adafruit
//******************************************************************************
void lecturanum(void){  
    PIR1bits.RCIF  = 0;
    while(!RCIF);
    cent = RCREG;       // El primer dato que recibe lo almacena en cent
    PIR1bits.RCIF  = 0;
    while(!RCIF);       // El segundo en dec
    dec = RCREG;
    PIR1bits.RCIF  = 0;
    while(!RCIF);
    uni = RCREG;        // El tercero en uni
    PIR1bits.RCIF  = 0; 
}
//******************************************************************************
// Control de servomotores independiente del modo
//******************************************************************************
void controlservos(void){
    //PWM con módulo CCP
    ADCON0bits.CHS = 0b0000;    // Canal analógico 0
    __delay_us(100);
    ADCON0bits.GO = 1;          // Iniciar la conversión ADC
    while (ADCON0bits.GO == 1); // Revisa si ya terminó la conversión ADC
    ADIF = 0;                   // Apaga la bandera del ADC
    if (modo == 0){             // Hará el mapeo del ADC a valores para el servo
        v1PWM = mapeo(ADRESH, 0, 255, 63, 125); 
    }
    // Si no está en el modo 0 solo cambia según los valores de las variables
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
    ADCON0bits.GO = 1;          // Iniciar la conversión ADC
    while (ADCON0bits.GO == 1); // Revisa si ya terminó la conversión ADC
    ADIF = 0;                   // Apaga la bandera del ADC
    if (modo == 0){             // Hará el mapeo del ADC a valores para el servo
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
    ADCON0bits.GO = 1;          // Iniciar la conversión ADC
    while (ADCON0bits.GO == 1); // Revisa si ya terminó la conversión ADC
    ADIF = 0; 
    if (modo == 0){             //Si modo es 0 entonces hace el mapeo con ADRESH
        HIGHpulse0 = mapeo(ADRESH, 0, 255, 9, 26);
    }
    
     // Cambia a canal analógico 3
    ADCON0bits.CHS = 0b0011;
    __delay_us(100);
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO == 1); // Revisa si ya terminó la conversión ADC
    ADIF = 0; 
    if (modo == 0){             //Si modo es 0 entonces hace el mapeo con ADRESH
        HIGHpulse1 = mapeo(ADRESH, 0, 255, 9, 20);
    }    
    __delay_ms(50);
}