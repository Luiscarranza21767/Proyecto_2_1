/* 
 * File:   ReadWrite.h
 * Author: Pablo
 *
 * Created on 18 de noviembre de 2022, 01:12 AM
 */

#include "ReadWrite.h"

void write_EEPROM(uint8_t address, uint8_t data){
    while(WR);
    EEADR = address;        // Recibe la dirección de memoria
    EEDAT = data;           // Carga el dato
    EECON1bits.EEPGD = 0;   
    EECON1bits.WREN = 1;    // Habilitar escritura
    INTCONbits.GIE = 0;     // Deshabilita interrupciones globales
    EECON2 = 0x55;          // Secuencia de escritura
    EECON2 = 0xAA;
    EECON1bits.WR = 1;
    EECON1bits.WREN = 0;
    
    INTCONbits.GIE = 1;     // Vuelve a habilitar interrupciones globales
}

uint8_t read_EEPROM(uint8_t address){
    while(WR || RD);
    EEADR = address;        // Carga la dirección 
    EECON1bits.EEPGD = 0;   // 
    EECON1bits.RD = 1;      // Habilita la lectura
    return EEDAT;           // Regresa el dato leído
}

