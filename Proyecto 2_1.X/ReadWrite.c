/* 
 * File:   ReadWrite.h
 * Author: Pablo
 *
 * Created on 18 de noviembre de 2022, 01:12 AM
 */

#include "ReadWrite.h"

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

