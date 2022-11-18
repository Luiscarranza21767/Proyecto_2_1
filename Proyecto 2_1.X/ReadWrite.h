/* 
 * File:   ReadWrite.h
 * Author: Pablo
 *
 * Created on 18 de noviembre de 2022, 01:12 AM
 */

#ifndef READWRITE_H
#define	READWRITE_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>

void write_EEPROM(uint8_t address, uint8_t data);
uint8_t read_EEPROM(uint8_t address);



#endif	/* READWRITE_H */

