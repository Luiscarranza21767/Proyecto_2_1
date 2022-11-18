/* 
 * File:   OpNum.h
 * Author: Pablo
 *
 * Created on 18 de noviembre de 2022, 01:04 AM
 */

#ifndef OPNUM_H
#define	OPNUM_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>


int chartoint(char num);
int convint(char centenas, char decenas, char unidades);
unsigned int mapeo(int valor, int inmin, int inmax, int outmin, int outmax);

#endif	/* OPNUM_H */

