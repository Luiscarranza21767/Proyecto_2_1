/*
 * File:   oscilador.c
 * Author: Luis Pablo Carranza
 *
 * Created on 24 de octubre de 2022, 11:37 PM
 */

#include "OpNum.h"


int chartoint(char num){
    if (num == '0'){
        return 0;
    }
    else if(num == '1'){
        return 1;
    }
    else if(num == '2'){
        return 2;
    }
    else if(num == '3'){
        return 3;
    }
    else if(num == '4'){
        return 4;
    }
    else if(num == '5'){
        return 5;
    }
    else if(num == '6'){
        return 6;
    }
    else if(num == '7'){
        return 7;
    }
    else if(num == '8'){
        return 8;
    }
    else if(num == '9'){
        return 9;
    }
}

int convint(char centenas, char decenas, char unidades){
    int u;
    int d;
    int c;
    u = chartoint(unidades);
    d = chartoint(decenas);
    c = chartoint(centenas);
    return ((c*100)+(d*10)+u); 
}

unsigned int mapeo(int valor, int inmin, int inmax, int outmin,int outmax){
    unsigned int resultado;
    resultado = (((outmax-outmin)*(valor-inmin)/(inmax)) + outmin);
    return resultado;
}




