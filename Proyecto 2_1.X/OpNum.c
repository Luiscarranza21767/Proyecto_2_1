/*
 * File:   oscilador.c
 * Author: Luis Pablo Carranza
 *
 * Created on 24 de octubre de 2022, 11:37 PM
 */

#include "OpNum.h"

// Función para convertir variables de tipo caracter a entero
int chartoint(char num){
    if (num == '0'){ // Si el caracter recibido es '0' devuelve un 0 en entero
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

// Función para convertir variables de tipo caracter con 3 posiciones a entero 
int convint(char centenas, char decenas, char unidades){
    int u;
    int d;
    int c;
    u = chartoint(unidades);    // Convierte las unidades de caracter a entero
    d = chartoint(decenas);
    c = chartoint(centenas);
    return ((c*100)+(d*10)+u);  // Devuelve solo un número de 3 dígitos
}

// Mapea las variables de un rango s-t a uno x-y
unsigned int mapeo(int valor, int inmin, int inmax, int outmin,int outmax){
    unsigned int resultado; // Regla de tres para regresar un valor
    resultado = (((outmax-outmin)*(valor-inmin)/(inmax)) + outmin);
    return resultado;
}




