#include "PWM_SERVO1.h"
#ifndef _XTAL_FREQ
#define _XTAL_FREQ 4000000
#endif
void PWM_config(char canal, float periodo_ms){
    PR2 = (char) (periodo_ms/(4*(1.0f/_XTAL_FREQ)*16));   //mejor usar la constante_Xtal_freq 
    if (canal == 1){
        TRISCbits.TRISC2 = 1; //RC1 / CCP2 como entrada
        CCP1CONbits.P1M = 0; //PWM mode una salida
        CCP1CONbits.CCP1M = 0b1101; //Modo PWM - no hay diferencia pero asi el codigo parece mas al del ccp2
        CCPR1L = 255; //Inicio del ciclo de trabajo
        CCP1CONbits.DC1B0 = 0;  
        CCP1CONbits.DC1B1 = 0;  
    }
    else if (canal == 2){
        TRISCbits.TRISC1 = 1; //RC2 / CCP2 como entrada
        //CCP2CONbits.CCP2M = 0; //PWM mode una salida no es necesario
        CCP2CONbits.CCP2M = 0b1111;  //Modo PWM  
        CCPR2L = 0; //Inicio del ciclo de trabajo
        CCP2CONbits.DC2B0 = 0;
        CCP2CONbits.DC2B1 = 0;
    }    
    PIR1bits.TMR2IF = 0; //Limpiamos la bandera del TMR2
    T2CONbits.T2CKPS = 0b11; //prescaler 1:16  - solo son 2 bits, no 3
    T2CONbits.TMR2ON = 1; //Encendemos el TMR2
    while(PIR1bits.TMR2IF == 0); //Esperamos a que se complete un ciclo del TMR2 (hasta que la bandera del TMR2 se encienda)
    PIR1bits.TMR2IF = 0; //Bajamos la bandera del TMR2
    if (canal == 1) //Si el canal está en 1
        TRISCbits.TRISC2 = 0; //Ponemos como salida el RC2
    else 
        TRISCbits.TRISC1 = 0; //Sino ponemos como salida el RC1
    return;
}
void PWM_duty(char canal, float duty){
    if (canal == 1){
        int particion1 = (int)(4*duty/((1.0f/_XTAL_FREQ)*16)); 
        CCPR1L = (char)(-((particion1 >> 2)-69)); //Lo corremos dos bits  a la derecha para ingresarle los 8 bits más significativos
        /*if ((particion1 >> 2) <= 130){
            CCPR1L = (char)(-((particion1 >> 2)-94)); //Lo corremos dos bits  a la derecha para ingresarle los 8 bits más significativos
        }*/
        CCP1CONbits.DC1B0 = (particion1&(0b1)); //Hacemos un and 
        CCP1CONbits.DC1B1 = ((particion1>>1) &(0b1)); //Hacemos un and
    }
    else if (canal == 2){
        int particion2 = (int)(4*duty/((1.0f/_XTAL_FREQ)*16));
        CCPR2L = (char)(((particion2 >> 2)+60)); //Lo corremos dos bits  a la derecha para ingresarle los 8 bits más significativos
        /*if ((particion2 >> 2) <= 95){
            CCPR2L = (char)(((particion2 >> 2)+94)); //Lo corremos dos bits  a la derecha para ingresarle los 8 bits más significativos
        }*/
        CCP2CONbits.DC2B0 = (particion2&(0b1)); //Hacemos un and 
        CCP2CONbits.DC2B1 = ((particion2>>1) &(0b1)); //Hacemos un and
    }
    return;
}

 