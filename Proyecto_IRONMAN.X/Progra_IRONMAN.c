/* Archivo: PostLab02.c
 * Dispositivo: PIC16F887
 * Autor: Kevin Alarc�n
 * Compilador: XC8(v2.40), MPLABX V6.05
 * 
 * 
 * Programa: Utilizar los PWM del pic y crear un nuevo PWM para la intensidad de un led
 * Hardware: Potenci�metros en RA0, RA2 y RA5; 2 servo motores en RC1 y RC2, 1 led en RC3
 * 
 * Creado: 10 de abril, 2023
 * �ltima modificaci�n: 13 de abril, 2023
 */
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

//---------------------Librer�as----------------------------------
#include <xc.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <pic16f887.h>
#include "PWM_SERVO1.h"

#define _tmr0_value 0 // valor de tmr0 para que la interrupci�n sea cada 16.38ms 
#define _XTAL_FREQ 4000000 //definimos la frecuencia del oscilador
#define canal 2 //Canal 1 del PWM
#define periodo 0.020f //Periodo de 20 ms
#define periodoTotal 0.0000001f //Tendremos un periodo de 1 ms para el led
#define ciclo_trabajo 0.00035f //Ciclo de trabajo de 2 ms
#define LED_PIN PORTCbits.RC3 /// usar e0 como salida del led
#define servo1 PORTCbits.RC4 /// usar e0 como salida del servo 1
#define servo2 PORTCbits.RC5 /// usar e0 como salida del servo 1
#define aumentar PORTBbits.RB0
#define decrementar PORTBbits.RB1


unsigned char ang1, ang2, cont = 1;
//---------------------Variables---------------------------------
int i = 0;
int valorPot1 = 0;
int valorPot2 = 0;
int dutyPot = 0;
int periodoPot = 0;
int potenciometro = 0;
float potMapeado = 0;



//-------------------Prototipos de funciones------------------
void setup(void);
void duty_cicle(int ciclo);
void pulse();

//----------------------Interrupciones---------------------------
void __interrupt() isr(void) {
    if (INTCONbits.T0IF){ //Si se activa la bandera de interrupci�n del TMR0
        pulse(); //Llamaremos a nuestra funci�n de ciclo de trabajo para el led
        INTCONbits.T0IF = 0; //Limpiamos la bandera del TMR0
    }
    
    if (INTCONbits.RBIF) //Revisamos si la bandera de interrupci�n del Puerto B se enciende
    {
        //RB0 -> Aumentar el contador 
        /*if (PORTBbits.RB0 == 0){
            ang1++;
        }
        //RB1 -> Decrementar el contador
        else if (PORTBbits.RB1 == 0){
            ang1--;
        }*/
        INTCONbits.RBIF = 0; //Apagamos la bandera del puerto B
    }
    
    if (PIR1bits.ADIF) { //Si se activa la bandera de interrupcion del ADC
        if (ADCON0bits.CHS == 0b0000){ //Si est� en ADC AN0
        }
        else if (ADCON0bits.CHS == 0b0100){ //Si est� en ADC AN4
            valorPot2 = 0.37*ADRESH;
            valorPot1 = 0.37*ADRESH;
            PWM_duty(2, ciclo_trabajo*((valorPot2)/255.0f)); //Llamamos a nuestra funci�n de ciclo de trabajo
            PWM_duty(1, ciclo_trabajo*((valorPot1)/255.0f)); //Llamamos a nuestra funci�n de ciclo de trabajo
        }
        else if (ADCON0bits.CHS == 0b0010){//Si est� en ADC AN2
            ang1 = ((ADRESH*0.07058823529)+14); //Convertimos el valor del pot para que vaya de 0 a 18 y le sumamos 14 para que vaya en un rango de 14 a 32
        }
        PIR1bits.ADIF = 0; //Limpiar la bandera de la interrupcion del ADC
    }
    return; 
}

void pulse(){
    if (servo1 == 1)
        {
            TMR0 = ang1; //Le ingresamos el valor de 14 al TMR0 y colocamos el pin en bajo para que el TMR0 tarde 241 en desbordarse (en este tiempo el pin estar� en bajo)
            servo1 = 0;
        }
        else 
        {
            TMR0 = (255-ang1); //Le ingresamos el valor de (255-14) al TMR0 y colocamos el pin en alto para que el TMR0 tarde 14 en desbordarse (en este tiempo el pin estar� en alto)
            servo1 = 1;
        }
    return;
}

void main(void) {
    int Lmin = 14, Lmax = 32;
    ang1 = Lmin;
    ang2 = Lmin;
    setup (); 
    ADCON0bits.GO = 1; //Activamos la lectura del ADC
    while(1){ //loop forever
        
        /*if(aumentar){
            __delay_ms(200);
            ang1++;
        }
        if(decrementar){
            __delay_ms(200);
            ang1--;
        }*/
        //PORTD = ang1;
        ang1 = (ang1<Lmin) ? Lmin:ang1;
        ang1 = (ang1>Lmax) ? Lmax:ang1;
        
        potMapeado = (0.39215686f*potenciometro)/100; //Esta variable contendr� el valor que tenga el potenciometro en un rango (0-100) en porcentaje
        dutyPot = (int)(periodoTotal * potMapeado); //En el ciclo de trabajo ingresamos el porcentaje del potenciometro multiplicado por el total del periodo 
                                               //para que vaya incrementando/decrementando el ciclo de trabajo conforme se modifique el potenciometro
        //PORTB = (unsigned char)dutyPot; //Presentamos el valor del ciclo de trabajo en el PUERTOB para verificar el valor del ciclo conforme se var�a el potenciometro
        if (ADCON0bits.GO == 0) { // Si la lectura del ADC se desactiva
            if(ADCON0bits.CHS == 0b0000) //Revisamos si el canal esta en el AN0
            {
                ADCON0bits.CHS = 0b0100; //Si, s� est� cambiamos el ADC al canal AN4
            }
            else if(ADCON0bits.CHS == 0b0100) //Revisamos si el canal esta en el AN4
            {
                ADCON0bits.CHS = 0b0010; //Si, s� est� cambiamos el ADC al canal AN2
            }
            else if(ADCON0bits.CHS == 0b0010) //Revisamos si el canal esta en el AN2
            {
                ADCON0bits.CHS = 0b0000; //Si, s� est� cambiamos el ADC al canal AN0
            }
            __delay_us(1000); //Este es el tiempo que se dar� cada vez que se desactiva la lectura
            ADCON0bits.GO = 1; //Activamos la lectura del ADC
        }
    }
    return;
}

/*void duty_cicle(int ciclo){
    i++; //Incrementamos nuestra variable comparadora
    if (i <= ciclo){ //Si nuestra variable comparadora es menor o igual a nuestro ciclo de trabajo
        LED_PIN = 1; //Que se mantenga encendido nuestro led
    }
    else{
        LED_PIN = 0; //Sino que lo apague
        i = 0; //Reseteamos nuestra variable comparadora
    }
    return;
}*/

void setup(void){
    //definir digitales
    ANSELbits.ANS0 = 1; //Seleccionamos solo los dos pines que utilizaremos como anal�gicos
    ANSELbits.ANS2 = 1;
    ANSELbits.ANS4 = 1;
    ANSELH = 0; 
    
    //Definimos puertos que ser�n salidas
    TRISB = 0x0f;
    TRISC = 0;
    TRISD = 0;
    TRISE = 0;
    
    //Limpiamos los puertos
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    PORTE = 0;
    
    OPTION_REGbits.nRBPU = 0; //Habilitamos los PULLUPS
    IOCB = 0b01111111; //Habilitamos las interrupciones al cambiar de estaso el puerto B
    //////////////oscilador
    OSCCONbits.IRCF = 0b110 ; ///4Mhz
    OSCCONbits.SCS = 1; //Utilizar oscilados interno
    
    //PSTRCON = 0b11000000;

    /////////////// tmr0
    OPTION_REGbits.T0CS = 0; //Usar Timer0 con Fosc/4
    OPTION_REGbits.PSA = 0; //Prescaler con el Timer0
    OPTION_REGbits.PS2 = 1; //Prescaler de 64
    OPTION_REGbits.PS1 = 0;
    OPTION_REGbits.PS0 = 1;  
    TMR0 = _tmr0_value; ///VALOR INICIAL DEL TMR0
    
    /////////Banderas e interrupciones
    INTCONbits.RBIF = 0; //Apagamos la bandera del puerto B
    INTCONbits.RBIE = 1; //Habilitamos la interrupci�n en el puerto B
    INTCONbits.T0IF = 0; //interrupcion del tmr0
    INTCONbits.T0IE = 1; //Hbilitar interrupci�n del TMR0
    PIR1bits.ADIF = 0; //Apagamos la bandera del ADC
    INTCONbits.PEIE = 1; //Habilitar interrupciones perif�ricas
    PIE1bits.ADIE = 1; //Habilitar interrupciones del ADC
    INTCONbits.GIE = 1; //Habilitar interrupciones globales
    ADCON0bits.GO = 1; //Activamos la lectura del ADC
    
        //Configuraci�n ADC
    ADCON0bits.CHS = 0b0000; //Elegimos canal RA0 como inicial
    ADCON1bits.VCFG0 = 0; //Voltaje referenciado de 0V
    ADCON1bits.VCFG1 = 0; //Voltaje referenciado de 5V
    ADCON0bits.ADCS = 0b10; // Fosc/32
     
    ADCON1bits.ADFM = 0; //Justificado a la izquierda
    ADCON0bits.ADON = 1;//Encendemos el m�dulo del ADC
     __delay_ms(1); 
    
    //////Configuraci�n PWM
    PWM_config(1, periodo);
    PWM_duty(1, ciclo_trabajo);
    PWM_config(2, periodo);
    PWM_duty(2, ciclo_trabajo);
    return;
}
