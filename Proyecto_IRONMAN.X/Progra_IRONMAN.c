/* Archivo: PostLab02.c
 * Dispositivo: PIC16F887
 * Autor: Kevin Alarcón
 * Compilador: XC8(v2.40), MPLABX V6.05
 * 
 * 
 * Programa: Utilizar los PWM del pic y crear un nuevo PWM para la intensidad de un led
 * Hardware: Potenciómetros en RA0, RA2 y RA5; 2 servo motores en RC1 y RC2, 1 led en RC3
 * 
 * Creado: 10 de abril, 2023
 * Última modificación: 13 de abril, 2023
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

//---------------------Librerías----------------------------------
#include <xc.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <pic16f887.h>
#include "PWM_SERVO1.h"

#define _tmr0_value 0 // valor de tmr0 para que la interrupción sea cada 16.38ms 
#define _XTAL_FREQ 4000000 //definimos la frecuencia del oscilador
#define periodo 0.020f //Periodo de 20 ms
#define ciclo_trabajo 0.00025f //Ciclo de trabajo de 2 ms
#define servo1 PORTCbits.RC4 /// usar C4 como salida del servo 1
#define servo2 PORTCbits.RC5 /// usar C5 como salida del servo 2



unsigned char servo = 1, ang1, ang2;

//---------------------Variables---------------------------------
int modo = 0;
int i = -1;
int valorPot1 = 0;
int valorPot2 = 0;
int valor_uart = 0;
int contador = 0;
char numero[2];
int valor_entero;
char caracter[2];
int flag = 0;
int bandera = 0;
int entero = 0;
uint8_t  address = 0, data = 0, potenciometro = 0;



//-------------------Prototipos de funciones------------------
void setup(void);
void initUART(void);
void TXT(void);
void pulse();
void servos(uint8_t  dato, int modo);
void write_EEPROM (uint8_t  address, uint8_t  data);
uint8_t  read_EEPROM(uint8_t  address);

//----------------------Interrupciones---------------------------
void __interrupt() isr(void) {
    //Interrupción del TMR0
    if (INTCONbits.T0IF) //Si se activa la bandera de interrupción del TMR0
    {
        pulse(); //Llamaremos a nuestra función de ciclo de trabajo para el led
        INTCONbits.T0IF = 0; //Limpiamos la bandera del TMR0
    }
    
    //Interrupción del puerto serial
    if(PIR1bits.RCIF){ //Verificamos que la bandera del EUSART esté llena (ya recibió un valor)
        TXT();
        //recibido = RCREG; //Ingresamos el dato recibido desde la hiperterminal en la variable recibido
        //RCREG es el registro que contiene el valor que ingresamos en la hiperterminal
    }

    //Interrupción del Puerto B
    if (INTCONbits.RBIF) //Revisamos si la bandera de interrupción del Puerto B se enciende
    {   //RB0 --> cambiar de modo
        if (PORTBbits.RB0 == 0){
            modo = modo + 1;
        }
        //RB1 -> Cambiar de servomotor
        else if (PORTBbits.RB1 == 0){
            servo = servo + 1;
        }
        //RB0 -> Aumentar la dirección de memoria
        else if (PORTBbits.RB2 == 0){
            address++; //Aumentamos nuestra variable de localidad
            //PORTA = 0; //Limpiamos el puerto A
            contador = 0; //Colocamos nuestra bandera de Sleep en 0
        }
        //RB3 -> Disminuir la dirección de memoria
        else if (PORTBbits.RB3 == 0){
            address--; //Disminuimos nuestra variable de localidad
            contador = 0; //Colocamos nuestra bandera de Sleep en 0
        }
        //RB4 -> Escribir en la EEPROM
        else if (PORTBbits.RB4 == 0){
            write_EEPROM (address, potenciometro); //Mandamos a llamar a nuestra función de escritra en la EEPROM
            contador = 0; //Colocamos nuestra bandera de Sleep en 0
        }
        //RB5 -> Leer le EEPROM en el Puerto D
        else if (PORTBbits.RB5 == 0){
            //PORTD = 0; //Limpiamos el puerto D
            data = read_EEPROM(address); //Mandamos a llamar a nuestra función de lectura en la EEPROM
            //PORTD = data; //Ingresamos el valor tomado de la EEPROM en el puerto D
            contador = 0; //Colocamos nuestra bandera de Sleep en 0
            servos(data, modo);
        }
        //RB6 -> Sleep
        else if (PORTBbits.RB6 == 0){
             contador = 1; //Colocamos nuestra bandera de Sleep en 1
             //PORTA = address; //Ingresamos en el puerto A el valor de la localidad en la que estamos
             //PORTC = potenciometro; //Ingresamos en el puerto C el valor del potenciómetro
        }
        INTCONbits.RBIF = 0; //Apagamos la bandera del puerto B
    }
    
    //Interrupción del ADC
    if (PIR1bits.ADIF) { //Si se activa la bandera de interrupcion del ADC
        if(modo == 0){ //Si está en 0 activamos el modo manual
            //Canal para girar ambos cañones PWM CCP1
            if (ADCON0bits.CHS == 0b0000){ //Si está en ADC AN0
                valorPot1 = 0.6*ADRESH;
                PWM_duty(1, ciclo_trabajo*((valorPot1)/255.0f)); //Llamamos a nuestra función de ciclo de trabajo
            }
            //Canal para girar el cañón 1
            else if (ADCON0bits.CHS == 0b0001){//Si está en ADC AN1
                ang1 = ((ADRESH*0.047)+6); //Convertimos el valor del pot para que vaya de 0 a 18 y le sumamos 14 para que vaya en un rango de 14 a 32
            }
            //Canal para girar el cañón 2
            else if (ADCON0bits.CHS == 0b0010){//Si está en ADC AN2
                ang2 = ((ADRESH*0.047)+6); //Bits bajos     Convertimos el valor del pot para que vaya de 0 a 3987 y le sumamos 3987 para que vaya en un rango de 3987 a 7974
            }
            //Canal para el IRONMAN PWM CCP2
            else if (ADCON0bits.CHS == 0b0011){ //Si está en ADC AN3
                valorPot2 = 0.9*ADRESH;
                PWM_duty(2, ciclo_trabajo*((ADRESH)/255.0f)); //Llamamos a nuestra función de ciclo de trabajo
            }
        }
        else if(modo == 1){ //Si está en 1 activamos el modo EEPROM
            //Canal para el IRONMAN PWM CCP2
            if (ADCON0bits.CHS == 0b0100){ //Si está en ADC AN4
                potenciometro = ADRESH;
                contador = 0; //Colocamos nuestra bandera de Sleep en 0
            }  
        }
            
        PIR1bits.ADIF = 0; //Limpiar la bandera de la interrupcion del ADC
    }
    return; 
}

void pulse(){
    if(servo == 1){
        if (servo1 == 1)
            {
                TMR0 = ang1; //Le ingresamos el valor de 14 al TMR0 y colocamos el pin en bajo para que el TMR0 tarde 241 en desbordarse (en este tiempo el pin estará en bajo)
                servo1 = 0;
            }
            else 
            {
                TMR0 = (255-ang1); //Le ingresamos el valor de (255-14) al TMR0 y colocamos el pin en alto para que el TMR0 tarde 14 en desbordarse (en este tiempo el pin estará en alto)
                servo1 = 1;
            }
    }
    else if(servo == 2){
        if (servo2 == 1)
            {
                TMR0 = ang2; //Le ingresamos el valor de 14 al TMR0 y colocamos el pin en bajo para que el TMR0 tarde 241 en desbordarse (en este tiempo el pin estará en bajo)
                servo2 = 0;
            }
            else 
            {
                TMR0 = (255-ang2); //Le ingresamos el valor de (255-14) al TMR0 y colocamos el pin en alto para que el TMR0 tarde 14 en desbordarse (en este tiempo el pin estará en alto)
                servo2 = 1;
            }
    }
    return;
}

void servos(uint8_t dato, int modo){
    if(modo == 1){
        //Canal para girar el cañón 1
        ang1 = ((dato*0.047)+6); //Convertimos el valor del pot para que vaya de 0 a 18 y le sumamos 14 para que vaya en un rango de 14 a 32
            
        //Canal para girar el cañón 2
        ang2 = ((dato*0.047)+6); //Bits bajos     Convertimos el valor del pot para que vaya de 0 a 3987 y le sumamos 3987 para que vaya en un rango de 3987 a 7974
            
        //Canal para girar ambos cañones PWM CCP1
        valorPot1 = 0.6*dato;
        PWM_duty(1, ciclo_trabajo*((valorPot1)/255.0f)); //Llamamos a nuestra función de ciclo de trabajo
            
        //Canal para el IRONMAN PWM CCP2
        //valorPot2 = 0.9*data;
        PWM_duty(2, ciclo_trabajo*((dato)/255.0f)); //Llamamos a nuestra función de ciclo de trabajo
    }
    return;
}

void main(void) {
    int Lmin = 6, Lmax = 19;
    ang1 = Lmin;
    ang2 = Lmin;
    setup (); 
    initUART();
    ADCON0bits.GO = 1; //Activamos la lectura del ADC
    while(1){ //loop forever
        //Revisamos nuestra bandera que nos indica si el valor recibido del UART ya está listo para utilizarse
        if(flag == 1){
            flag = 0; //ponemos la bandera en 0
            bandera = 1;
            valor_uart = PORTD; //Ingresamos el valor recibido en el PUERTO D
        }
       
        //Comprobación de modo sleep
        if (contador == 1){ //Revisamos si nuestra bandera de Sleep está encendida
            SLEEP(); //Mientras esté encendida pondremos al Pic en modo de reposo
        }
        
        PORTE = modo; //Ingresamos en el puerto E el modo en el que está nuestro programa
        
        if(modo > 2){ //No permitimos que el modo avance más de 2 ya que solo 3 modos tenemos contando el modo 0
            modo = 0;
        }
        
        if(servo > 2){ //No permitimos que la variable servo sea más de 2 porque solo tenemos dos servos manejados con TMR0
            servo = 1;
        }
        
        //Les colocamos los limites a los servos manejados con TMR0
        ang1 = (ang1<Lmin) ? Lmin:ang1;
        ang1 = (ang1>Lmax) ? Lmax:ang1;
        
        ang2 = (ang2<Lmin) ? Lmin:ang2;
        ang2 = (ang2>Lmax) ? Lmax:ang2;
        
        //valor_uart = PORTD; //Le ingresamos a nuestra variable UART el valor recibido en el puerto D

        if (ADCON0bits.GO == 0) { // Si la lectura del ADC se desactiva
            if(ADCON0bits.CHS == 0b0000) //Revisamos si el canal esta en el AN0
            {
                ADCON0bits.CHS = 0b0100; //Si, sí está cambiamos el ADC al canal AN4
            }
            else if(ADCON0bits.CHS == 0b0100) //Revisamos si el canal esta en el AN4
            {
                ADCON0bits.CHS = 0b0011; //Si, sí está cambiamos el ADC al canal AN3
            }
            else if(ADCON0bits.CHS == 0b0011) //Revisamos si el canal esta en el AN3
            {
                ADCON0bits.CHS = 0b0010; //Si, sí está cambiamos el ADC al canal AN2
            }
            else if(ADCON0bits.CHS == 0b0010) //Revisamos si el canal esta en el AN2
            {
                ADCON0bits.CHS = 0b0001; //Si, sí está cambiamos el ADC al canal AN1
            }
            else if(ADCON0bits.CHS == 0b0001) //Revisamos si el canal esta en el AN1
            {
                ADCON0bits.CHS = 0b0000; //Si, sí está cambiamos el ADC al canal AN0
            }
            __delay_us(1000); //Este es el tiempo que se dará cada vez que se desactiva la lectura
            ADCON0bits.GO = 1; //Activamos la lectura del ADC
        }
        
        if(bandera == 1){
            bandera = 0;
            //Canal para girar el cañón 1
            ang1 = ((valor_uart*0.047)+6); //Convertimos el valor del pot para que vaya de 0 a 18 y le sumamos 14 para que vaya en un rango de 14 a 32

            //Canal para girar el cañón 2
            ang2 = ((valor_uart*0.047)+6); //Bits bajos     Convertimos el valor del pot para que vaya de 0 a 3987 y le sumamos 3987 para que vaya en un rango de 3987 a 7974

            //Canal para girar ambos cañones PWM CCP1
            valorPot1 = 0.6*valor_uart;
            PWM_duty(1, ciclo_trabajo*((valorPot1)/255.0f)); //Llamamos a nuestra función de ciclo de trabajo

            //Canal para el IRONMAN PWM CCP2
            PWM_duty(2, ciclo_trabajo*((valor_uart)/255.0f)); //Llamamos a nuestra función de ciclo de trabajo
        }
        
    }
    return;
}

void setup(void){
    //definir digitales
    ANSELbits.ANS0 = 1; //Seleccionamos solo los dos pines que utilizaremos como analógicos
    ANSELbits.ANS1 = 1;
    ANSELbits.ANS2 = 1;
    ANSELbits.ANS3 = 1;
    ANSELbits.ANS4 = 1;
    ANSELH = 0; 
    
    //Definimos puertos que serán salidas
    TRISB = 0b11111111;
    TRISA = 0b11111111;
    TRISC = 0b11000000;
    TRISD = 0;
    TRISE = 0; 
    
    //Limpiamos los puertos
    modo = 0;
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    PORTE = 0;
    
    OPTION_REGbits.nRBPU = 0; //Habilitamos los PULLUPS
    IOCB = 0b11111111; //Habilitamos las interrupciones al cambiar de estaso el puerto B
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
    INTCONbits.RBIE = 1; //Habilitamos la interrupción en el puerto B
    INTCONbits.T0IF = 0; //interrupcion del tmr0
    INTCONbits.T0IE = 1; //Hbilitar interrupción del TMR0
    PIR1bits.ADIF = 0; //Apagamos la bandera del ADC
    INTCONbits.PEIE = 1; //Habilitar interrupciones periféricas
    PIE1bits.ADIE = 1; //Habilitar interrupciones del ADC
    INTCONbits.GIE = 1; //Habilitar interrupciones globales
    ADCON0bits.GO = 1; //Activamos la lectura del ADC
    
        //Configuración ADC
    ADCON0bits.CHS = 0b0000; //Elegimos canal RA0 como inicial
    ADCON1bits.VCFG0 = 0; //Voltaje referenciado de 0V
    ADCON1bits.VCFG1 = 0; //Voltaje referenciado de 5V
    ADCON0bits.ADCS = 0b10; // Fosc/32
     
    ADCON1bits.ADFM = 0; //Justificado a la izquierda
    ADCON0bits.ADON = 1;//Encendemos el módulo del ADC
     __delay_ms(1); 
    
    //////Configuración PWM
    PWM_config(1, periodo);
    PWM_duty(1, ciclo_trabajo);
    PWM_config(2, periodo);
    PWM_duty(2, ciclo_trabajo);
    return;
}

void initUART(void){
    //paso 1
    SPBRG = 25; //SPBRGH:SPBRG  = [(4Mhz/9600)/64]-1 = 12 ? real 9615.38
    SPBRGH = 0; //%error = (9615.38-9600)/9600 * 100 = 0.16%
    BRGH = 1;   
    BRG16 = 0;
    
    //paso 2
    CREN = 1;
    SYNC = 0;   // TXSTAbits ? habilitar transmision & TXIF = 1
    SPEN = 1;   //RCSTAbits ? apagar bandera TX
    TXSTAbits.TXEN = 1; //permite transmitir
    
    //paso 3: habilitar los 9 bits
    RCSTAbits.RX9 = 0;
    
    //paso 4
    TXEN = 1; //TXSTAbits -> habilitar transmision & TXIF = 1
    TXIF = 0; //PIRbits -> apagar bandera TX
    //C
    
    //paso 5: interrupciones
    PIE1bits.RCIE = 1; //Habilitamos interrupción de RCIF
    PIR1bits.RCIF = 0; //Limpiamos bandera del RCIF

    
    
    //paso 6: cargar 9no bit
    //paso 7: cargar 
    //return;
}

uint8_t read_EEPROM(uint8_t address){
    while(WR||RD); //Verifificar si hay algún proceso de escritura
                   //o lectura en proceso
    EEADR = address; //Accedemos a la dirección que queremos leer
    EECON1bits.EEPGD = 0; //Lectura de la EEPROM
    EECON1bits.RD = 1; //Obtenemos el dato correspondiente a la dirección 
    return EEDAT;
}

void write_EEPROM (uint8_t address, uint8_t data){
    uint8_t gieStatus;
    while (WR); //Verificar WR para saber si hay un proceso
                // de escritura en proceso
    EEADR = address; //Dirección de memoria a escribir
    EEDAT = data; //Ingresamos el datos que queremos escribir
    EECON1bits.EEPGD = 0; //Acceso a memoria de datos en la EEPROM
    EECON1bits.WREN = 1; //Habilitamos la escritura en la EEPROM
    gieStatus = GIE;
    INTCONbits.GIE = 0; //Deshabilitamos interrupciones
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1; //Iniciamos escritura
    EECON1bits.WREN = 0; //Deshabilitamos la escritura en la EEPROM
    
    INTCONbits.GIE = gieStatus; //Habilitamos las interrupciones
} 

void TXT(void)
{   
    if (modo == 2){ //Revisamos si estamos en el modo UART
        while(!PIR1bits.RCIF); //Esperar a que presione un valor en la terminal
        i++; //Incrementamos nuestra variable de localidad
        caracter[i] = RCREG; //Ingresamos en nuestra cadena caracter el valor recibido del UART
        entero = caracter[i] - '0'; //Nos devuelve el número entero al que corresponde el caracter recibido
        numero[i] = entero; //Ingresamos este numero entero en una de las localidades de nuestra cadena numero
        
        if (i == 2){ //Ya que nuestro valor recibido será máximo de 3 valores revisamos si nuestra variable de localidad ya llegó a 2 porque cuenta el 0
            i = -1; //Colocamos nuestra variable localidad en -1 para que comience en 0 cuando se incremente 
            flag = 1; //Activamos la bandera que nos indica que el valor ya fue recibido completamente
            //valor_entero = atoi(numero);
            valor_entero = numero[0]*100 + numero[1]*10 + numero[2]*1; //Mapeamos los valores de decenas, centenas y unidades
            PORTD = valor_entero;
        }
    }
    return;
}