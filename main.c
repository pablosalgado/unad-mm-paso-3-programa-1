/*
 * File:   main.c
 * Author: Pablo Salgado
 *
 * Created on April 5, 2018, 4:20 AM
 */

// CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (XT oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3/PGM pin has PGM function; low-voltage programming enabled)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 20000000      // Fosc  frequency for _delay()  library

#include "lcd_hd44780_pic16.h"

uint8_t datos[40];       // Aquí se van a guardar los 40 bits leídos del DHT11.
uint8_t temperatura;     // Aquí se va a guardar la temperatura leída.
uint8_t humedad;         // Aquí se va a guardar la humedad leída.
uint8_t alarma_t;        // Indica que se han superado los 30 grados de temp.
uint8_t alarma_h;        // Indica que se han superado el 60% de humedad.

void main(void) {  
    SSPCONbits.SSPEN = 0; // Desahabilitar I2C
    CMCON = 0x07;         // Desahabilitar comparadores
    ADCON1 = 0x06;        // Deshabilitar AD conversion
    TRISC = 0x00;
    
    /* Iniciar la subrutina del LCD*/
    LCDInit(LS_NONE);

    __delay_ms(50);
    
    /* Se hace un ciclo infinito leyendo el dispositivo DHT11 cada 50 ms.*/
    while(1) {
        /* Limpiar el LCD para escribir la nueva lectura */
        LCDClear();
        
        /* 
         * Leer la respuesta del DHT11. 0 significa que hubo algún error, 1 es
         * una lectura correcta y se puede escribir al LCD.
         */
        if (leer_dht11()) {
            /* 
             * La lectura fue correcta. Se escribe la temperatura y humedad
             * relativa en el LCD.
             */
            escribir_humedad();
            escribir_temperatura();
            
            /* Se activan las alarmas y se encienden/apagan los motores.*/
            controlar_actuadores();
        } else {
            /* 
             * Hubo algún error, se escribe en el LCD "No se encontró sensor."
             */
            escribir_error();
        }
        
        /* Esperar antes de realizar la siguiente lectura del DHT11.*/
        __delay_ms(2000);
    }
}

/* 
 * Implementa el protocolo de lectura del dispositivo DHT11.
 * 
 * @returns 0 si ocurre algún error durante el proceso.
 *          1 si se lee con éxito la respuesta del DHT11.
 */
int leer_dht11() {
    /***************************************************************************
     * 1. Enviar la señal de inicio al DHT11.
     **************************************************************************/
    
    /* 
     * Colocar un estado bajo durante al menos 18ms en el bus de datos. En este
     * caso se utiliza el bit 0 del puerto D que se configura como salida y se
     * mantiene en 0 por 20ms.
     */
    PORTDbits.RD5 = 0;
    TRISDbits.TRISD5 = 0;           
    __delay_ms(20); 
   
    /* 
     * Colocar de nuevo un estado de alta impedancia en el bus datos, para
     * esperar por la respuesta del DHT11.
     */
    TRISDbits.TRISD5 = 1;
    
    
    /***************************************************************************
     * 2. Detectar respuesta del DHT11. El DHT11 envia la respuesta colocando un
     * estado bajo en el bus de datos por un espacio de 80us, seguido de un 
     * estado alto por otros 80 us.
     **************************************************************************/
    
    /*
     * El DHT11 puede tomar entre 20 y 40us para enviar la señal de respuesta. 
     * Durante este tiempo el circuito de polorización mantiene en estado alto 
     * el bus de datos. Si se supera el tiempo de espera, entonces ser retorna
     * con error.
     */    

    /* Sin preescalamiento del temporizador */
    T1CKPS0 = 0;
    T1CKPS1 = 0;
    
    /* Iniciar el temporizador */
    TMR1L = 0x00;
    TMR1H = 0x00;    
    TMR1ON = 1;
    
    /* Se lee el puerto hasta que se detecte un 0 */
    while(PORTDbits.RD5);
    
    /* Se detiene el temporizador */
    TMR1ON = 0;
    
    /* Se determina el conteo que ha hecho el temporizador */        
    uint16_t time = TMR1L;
    time = time | (TMR1H << 8);
    
    /* 
     * Como no se hace preescalamiento del temporizador, el tiempo del tick a 
     * 20Mhz es:
     * 
     * 4/20E6 = 0.2us.
     * 
     * El tiempo máximo para que el DHT11 responda es de 40us, de modo que el 
     * conteo máximo del temporizador debe ser:
     * 
     * 40uS/0.2us = 200
     */
    if (time > 200) {
        return 0;
    }
        
    /*
     * El DHT11 ha iniciado la señal de respuesta. En este punto el DHT11 debe
     * mantener el bus de datos en estado bajo durante 80us.
     */
    TMR1L = 0x00;
    TMR1H = 0x00;    
    TMR1ON = 1;
    
    while(!PORTDbits.RD5);
    
    TMR1ON = 0;
            
    time = TMR1L;
    time = time | (TMR1H << 8);

    /* 
     * El bus debe permanecer en estado bajo por parte del DHT11 debe durar 80us, 
     * de modo que el conteo máximo del temporizador debe ser:
     * 
     * 80uS/0.2us = 400
     */
    if(time > 500) {
        return 0;
    }
    
    /*
     * El DHT ha completado la primera parte de la señal de respuesta. En este
     * punto debe mantener el bus de datos en estado alto durante 80us
     */
    TMR1L = 0x00;
    TMR1H = 0x00;    
    TMR1ON = 1;
    
    while(PORTDbits.RD5);
    
    TMR1ON = 0;
            
    time = TMR1L;
    time = time | (TMR1H << 8);

    /* 
     * El conteo máximo del temporizador debe ser:
     * 
     * 80uS/0.2us = 400
     */
    if(time > 500) {
        return 0;
    }
    
    /***************************************************************************
     * 3. El DHT comienza el envio de los datos de temperatura y humedad. Se han
     * de recibir 40 bits de información en el siguiente formato:
     * 
     * 8 bits parte entera HR
     * 8 bits parte decimal HR
     * 8 bits parte entera T
     * 8 bits parte decimal T
     * 8 bits suma de chequeo de los 4 bytes anteriores
     * 
     * El DHT 11 envía primero los bits más significativos.
     **************************************************************************/
    
    /*
     * El siguiente ciclo va a leer los 40 bits del bus de datos, de acuerdo al
     * protocolo del DHT11.
     * 
     * 1. Coloca el bus de datos en estado bajo por 50us indicando que se va a
     *    transmitir un bit
     * 2. Coloca el bus de datos en estado alto por 26-28us para indicar que el 
     *    bit es "0"
     * 3. Coloca el bus de datos en estado alto por 70us para indicar que el bit
     *    es "1"
     */
    for (uint8_t i = 0; i < 40; i++) {
        /*
         * De acuerdo al protocolo, el DHT debe mantener en estado bajo el bus
         * de datos por 50us, de modo que simplemente se espera que se presente
         * el flanco de subida, pues en este punto se sabe que el DHT11 está
         * funcionando correctamente.
         */
        while(!PORTDbits.RD5);

        /*
         * Ahora se espera por el flanco de bajada y se contabiliza el tiempo
         * que permanece en estado alto el bus de datos
         */
        TMR1L = 0x00;
        TMR1H = 0x00;    
        TMR1ON = 1;

        while(PORTDbits.RD5);

        TMR1ON = 0;

        time = TMR1L;
        time = time | (TMR1H << 8);
        
        /*
         * Se determina que el bit fue un "0" si el conteo del temporizador  se 
         * halla entre 26-28us:
         * 
         * 26us/0.2us = 130
         * 28us/0.2us = 140
         * 
         * Se determna que el bit fue un "1" si el conteo del temporizador es 
         * mayor o igual a 70us:
         * 
         * 70us/0.2us = 350
         */
        if (time < 250) {
            /* 
             * Se ha recibido un 0 dado que el conteo de microsegundos está en
             * el intervalo 26-28us
             */
            datos[i] = 0;
        } else {
            /* 
             * Se ha recibido un 1 dado que el conteo de microsegundos es de 
             * 70us
             */
            datos[i] = 1;
        }
    }
    
    /* 
     * EL primer byte  de la variable datos corresponde a la parte entera de la 
     * humedad relativa.
     */
    humedad = 0;
    
    for(uint8_t i = 0; i < 8; i++) {
        if(1 == datos[i]) {
            humedad |= (1 << (7 - i));
        }
    }

    /* 
     * El tercer byte de la variable datos corresponde a la parte entera de la 
     * temperatura.
     */
    temperatura = 0;
    
    for(uint8_t i = 0; i < 8; i++) {
        if(1 == datos[i + 16]) {
            temperatura |= (1 << (7 - i));
        }
    }

    return 1;
}

/*
 * Escribe en el LCD el valor de humedad que se ha leído del DHT11.
 */
int escribir_humedad() {    
    LCDWriteStringXY(0, 0, "H.R.:    % ");
    LCDWriteIntXY(6, 0, humedad, -1);
            
    return 1;
}

/*
 * Escribe en el LCD el valor de temperatura que se ha leído del DHT11.
 */
int escribir_temperatura() {   
    LCDWriteStringXY(0, 1, "Temp:    %0C");
    LCDWriteIntXY(6, 1, temperatura, -1);
            
    return 1;
}

/* 
 * Controla los actuadores conectados al puerto C de acuerdo a la Tabla 1.
 * 
 * RC0		LED verde encendido mientras T <= 15o.
 * RC1		LED amarillo encendido mientras 20o<= T <= 28o
 * RC2		LED rojo encendido mientras T >= 30o
 * RC3		Motor de temperatura. Enciende cuando T >= 30o y se mantiene así 
 * hasta que T <= 20o
 * RC4		Motor de humedad. Enciende cuando HR >= 60% y se mantiene así hasta 
 * que HR <= 50%
 */
int controlar_actuadores() {
    /* El byte de control a escribir en el puerto C */
    uint8_t salida = 0x00;
    
    /* Si la temperatura es menor a 17 grados se enciende el led verde */
     if (temperatura <= 15) {
         salida |= 0x01;
     }
    
    /* Si la temperatura está entre los 20 y 28 grados se enciende el led amarillo*/
    if (19 < temperatura && temperatura < 29) {
        salida |= 0x02;
    }
    
    /*
     * Si la temperatura alcanza 30 grados se enciende el led rojo y se activa
     * la alarma_t permanente que solo se desactiva al bajar la temperatura a 20
     * grados
     */
    if (temperatura > 29) {
        salida |= 0x04;
        alarma_t = 1;
    }
    
    /* 
     * Mientras la temperatura sea mayor de 20 grados se mantiene activa la
     * alarma.
     */
    if (temperatura < 20) {
        alarma_t = 0;
    }
    
    /* 
     * El motor de ventilación de temperatura se mantiene encendido mientras la 
     * alarma este acitva.
     */
    if (alarma_t) {
        salida |= 0x08;        
    }
    
    /* 
     * Si la humedad relativa alcanza el 60% se activa la alarma de humedad y se 
     * enciende el motor de ventilación de humedad.
     */
    if (humedad > 59) {
        salida |= 0x10;
        alarma_h = 1;        
    }
    
    /* 
     * Se mantiene encendido el motor de ventilación de humedad mientras la 
     * humedad sea superior al 50%
     */
    if (humedad < 50) {
        alarma_h = 0;
    } else if (alarma_h) {
        salida |= 0x10;
    }

    PORTC = salida;
    
    return 1;
}

/*
 * Escribe en el LCD "No se encuentra sensor", si no es posible la lectura del 
 * DHT11
 */
int escribir_error() {
    LCDWriteStringXY(0, 0, "No se encuentra sensor.")
    return 1 ;
}
