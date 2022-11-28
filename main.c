//Proyecto Tivaware
//Juan Emilio Reyes Orantes
//20959

//librerias.
#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "inc/hw_types.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"


//Variables a usar
uint32_t i = 0;
uint32_t ADC = 0;
int cont;
uint8_t ui32Period;
uint8_t PSH1;//pushbutton1
uint8_t PSH2;//pushbutton2
int DIP1_STATUS;//switch1
int DIP2_STATUS;//switch2
uint8_t POT;//potenciometro1
uint8_t POT_unit; //unidades del potenciometro
uint8_t POT_dece; //decenas del potenciometro
uint8_t BTN_anti;//modos a utilizar
uint8_t BTN_anti2;//modos a utilizar
uint8_t display[16]={0x3F,//0
                     0x06,//1
                     0x5B,//2
                     0x4F,//3
                     0x66,//4
                     0x6D,//5
                     0x7D,//6
                     0x07,//7
                     0x7F,//8
                     0x6F,//9
                     0x77,//A
                     0x7C,//B
                     0x39,//C
                     0x5E,//D
                     0x79,//E
                     0x71};//F




//------------------------ Prototipo de funciones ------------------------------
//Funciones descargadas de canvas
void delay(uint32_t msec);
void delay1ms(void);
void InitUART(void);
void UART0ReadIntHandler(void);
void Timer0IntHandler(void);
unsigned short map(uint32_t val, uint32_t in_min, uint32_t in_max, unsigned short out_min, unsigned short out_max);
//-----
void setup(void);
void INTERRP_A(void);
void INTERRP_E(void);


int main(void)
{
    setup();
    while(1){
        UART0ReadIntHandler();
        //cambiar
        DIP1_STATUS = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4); //Verifica el estado del switch1
        DIP2_STATUS = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2); //Verifica el estado del switch2

        if ((GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) == 0) && (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) == 0)){
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, display[cont]);
        }

        else if ((GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) == 0) && (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) != 0)){
            //GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, display[contador2]);
        }

        else if ((GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) != 0) && (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) == 0)){
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, display[cont]);
        }
        else {
            ADCProcessorTrigger(ADC0_BASE, 3);                          // Iniciamos conversiÃ³n de ADC
            while(!ADCIntStatus(ADC0_BASE, 3, false)){}                 // Esperamos a que termine la conversiÃ³n
            ADCIntClear(ADC0_BASE, 3);                                  // Limpiamos bandera de interrupciÃ³n
            ADCSequenceDataGet(ADC0_BASE, 3, &ADC);              // Guardamos el valor de la lectura en una variable

            POT = map(ADC, 50, 4030, 0, 99); //Mapeo de la lectura ADC de 0 a 99
            POT_dece = POT/10;                        //obtener decenas
            POT_unit = POT-POT_dece*10;                //Obtener unidades

            //Lo mostramos en los LEDs respectivos
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, display[POT_unit]);
            //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 4);
            GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, POT_dece);

        }



    }
}


void setup(void){
    // Se setea oscilador externo de 16MHz
        SysCtlClockSet(
                SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ
                                    | SYSCTL_OSC_MAIN);  //16MHz

        // Se asigna reloj a puerto F
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
        // Se asigna reloj a puerto A
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        // Se asigna reloj a puerto B
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
        // Se asigna reloj a puerto D
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
        // Se asigna reloj a puerto D
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);


        // Se establecen como salidas los pines del puerto F, B, D
        GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,
                              GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
        GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,
                              GPIO_PIN_0 |GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
        GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

        //Se establecen como entradas los pines de los puertos F, A, E
        GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
        GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2);
        GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_3 | GPIO_PIN_4);
        GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_4);

        //Configurar tipo de entrada
        GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
        GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
        GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_3 |GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
        GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

        //Configurar intetrupcion puerto A, E
        GPIOIntRegister(GPIO_PORTA_BASE, INTERRP_A);
        GPIOIntRegister(GPIO_PORTE_BASE, INTERRP_E);
        GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_3 |GPIO_PIN_4, GPIO_FALLING_EDGE); // Configuramos la interrupciÃ³n para que se ejecute al presionar el botÃ³n
        GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
        GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_3 |GPIO_PIN_4);                     // Habilitamos la interrupciÃ³n del botÃ³n
        GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_4);


        SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);                         // Habilitamos el mÃ³dulo ADC
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);                        // Habilitamos PORTE
        GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);                        // Configuramos PE3 como entrada analÃ³gica
        ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);       // Configuramos el muestreador que utilizarÃ¡ al mÃ³dulo ADC (1 muestra por conversiÃ³n)
        ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);  // Configuramos secuencia de muestreo del ADC
        ADCSequenceEnable(ADC0_BASE, 3);                                    // Habilitamos la secuencia de muestreo del ADC
        ADCIntClear(ADC0_BASE, 3);                                          // Limpiamos bandera de interrupciÃ³n (Necesario aunque no usemos interrupciones)
        InitUART();
        // Habilitar interrupciones
        IntEnable(INT_UART0);
        // Habilitar el FIFO en 1/8 datos recibidos
        UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
        //UARTFIFODisable(UART0_BASE);
        // Habilitar interrupciÃ³n de recepciÃ³n de UART0
        UARTIntEnable(UART0_BASE, UART_INT_RX);
        // Habilitar interrupciones globales
        IntMasterEnable();


       // Se habilita el reloj para el temporizador
       SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
       // Se habilita el reloj para el temporizador
       SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

       // ConfiguraciÃ³n del Timer 0 como temporizador perï¿½odico
       TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

       // Se calcula el perÃ­odo para el temporizador (1 seg)
       ui32Period = (SysCtlClockGet()) / 2;
       // Establecer el periodo del temporizador
       TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1);

       // Se habilita la interrupciÃ³n por el TIMER0A
       IntEnable(INT_TIMER0A);
       // Se establece que exista la interrupciÃ³n por Timeout
       TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
       // Se habilitan las interrupciones Globales
       IntMasterEnable();
       // Se habilita el Timer
       TimerEnable(TIMER0_BASE, TIMER_A);


}


//**************************************************************************************************************
// FunciÃ³n para hacer delay en milisegundos --- Obtenidos de ejemplos subidos a Canvas
//**************************************************************************************************************
void delay(uint32_t msec)
{
    for (i = 0; i < msec; i++)
    {
        delay1ms();
    }

}
//**************************************************************************************************************
// FunciÃ³n para hacer delay de 1 milisegundos --- Obtenidos de ejemplos subidos a Canvas
//**************************************************************************************************************
void delay1ms(void)
{
    SysTickDisable();
    SysTickPeriodSet(16000);
    SysTickEnable();

    while ((NVIC_ST_CTRL_R & NVIC_ST_CTRL_COUNT) == 0); //Pg. 138
}

//**************************************************************************************************************
// InicializaciÃ³n de  CONF UART --- Obtenidos de ejemplos subidos a Canvas
//**************************************************************************************************************
void InitUART(void)
{
    /*Enable the GPIO Port A*/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    /*Enable the peripheral UART Module 0*/
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    /* Make the UART pins be peripheral controlled. */
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* Sets the configuration of a UART. */
    UARTConfigSetExpClk(
            UART0_BASE, SysCtlClockGet(), 115200,
            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}

void UART0ReadIntHandler(void)
{
    UARTIntClear(UART0_BASE, UART_INT_RX);

    //charIn = UARTCharGet(UART0_BASE);

    //UARTCharPut(UART0_BASE, charIn);
}

//**************************************************************************************************************
//Funcion de mapeo --- Obtenidos de ejemplos subidos a Canvas
//**************************************************************************************************************
unsigned short map(uint32_t x, uint32_t x0, uint32_t x1,
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}

//------------------- interrupciones -----------------
//OVERFLOW
void INTERRP_A (void){
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_3);                  // Clear al puerto
    delay(10);
    if((DIP1_STATUS != 0) && (DIP2_STATUS == 0)){
        cont ++;
        if(cont > 15){
            cont = 0;
        }
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, cont);
    }
}

//UNDERFLOW
void INTERRP_E (void){
    GPIOIntClear(GPIO_PORTE_BASE, GPIO_INT_PIN_4);                  // Clear al puerto
    delay(10);
    if((DIP1_STATUS != 0) && (DIP2_STATUS == 0)){
        cont --;
        if(cont < 0 ){
            cont =15;
        }
        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, cont);
    }
}

//**************************************************************************************************************
// Handler de la interrupciÃ³n del TIMER 0 - Recordar modificar el archivo tm4c123ght6pm_startup_css.c
//**************************************************************************************************************
void Timer0IntHandler(void)
{
    // Clear the timer interrupt
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Read the current state of the GPIO pin and
    // write back the opposite state
    if ((DIP1_STATUS != 0) && (DIP2_STATUS == 0)){
    }
    //TERCER CASO
    else if ((DIP1_STATUS == 0) && (DIP2_STATUS != 0)){
        //BTN_anti2 ++;
        //Como aparecerÃ¡ en el display
        if (BTN_anti == 0){
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 1);
            BTN_anti = 1;
        }
        else if (BTN_anti == 1){
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 2);
            BTN_anti = 2;
        }
        else if (BTN_anti == 2){
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 4);
            BTN_anti = 3;
        }
        else if (BTN_anti == 3){
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 8);
            BTN_anti = 4;
        }
        else if (BTN_anti == 4){
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 16);
            BTN_anti = 5;
        }
        else if (BTN_anti == 5){
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 32);
            BTN_anti = 6;
        }
        else if (BTN_anti == 6){
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 64);
            BTN_anti = 7;
        }
        else if (BTN_anti == 7){
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 128);
            BTN_anti = 0;
        }

        //Para estado 2 LEDS que irÃ¡ cada segundo
        if (BTN_anti2 == 0){
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 1);
            BTN_anti2 = 1;
        }
        else if (BTN_anti2 == 1){
            //GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 2);
            BTN_anti2 = 2;
        }
        else if (BTN_anti2 == 2){
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 2);
            BTN_anti2 = 3;
        }
        else if (BTN_anti == 3){
            //GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 8);
            BTN_anti2 = 4;
        }
        else if (BTN_anti == 4){
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 4);
            BTN_anti2 = 5;
        }
        else if (BTN_anti == 5){
            //GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 4);
            BTN_anti2 = 6;
        }
        else if (BTN_anti == 6){
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 8);
            BTN_anti2 = 7;
        }
        else if (BTN_anti == 7){
            //GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, 1);
            BTN_anti2 = 0;
        }
    }

}
