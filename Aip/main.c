/*
 * Aip.c
 *
 * Created: 04.10.2018 21:58:39
 * Author : KIvanychev
 */ 

//=======================================================================================================================
// INCLUDES
//=======================================================================================================================

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


//=======================================================================================================================
// CONSTANTS
//=======================================================================================================================

#define FW_VERSION          "2019.03.26"
#define BOARD_ID            "0001"

#define FOSC                16000000            // MCU Clock Speed
#define UART_BR             1000000             // 1 Mbit
#define MYUBRR              FOSC/16/UART_BR-1

#define TRUE                1
#define FALSE               0

#define ON                  1
#define OFF                 0

// Functional pin names for PORTA
#define PIN_Bat1_01         PA0
#define PIN_Bat1_02         PA1
#define PIN_Bat1_03         PA2
#define PIN_Bat1_04         PA3
#define PIN_Bat1_05         PA4
#define PIN_Bat1_06         PA5
#define PIN_Bat1_07         PA6
#define PIN_Bat1_08         PA7 

// Functional pin names for PORTB
#define PIN_Bat2_05         PB0
#define PIN_Bat2_06         PB1
#define PIN_Bat2_07         PB2
#define PIN_Bat2_08         PB3
#define PIN_Bat2_09         PB4
#define PIN_Bat2_10         PB5
#define PIN_Bat2_11         PB6
#define PIN_Bat2_12         PB7

// Functional pin names for PORTC
#define PIN_Bat1_09         PC0
#define PIN_Bat1_10         PC1
#define PIN_Bat1_11         PC2
#define PIN_Bat1_12         PC3
#define PIN_Bat2_01         PC4
#define PIN_Bat2_02         PC5
#define PIN_Bat2_03         PC6
#define PIN_Bat2_04         PC7

// Functional pin names for PORTD
#define PIN_Int0            PD0
#define PIN_Int1            PD1
#define PIN_RXD1            PD2
#define PIN_TXD1            PD3
#define PIN_ICP1            PD4
#define PIN_XCK1            PD5

// Functional pin names for PORTE
#define PIN_RXD0_in         PE0
#define PIN_TXD0_out        PE1
#define PIN_Function_in     PE2
#define PIN_Uzt_out         PE3
#define PIN_StartPWM_out    PE5
#define PIN_Ustir_out       PE6
#define PIN_StartZU_out     PE7

// Functional pin names for PORTF
#define PIN_S8              PF0
#define PIN_Uosn1_in        PF1
#define PIN_Uost1_in        PF2
#define PIN_Uosn2_in        PF3
#define PIN_Uost5_in        PF4
#define PIN_Uost4_in        PF5
#define PIN_Uost3_in        PF6
#define PIN_Uost2_in        PF7

// Functional pin names for PORTG
#define LED1                PG3
#define LED2                PG4


#define TIMER_PERIOD        100  // us

#define UZT_HALF            0x7F
#define UZT_ZERO            0x01

#define LED1_FLASH_CNT      500

// ADC constants
#define CHANNEL_S8          0
#define CHANNEL_UOSN1       1
#define CHANNEL_UOST1       2
#define CHANNEL_UOSN2       3
#define CHANNEL_UOST5       4
#define CHANNEL_UOST4       5
#define CHANNEL_UOST3       6
#define CHANNEL_UOST2       7
#define MAX_ADC_CHANNNEL    CHANNEL_UOST2

#define V_REF               5
#define ADC_DIGITS          10                  // Разрядность АЦП

// Коэффициенты для измеряемых параметров
#define U_OSN1_COEFF    5 / 1200
#define U_OST1_COEFF    5 / 1200
#define U_OSN2_COEFF    5 / 1200
#define U_OST5_COEFF    5 / 1200
#define U_OST4_COEFF    5 / 1200
#define U_OST3_COEFF    5 / 1200
#define U_OST2_COEFF    5 / 1200

// Коды символов клавиатуры
#define KEY_ESC         0x1B
#define KEY_ENTER       0x0D
#define KEY_BACKSPACE   0x7F
#define KEY_SPACE       0x20

// Состояния
#define ST_ 0x01


//=======================================================================================================================
// MACCROS
//=======================================================================================================================

#define StartConvAdc() ADCSRA |= (1<<ADSC)  

//=======================================================================================================================
// LOCAL TYPES
//=======================================================================================================================

typedef unsigned char BOOL;

// Command codes after conversion from string equivalents
typedef enum {
    CMD_START_PWM,
    CMD_STOP_PWM,
    CMD_START_ZU,
    CMD_STOP_ZU,
    CMD_VERSION,
    CMD_GET_BOARD_ID,
    CMD_SET_UPDATE_TIME,
    CMD_START_BAT1,
    CMD_START_BAT2,
    CMD_START_BAT12,
    CMD_GET_BAT1,
    CMD_GET_BAT2,
    CMD_GET_BAT12,
    CMD_GET_UOST,
    CMD_GET_UOSN,
    CMD_SET_UZT,
    CMD_USTIR_ON,
    CMD_USTIR_OFF,

    CMD_LEN

} TCmdId;


// 

typedef enum {
    UART_OK,
    UART_ERROR

} TUartResult;

typedef struct {
    char *CommandName;                              // Имя команды, которое оператор вводит в терминале
    TCmdId CmdId;                                   // ID команды для внутренних задач

} TCmd;


//=======================================================================================================================
// LOCAL VARIABLES
//=======================================================================================================================

TCmd g_Commands[] = {
    { "start-pwm",          CMD_START_PWM},         // Запустить ШИП
    { "stop-pwm",           CMD_STOP_PWM},          // Остановить ШИП
    { "start-zu",           CMD_START_ZU},          // Запустить зарядное устройство (ЗУ)
    { "stop-zu",            CMD_STOP_ZU},           // Остановить ЗУ
    { "version",            CMD_VERSION},           // Получить версию ПО
    { "get-board-id",       CMD_GET_BOARD_ID},      // Получить id платы

    { "set-update-time",    CMD_SET_UPDATE_TIME},   // Установить время обновления параметров, время, мс
                                                    // батареи во время работы диагностики в секундах

    { "start-bat1",         CMD_START_BAT1},        // Запустить режим вывода на экран состояний
                                                    // секций батареи 1 (останавливается <Esc>)

    { "start-bat2",         CMD_START_BAT2},        // Запустить режим вывода на экран состояний
                                                    // секций батареи 2 (останавливается <Esc>)

    { "start-bat12",        CMD_START_BAT12},       // Запустить режим вывода на экран состояний
                                                    // секций батареи 1 и батареи 2
                                                    // (останавливается <Esc>)

    { "get-bat1",           CMD_GET_BAT1},          // Получить разово на консоли состояние секций батареи 1
    { "get-bat2",           CMD_GET_BAT2},		    // Получить разово на консоли состояние секций батареи 2
    { "get-bat12",          CMD_GET_BAT12},         // Получить разово на консоли состояние секций батареи 1 и батареи 2
    { "get-uost",           CMD_GET_UOST},          // Получить разово на консоли  значения напряжений Uoст1, … Uост5
    { "get-uosn",           CMD_GET_UOSN},		    // Получить разово на консоли  значения напряжений Uoсн1, Uосн2
    { "set-uzt",            CMD_SET_UZT},           // Установить напряжение задания на ток (0...5000мВ) напряжение мВ
    { "ustir-on",           CMD_USTIR_ON},          // Включить режим “Юстировка”
    { "ustir-off",          CMD_USTIR_OFF},         // Отключить режим “Юстировка”

    { NULL,                 CMD_LEN}                // Маркер конца массива
};

// Флаги для управления процессами работы
BOOL g_Uart0_echo = TRUE;
BOOL g_Debug_measured1 = FALSE;
BOOL g_Debug_measured2 = FALSE;
BOOL g_Debug_measured3 = FALSE;

volatile BOOL g_ExecuteCommand = FALSE;             // Флаг выставляется при нажатии Enter для
                                                    // исполнения принятой команды в основном цикле

unsigned int g_led1_flash_cnt = LED1_FLASH_CNT;
char g_StrBuf[64];                                  // String for USART output buffer

// Command buffer variables
char g_CmdBuffer[64];
unsigned short g_CmdSymbolIndex = 0;

char g_CmdToExecute[64];                            // Строка команды на исполнение

// Параметры, измеряемые АЦП
volatile unsigned int g_AdcBuf;                     // Переменная для чтения результата измерения из АЦП
unsigned char g_CurrentChannel = 0;                 //Текущий номер канала АЦП (0, 1 ... 4)

volatile volatile long  g_US8;
volatile volatile long  g_Uosn1;
volatile volatile long  g_Uosn2;
volatile volatile long  g_Uost1;
volatile volatile long  g_Uost2;
volatile volatile long  g_Uost3;
volatile volatile long  g_Uost4;
volatile volatile long  g_Uost5;

// Состояния батарей
volatile BOOL g_BatState1[12];
volatile BOOL g_BatState2[12];

//=======================================================================================================================
// FUNCTION PROTOTYPES
//=======================================================================================================================

void        UART0_Init();
TUartResult UART0_SendMessage(char *msg);
void        UART0_StartEcho();
void        UART0_StopEcho();

void        Set_LED1(BOOL state);       // Pass ON/OFF parameter for LED1 on or off
void        Set_LED2(BOOL state);       // Pass ON/OFF parameter for LED2 on or off
void        Toggle_LED1();              // Toggles LED1
void        Toggle_LED2();              // Toggles LED2

void        GPIO_Init();

void        Timer1_Init();
void        Timer3_Init();              // PWM mode for Uzt

int         Command_Receive(unsigned char *cmdStr, unsigned char *param1, unsigned char *param2);

void        ADC_Init();

void        Set_StartPWM(BOOL state);
void        Set_StartZU(BOOL state);

//=======================================================================================================================
// IMPLEMENTATION
//=======================================================================================================================

/**********************************************************************************************************/
// Описание:        Включает эхо-режим для USART0
// Параметры:

void UART0_StartEcho()
{
}

/**********************************************************************************************************/
// Описание:        Выключает эхо-режим для USART0
// Параметры:

void UART0_StopEcho()
{
}

/**********************************************************************************************************/
// Описание:        Настраивает параметры Timer1 Для отсчета системного времени
// Параметры:

void Timer1_Init()
{
    /********************************/
    /* Initialize Timer0            */
    /* Measures time                */
    /* Clock prescaler CKL/8        */
    /* Operation mode: CTC: WGM=010 */
    /* Interrupt on Compare Match   */
    /********************************/

    TCCR1A = (1 << WGM11);
    TCCR1B = (1 << CS11);
    TIMSK = (1 << OCIE1A);
    OCR1A = TIMER_PERIOD * 2; // us

}

/**********************************************************************************************************/
// Функция :        Timer3_Init
// Описание:        Настраивает параметры работы Timer3
// Параметры:

void Timer3_Init()
{
    /********************************/
    /* Initialize Timer3            */
    /*                              */
    /* Clock prescaler = 3          */
    /* OC mode: clear on compare    */
    /* PWM: 8 bit, A, WGM = 0101    */
    /********************************/

    TCCR3A = (1 << COM3A1) + (1 << WGM30);
    TCCR3B = (1 << WGM32) + (1 << CS30);
    TCCR3C = 0;

    OCR3A = UZT_HALF;    // Set Chagre current to HALF of max
}

/**********************************************************************************************************/
// Функция:         GPIO_Init
// Описание:        Настраивает параметры выподов микроконтроллера в соответствии с назначением по схеме

void GPIO_Init()
{
    unsigned char tmp = 0; 
    // Initialize Battery inputs
    DDRA = tmp;     // PA0..PA7
    DDRC = tmp;     // PC0..PC7
    DDRB = tmp;     // PB0, PB4..PB7
    DDRF = tmp;     // Analog signals port, sensors
    
    // Set up LEDs as outputs
    tmp = (1 << LED1) | (1 << LED2);
    DDRG = tmp;
    
    tmp = (1 << PIN_TXD0_out) | (1 << PIN_Uzt_out) | (1 << PIN_StartPWM_out) | (1 << PIN_Ustir_out) | (1 << PIN_StartZU_out);
    DDRE = tmp;
    
}


/**********************************************************************************************************/
// Описание:        Устанавливает состояние VD1
// Параметры:       state - новое состояние

void Set_LED1(BOOL state)
{
    unsigned char tmp;

    tmp = PORTG;
    tmp = tmp & (0xFF - (1 << LED1));   // Clear LED1 state

    if(state == OFF)
    {
        tmp = tmp | (1 << LED1);
    }

    PORTG = tmp;
}

/**********************************************************************************************************/
// Описание:        Устанавливает состояние VD2
// Параметры:       state - новое состояние

void Set_LED2(BOOL state)
{
    unsigned char tmp;

    tmp = PORTG;
    tmp = tmp & (0xFF - (1 << LED2));   // Clear LED2 state

    if(state == OFF)
    {
        tmp = tmp | (1 << LED2);
    }

    PORTG = tmp;
}

/**********************************************************************************************************/
// Описание:        Изменяет состояние VD1 на противоположное
// Параметры:

void Toggle_LED1()
{
    unsigned char tmp;

    tmp = PORTG;
    tmp = tmp ^ (1 << LED1);

    PORTG = tmp;
}

/**********************************************************************************************************/
// Описание:        Изменяет состояние VD2 на противоположное
// Параметры:

void Toggle_LED2()
{
    unsigned char tmp;

    tmp = PORTG;
    tmp = tmp ^ (1 << LED2);

    PORTG = tmp;
}

/**********************************************************************************************************/
// Описание:    
// Параметры:   


/**********************************************************************************************************/
// Описание:    Настройка работы последовательного интерфейса USART
// Параметры:   baudrate - скорость передачи, бит/c

void USART0_Init(unsigned int baudrate)
{

	// параметры кадра данных: 8 Data, 1 Stop, No Parity
	// USART0 Transmitter: On
	// USART0 Mode: Asynchronous
	UCSR0A = 0x00;

    UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) + (1 << UCSZ00); // 8 bit data frame глава 20.11.4

	UBRR0H = baudrate >> 8;
	UBRR0L = baudrate;
}

/**********************************************************************************************************/
// Описание:    Передает текстовый символ по последовательному интерфейсу
// Параметры:   ch -- текстовый символ для передачи по USART

inline void USART0_SendChar(unsigned char ch)
{
	// wait for data to be received
	while(!(UCSR0A & (1<<UDRE0)));
	// send data
	UDR0 = ch; 
	
}

/**********************************************************************************************************/
// Функция:     USART0_SendStr
// Описание:    Отправляет текстовую строку по последовательному интерфейсу USART
// Параметры:   str -- pointer to the string to be sent

void USART0_SendStr(char *str)
{
    static unsigned char ind;

//    cli();
    for(ind = 0; str[ind] != '\0'; ++ind)
    {
        USART0_SendChar(str[ind]);

        if(str[ind] == '\n')
            break;
    }
//    sei();
}


/**********************************************************************************************************/
// Функция      ADC_Init()
// Описание:    Настроить параметры работы АЦП

void ADC_Init()
{
    // Initializing ADC:
    // Опорное напряжение подключить к Vcc
    // Лево-ориентированный результат,
    // включить канал AD0
    ADMUX = (1<<REFS0);

    // Turn on ADC, Single conversion mode, Enable ADC interrupts
    // Set conversion frequency to FCPU/128
    ADCSRA = (1<<ADEN) | (1<<ADSC) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);

}


/*********************************************************************/
/* Описание:        Устанавливает состояние выхода PIN_StartPWM_out  */
/* Параметры:       state - новое состояние                          */
/*********************************************************************/

void Set_StartPWM(BOOL state)
{
    unsigned char tmp;

    tmp = PORTE;
    tmp = tmp & (0xFF - (1 << PIN_StartPWM_out));   // Clear PIN_StartPWM_out state

    if(state == ON)
    {
        tmp = tmp | (1 << PIN_StartPWM_out);
    }

    PORTE = tmp;
}

/*********************************************************************/
/* Описание:        Устанавливает состояние выхода PIN_StartPWM_out  */
/* Параметры:       state - новое состояние                          */
/*********************************************************************/

void Set_StartZU(BOOL state)
{
    unsigned char tmp;

    tmp = PORTE;
    tmp = tmp & (0xFF - (1 << PIN_StartZU_out));   // Clear PIN_StartZU_out state

    if(state == ON)
    {
        tmp = tmp | (1 << PIN_StartZU_out);
    }

    PORTE = tmp;
}
/***************************************************************************/
/* Описание:        Устанавливает состояние выхода PIN_StartPWM_out        */
/* Параметры:       value - значение для регистра сравнения от 0 до 1023   */
/***************************************************************************/

void Set_Uzt(unsigned int value)
{
    OCR3A = value;
}

//=======================================================================================================================
// PROGRAM ENTRY POINT
//=======================================================================================================================

/****************************************/
/* Функция:     main                    */
/* Описание:    Точка входа программы   */
/****************************************/

int main(void)
{
    unsigned short cmdIndex;

    GPIO_Init();
    Timer3_Init();

    Toggle_LED2();   
    USART0_Init(MYUBRR);
    
    ADC_Init();
    StartConvAdc();
    
    USART0_SendStr("Give me battery\r\n");
    
    USART0_SendStr("Power supply Controller ver 0.1\r\n");
    USART0_SendStr("(c) NNTU, 2018\r\n");
    USART0_SendStr("Echo mode ON\r\n");
    USART0_SendStr("Ready\r\n");

    sei();

    while (1) 
    {
        if(g_ExecuteCommand == TRUE)
        {
            USART0_SendStr("\r\n");
            USART0_SendStr("Searching: ");
            USART0_SendStr(g_CmdToExecute);
            USART0_SendStr("\r\n");
            
            // Найти команду
            for(cmdIndex = 0; g_Commands[cmdIndex].CommandName != NULL; ++cmdIndex)
            {
                if(strcmp(g_CmdToExecute, g_Commands[cmdIndex].CommandName) == 0)
                {
                    USART0_SendStr("Command found!\r\n");
                    switch(g_Commands[cmdIndex].CmdId)
                    {
                        case CMD_START_PWM:
                            Set_StartPWM(ON);
                            USART0_SendStr("CMD_START_PWM finished!\r\n");
                            break;
                            
                        case CMD_STOP_PWM:
                            Set_StartPWM(OFF);
                            USART0_SendStr("CMD_STOP_PWM finished!\r\n");
                            break;
                            
                        case CMD_START_ZU:
                            Set_StartZU(ON);
                            USART0_SendStr("CMD_START_ZU finished!\r\n");
                            break;
                            
                        case CMD_STOP_ZU:
                            Set_StartZU(OFF);
                            USART0_SendStr("CMD_STOP_ZU finished!\r\n");
                            break;

                        case CMD_VERSION:
                            USART0_SendStr("Firmware version: " FW_VERSION "\r\n");
                            break;
                            
                        case CMD_GET_BOARD_ID:
                            USART0_SendStr("Board ID: " BOARD_ID "\r\n");
                            break;
                            
                        case CMD_SET_UPDATE_TIME:
                            break;
                            
                        case CMD_START_BAT1:
                            break;
                            
                        case CMD_START_BAT2:
                            break;
                            
                        case CMD_START_BAT12:
                            break;
                            
                        case CMD_GET_BAT1:
                            break;
                            
                        case CMD_GET_BAT2:
                            break;
                            
                        case CMD_GET_BAT12:
                            break;
                            
                        case CMD_GET_UOST:
                            break;
                            
                        case CMD_GET_UOSN:
                            break;
                            
                        case CMD_SET_UZT:
                            USART0_SendStr("Setting Uzt to: ");
                            // Найти поле параметра команды
                            char *param = &(g_CmdToExecute[0]);
                            unsigned int value;
                            while(*param != '\0')
                            {
                                param++;
                            }
                            param++;

                            USART0_SendStr(param);
                            USART0_SendStr("\r\n");
                            value = atoi(param);
                            sprintf(g_StrBuf, "value=%d", value);
                            USART0_SendStr(g_StrBuf);
                            USART0_SendStr("\r\n");
                            
                            Set_Uzt(value);
                            
                            break;
                            
                        case CMD_USTIR_ON:
                            // PE6
                            break;
                            
                        case CMD_USTIR_OFF:
                            // PE6
                            break;

                        default:
                            break;
                    }

                    break;
                }
            }  
                          
            if(g_Commands[cmdIndex].CommandName == NULL)
            {
                 USART0_SendStr("Not found!\r\n");
            }
            
            g_ExecuteCommand = FALSE;
        }
        
        
    }
}

//=======================================================================================================================
// INTERRUPT HANDLERS
//=======================================================================================================================

/*********************************************************************/
/* Функция:     Обработчик прерывания Timer1 по событию Compare A    */
/* Описание:                                                         */
/*********************************************************************/

ISR(TIMER1_COMPA_vect)
{
    g_led1_flash_cnt--;
    if(g_led1_flash_cnt == 0)
    {
        Toggle_LED2();
        g_led1_flash_cnt = LED1_FLASH_CNT;
    }

}


/*****************************************************/
/* Функция:     Обработчик прерывания АЦП            */
/* Описание:    Измеряет значения всех параметров    */
/*****************************************************/

ISR(ADC_vect)
{
    g_AdcBuf = ADCL;
    g_AdcBuf = (ADCH << 8) | g_AdcBuf;

    switch(g_CurrentChannel)
    {
        case CHANNEL_S8:
            break;

        case CHANNEL_UOSN1:
            g_Uosn1 = g_AdcBuf * U_OSN1_COEFF;
            sprintf(g_StrBuf, "Uosn1=%ld\r\n", g_Uosn1);
            if(g_Debug_measured1) USART0_SendStr(g_StrBuf);
        break;

        case CHANNEL_UOST1:
            g_Uost1 = g_AdcBuf * U_OST1_COEFF;
            sprintf(g_StrBuf, "Uost1=%ld\r\n", g_Uost1);
            if(g_Debug_measured1) USART0_SendStr(g_StrBuf);
            break;

        case CHANNEL_UOSN2:
            g_Uosn2 = g_AdcBuf * U_OSN2_COEFF;
            sprintf(g_StrBuf, "Uosn2=%ld\r\n", g_Uosn2);
            if(g_Debug_measured1) USART0_SendStr(g_StrBuf);
            break;

        case CHANNEL_UOST5:
            g_Uost5 = g_AdcBuf * U_OST5_COEFF;
            sprintf(g_StrBuf, "Uost5=%ld\r\n", g_Uost5);
            if(g_Debug_measured1) USART0_SendStr(g_StrBuf);
            break;
            
        case CHANNEL_UOST4:
            g_Uost4 = g_AdcBuf * U_OST4_COEFF;
            sprintf(g_StrBuf, "Uost4=%ld\r\n", g_Uost4);
            if(g_Debug_measured1) USART0_SendStr(g_StrBuf);
            break;
        
        case CHANNEL_UOST3:
            g_Uost3 = g_AdcBuf * U_OST3_COEFF;
            sprintf(g_StrBuf, "Uost3=%ld\r\n", g_Uost3);
            if(g_Debug_measured1) USART0_SendStr(g_StrBuf);
            break;
        
        case CHANNEL_UOST2:
            g_Uost2 = g_AdcBuf * U_OST2_COEFF;
            sprintf(g_StrBuf, "Uost2=%ld\r\n", g_Uost2);
            if(g_Debug_measured1) USART0_SendStr(g_StrBuf);
            break;

        default:
            break;

    }


    g_CurrentChannel++;
    if(g_CurrentChannel > MAX_ADC_CHANNNEL)
    {
        g_CurrentChannel = 0;
    }

    // Установить следующий измеряемый канал по очереди (по кругу 0 1 2 3 4 5 0 1 2 3 4 5 0 1 ...)
    ADMUX = ADMUX & 0b11111000;
    ADMUX = ADMUX | g_CurrentChannel;
    
    // Restarting AD conversion defore exit
    StartConvAdc();
}

/************************************************************************************/
/* Функция:     Обработчик прерывания USART RX Complete                             */
/* Описание:    Принимает символы по USART0 от ПК или другого внешнего устройства и */
/*                передает введенные строки на определение команды в основной цикл  */
/*                через буфер g_CmdToExecute                                        */
/************************************************************************************/

ISR(USART0_RX_vect)
{
    unsigned char ch = UDR0;
    unsigned short ind;

    switch(ch)
    {
        // Включить или выключить режим вывода в консоль измеренных значений АЦП
        // в их изначальном виде
        case KEY_ESC:
            if(g_Debug_measured1 == TRUE)
            {
                g_Debug_measured1 = FALSE;
            }                    
            else
            {
                g_Debug_measured1 = TRUE;
            }                    
            break;
           
        // Проверить, была ли введена строка для команды
        case KEY_ENTER:
            if(g_CmdSymbolIndex == 0)
            {
                USART0_SendStr("No Command entered\r\n");
                break;
            }
            
            g_CmdBuffer[g_CmdSymbolIndex] = '\0';   // Зафиксировать окончание команды
            
            // Скопировать принятую команду в строку команды для исполнения
            for(ind = 0; g_CmdBuffer[ind] != '\0'; ++ind)
            {
                g_CmdToExecute[ind] = g_CmdBuffer[ind];
                
                // Заменить разделитель команды и параметра на 0
                if(g_CmdToExecute[ind] == KEY_SPACE)
                {
                    g_CmdToExecute[ind] = '\0';
                }
            }
            g_CmdToExecute[ind] = '\0';
            
            g_CmdSymbolIndex = 0;           // Вернуться на начало строки буфера команды
            USART0_SendStr("\r\n");         // Перевести строку на ерминале
            g_ExecuteCommand = TRUE;        // Выставить флаг на исполнение команды
            break;

        case KEY_BACKSPACE:
            if(g_CmdSymbolIndex > 0)
            {
                g_CmdSymbolIndex--;
            }
                            
            UDR0 = ch;
            break;
                
        default:
            g_CmdBuffer[g_CmdSymbolIndex] = ch;
            g_CmdSymbolIndex++;

            UDR0 = ch;
            break;
    
    }
}    
