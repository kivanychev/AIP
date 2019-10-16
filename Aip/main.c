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

// Battery parameters
#define SECTIONS_COUNT      12

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
#define PIN_Bat2_06         PG0
#define PIN_Bat2_07         PG1
#define PIN_Bat2_08         PG2
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
#define PIN_StartIt_out     PE5
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

#define LED1_FLASH_CNT      100

#define STRING_BUF_LEN      96
#define CMD_BUF_LEN         96

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

#define V_REF               5                               // Опорное напряжение для АЦП
#define ADC_DIVIDER         100                             // Сотые Доли Вольта
#define ADC_DIGITS          10                              // Разрядность АЦП
#define ADC_COEFF           V_REF * ADC_DIVIDER / ((1 << ADC_DIGITS) - 1)   // Для получения значения в Сотых Долях Вольта

// Коэффициенты для измеряемых параметров
#define U_AB_COEFF      207
#define U_NAGR_COEFF    207

#define I_ZAB_COEFF     210
#define I_K1_COEFF      210
#define I_K2_COEFF      210
#define I_K3_COEFF      210
#define I_UST_COEFF     210

// Коды символов клавиатуры
#define KEY_ESC         0x1B
#define KEY_ENTER       0x0D
#define KEY_BACKSPACE   0x7F
#define KEY_SPACE       0x20

// Состояния
#define ST_NORMAL       0x01
#define ST_FAILURE      0x02


//=======================================================================================================================
// MACROS
//=======================================================================================================================

#define StartConvAdc() ADCSRA |= (1<<ADSC)  
#define DEBUG2(str) if(g_Debug_2) DebugMessage(str)

//=======================================================================================================================
// LOCAL TYPES
//=======================================================================================================================

typedef unsigned char BOOL;

// Command codes after conversion from string equivalents
typedef enum {
    CMD_START_IT,
    CMD_STOP_IT,
    CMD_START_ZU,
    CMD_STOP_ZU,
    CMD_VERSION,
    CMD_GET_BOARD_ID,
    CMD_GET_BAT1,
    CMD_GET_BAT2,
    CMD_GET_UOST,
    CMD_GET_UOSN,
    CMD_SET_UZT,
    CMD_USTIR_ON,
    CMD_USTIR_OFF,
    CMD_START_DIAGN,

    CMD_LEN

} TCmdId;

// Сообщения статусов системы
typedef enum {
    MSG_READY,
    MSG_POWER_SUPPLY_FAILURE,
    MSG_CHARGER_FAILURE,
    MSG_BATTERY_FAILURE,
    MSG_COMMAND_NOT_FOUND,
    MSG_SYSTEM_ERROR,
    MSG_LOAD_ERROR
    
} TMsgId;
 
// USART states
typedef enum {
    UART_OK,
    UART_ERROR

} TUartResult;


// Command structure
typedef struct {
    char *CommandName;                              // Имя команды, которое оператор вводит в терминале
    TCmdId CmdId;                                   // ID команды для внутренних задач

} TCmd;


//=======================================================================================================================
// LOCAL VARIABLES
//=======================================================================================================================

// Message ID defined in TMsgId
char *g_Messages[] = {
  "00 ГОТОВ",
  "01 НЕИСПРАВНСТЬ ИТ!",
  "02 НЕИСПРАВНОСТЬ ЗУ!",
  "03 НЕИСПРАВНОСТЬ АБ ",
  "04 НЕИЗВЕСТНАЯ КОМАНДА!",
  "05 СИСТЕМА НЕИСПРАВНА!",
  "06 НЕИСПРАВНОСТЬ НАГРУЗКИ!"
};

TCmd g_Commands[] = {
    { "start-it",           CMD_START_IT},          // Запустить Источник Тока  
    { "stop-it",            CMD_STOP_IT},           // Остановить Источник Тока

    { "start-zu",           CMD_START_ZU},          // Запустить зарядное устройство (ЗУ)
    { "stop-zu",            CMD_STOP_ZU},           // Остановить ЗУ

    { "version",            CMD_VERSION},           // Получить версию ПО
    { "get-board-id",       CMD_GET_BOARD_ID},      // Получить id платы

    { "get-bat1",           CMD_GET_BAT1},          // Получить разово на консоли состояние секций батареи 1
    { "get-bat2",           CMD_GET_BAT2},		    // Получить разово на консоли состояние секций батареи 2
    { "get-uost",           CMD_GET_UOST},          // Получить разово на консоли  значения напряжений Uoст1, … Uост5
    { "get-uosn",           CMD_GET_UOSN},		    // Получить разово на консоли  значения напряжений Uoсн1, Uосн2
    { "set-uzt",            CMD_SET_UZT},           // Установить напряжение задания на ток (0...5000 мВ) напряжение мВ
    { "ustir-on",           CMD_USTIR_ON},          // Включить режим “Юстировка”
    { "ustir-off",          CMD_USTIR_OFF},         // Отключить режим “Юстировка”
    { "start-diagn",        CMD_START_DIAGN},       // Запустить режим “Непрерывная диагностика”

    { NULL,                 CMD_LEN}                // Маркер конца массива
};

//-------------------------------------------------
// Флаги для управления процессами работы
//-------------------------------------------------
BOOL g_Debug_1 = TRUE;
BOOL g_Debug_2 = FALSE;


volatile BOOL g_ExecuteCommand = FALSE;             // Флаг выставляется при нажатии Enter для
                                                    // исполнения принятой команды в основном цикле

volatile BOOL g_DiagnosticOn = FALSE;
volatile BOOL g_FinishDiagnostic = FALSE;

volatile BOOL g_FailureOn = FALSE;

//-------------------------------------------------
// Helper
//-------------------------------------------------

unsigned int g_led1_flash_cnt = LED1_FLASH_CNT;
char g_StrBuf[STRING_BUF_LEN];

//-------------------------------------------------
// Command buffer variables
//-------------------------------------------------
char g_CmdBuffer[CMD_BUF_LEN];
unsigned short g_CmdSymbolIndex = 0;

char g_CmdToExecute[CMD_BUF_LEN];                            // Строка команды на исполнение

//-------------------------------------------------
// Переменные для работы АЦП
//-------------------------------------------------
unsigned char g_CurrentChannel = 0;                 // Текущий номер канала АЦП (0, 1 ... 4)

//-------------------------------------------------
// Параметры, измеряемые в системе
//-------------------------------------------------

// Значения, измеренные на входах АЦП (0..5В)
volatile int  g_Uosn1;
volatile int  g_Uosn2;

volatile int  g_Uost1;
volatile int  g_Uost2;
volatile int  g_Uost3;
volatile int  g_Uost4;
volatile int  g_Uost5;

// Параметры на основе измеренных значений со входов АЦП
volatile int  g_Uab = 0;    // g_Uosn1 * 20
volatile int  g_Unagr = 0;  // g_Uosn2 * 20

volatile int  g_Uzab = 0;   // g_Uost1 - 2.5V
volatile int  g_Uk1 = 0;    // g_Uost2 - 2.5V
volatile int  g_Uk2 = 0;    // g_Uost3 - 2.5V
volatile int  g_Uk3 = 0;    // g_Uost4 - 2.5V
volatile int  g_Uust = 0;   // g_Uost5 - 2.5V

volatile int  g_Izab = 0;   // g_Uzab * KT
volatile int  g_Ik1 = 0;    // g_Uk1 * KT
volatile int  g_Ik2 = 0;    // g_Uk2 * KT
volatile int  g_Ik3 = 0;    // g_Uk3 * KT
volatile int  g_Iust = 0;   // g_Uust * KT

volatile int  g_Uzt;	    // Напряжение задания на ток в мВ с шагом 20 мВ

//-------------------------------------------------
// Состояния батарей
//-------------------------------------------------
volatile BOOL g_BatState1[12];
volatile BOOL g_BatState2[12];

volatile long  g_US8;
volatile unsigned int g_BatFailedCounter = 0;   // Задержка на включение ошибки по нагрузки при появлении помех
                                                // на входах батарей

//-------------------------------------------------
// Состояния и режимы системы
//-------------------------------------------------

volatile int g_State;
volatile int g_Mode;

//-------------------------------------------------
// Состояния ошибок
//-------------------------------------------------

volatile int g_FailureCounter = 0;

//-------------------------------------------------
// Таймеры
// Нужно задать начальное значение в тиках.
// Каждый тик составляет 100 мс.
// Значение каждой переменной таймера будет 
// уменьшаться на 1 с интервалом времени тика.
//-------------------------------------------------

volatile unsigned int g_TimerMain = 0;      // For fun in main()
volatile unsigned int g_TimerFailure = 0;   // For handling Failure
volatile unsigned int g_TimerDiagn = 0;     // Timer for diagnostic
volatile unsigned int g_TimerIT = 0;
volatile unsigned int g_TimerZU = 0;


//=======================================================================================================================
// FUNCTION PROTOTYPES
//=======================================================================================================================

void        USART0_Init();
void        USART0_SendStr(char *str);

void        Set_LED1(BOOL state);               // Pass ON/OFF parameter for LED1 on or off
void        Set_LED2(BOOL state);               // Pass ON/OFF parameter for LED2 on or off
void        Toggle_LED1();                      // Toggles LED1
void        Toggle_LED2();                      // Toggles LED2

void        GPIO_Init();

void        Timer1_Init();                      // Time counter
void        Timer3_Init();                      // PWM mode for Uzt

int         Command_Receive(unsigned char *cmdStr, unsigned char *param1, unsigned char *param2);

void        ADC_Init();
void        SendMessage(TMsgId msgId, void *param);
void        DebugMessage(char *msg);
void        DebugMessageLn(char *msg);

//------------------------------------------------------
// Функции для обработки команд от внешнего ЭБУ или ПКє
//------------------------------------------------------

void        Set_StartIt(BOOL state);            // Called for 'start-it' and 'stop-it' commands
BOOL        Get_StartIt();
void        Set_StartZU(BOOL state);            // Called for 'start-zu' and 'stop-zu' commands
BOOL        Get_StartZU();
void        Set_Uzt(unsigned int value);        // Called for 'set-uzt <value>' command
void        StartDiagnostic();                  // Called for 'start-diagn' command
void        StopDiagnostic();                   // Called for <Esc> command or key press
void        GetBat1();                          // Called for 'get-bat1; command
void        GetBat2();                          // Called for 'get-bat2; command
void        GetUost();                          // Called for 'get-uost; command
void        GetUosn();                          // Called for 'get-uosn; command

//------------------------------------------------------
// Функции для настройки работы системы
//------------------------------------------------------

void        InitializeController();
void        Failure(unsigned int delay);



//=======================================================================================================================
//                      ФУНКЦИИ НАСТРОЙКИ АППАРАТНОЙ ЧАСТИ МИКРОКОНТРОЛЛЕРА
//=======================================================================================================================



/************************************************************************/
/* Функция:     USART0_Init                                             */
/* Описание:    Настройка работы последовательного интерфейса USART     */
/* Параметры:   baudrate - скорость передачи, бит/c                     */
/************************************************************************/
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

/************************************************************************************/
/* Функция:     USART0_SendStr                                                      */
/* Описание:    Отправляет текстовую строку по последовательному интерфейсу USART   */
/* Параметры:   str -- pointer to the string to be sent                             */
/************************************************************************************/
void USART0_SendStr(char *str)
{
    static unsigned char ind;

    //    cli();
    for(ind = 0; str[ind] != '\0'; ++ind)
    {
        // wait for data to be received
        while(!(UCSR0A & (1<<UDRE0)));

        // send data
        UDR0 = str[ind];

        if(str[ind] == '\n')
        break;
    }
    //    sei();
}

/************************************************************************************/
/* Функция :        Timer1_Init                                                     */
/* Описание:        Настраивает параметры Timer1 Для отсчета системного времени     */
/*                  с шагом 10 мс                                                   */
/* Параметры:       -                                                               */
/************************************************************************************/
void Timer1_Init()
{
    /********************************/
    /* Initialize Timer0            */
    /* Measures time                */
    /* Clock prescaler CKL/256      */
    /* Operation mode: CTC: WGM=0100*/
    /* Interrupt on Compare Match   */
    /********************************/

    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS12);   //CTC mode CLK / 256
    TIMSK = (1 << OCIE1A);
    OCR1A = 625;            // 625 * 16 us = 10 000 us = 10 ms (tick time)
}

/************************************************************/
/* Функция :        Timer3_Init                             */
/* Описание:        Настраивает параметры работы Timer3     */
/* Параметры:                                               */
/************************************************************/
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

/**************************************************************************/
/* Функция:         GPIO_Init                                             */
/* Описание:        Настраивает параметры выподов микроконтроллера в      */
/*                    соответствии с назначением по                       */
/**************************************************************************/
void GPIO_Init()
{
    unsigned char tmp = 0; 
    // Initialize Battery inputs
    DDRA = tmp;     // PA0..PA7
    DDRC = tmp;     // PC0..PC7
    DDRB = tmp;     // PB0, PB4..PB7
    DDRF = tmp;     // Analog signals port, sensors
    
    PORTA = 0xFF;   // Pull-ups at Battery inputs
    PORTB = 0xFF;   // Pull-ups at Battery inputs
    PORTC = 0xFF;   // Pull-ups at Battery inputs
    PORTG = 0xFF;   // Pull-ups at Battery inputs

    // Set up LEDs as outputs
    tmp = (1 << LED1) | (1 << LED2);
    DDRG = tmp;
    
    tmp = (1 << PIN_TXD0_out) | (1 << PIN_Uzt_out) | (1 << PIN_StartIt_out) | (1 << PIN_Ustir_out) | (1 << PIN_StartZU_out);
    DDRE = tmp;
}

/****************************************************/
/* Функция:         Set_LED2                        */
/* Описание:        Устанавливает состояние VD1     */
/* Параметры:       state - новое состояние         */
/****************************************************/
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

/****************************************************/
/* Функция:         Set_LED2                        */
/* Описание:        Устанавливает состояние VD2     */
/* Параметры:       state - новое состояние         */
/****************************************************/
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

/****************************************************************/
/* Функция:         Toggle_LED1                                 */
/* Описание:        Изменяет состояние VD1 на противоположное   */
/* Параметры:                                                   */
/****************************************************************/
void Toggle_LED1()
{
    unsigned char tmp;

    tmp = PORTG;
    tmp = tmp ^ (1 << LED1);

    PORTG = tmp;
}

/****************************************************************/
/* Функция:         Toggle_LED2                                 */
/* Описание:        Изменяет состояние VD2 на противоположное   */
/* Параметры:                                                   */
/****************************************************************/
void Toggle_LED2()
{
    unsigned char tmp;

    tmp = PORTG;
    tmp = tmp ^ (1 << LED2);

    PORTG = tmp;
}

/********************************************************************/
/* Функция      ADC_Init                                            */
/* Описание:    Настроить параметры работы АЦП                      */
/********************************************************************/
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
//=======================================================================================================================
//                      ОПЕРАЦИОННЫЕ ФУНКЦИИ КОНТРОЛЛЕРА АБ
//=======================================================================================================================

/********************************************************************/
/* Функция:         Set_StartIt                                     */
/* Описание:        Устанавливает состояние выхода PIN_StartIt_out  */
/* Параметры:       state - новое состояние                         */
/********************************************************************/
void Set_StartIt(BOOL state)
{
    unsigned char tmp;

    tmp = PORTE;
    tmp = tmp & (0xFF - (1 << PIN_StartIt_out));   // Clear PIN_StartIt_out state

    if(state == ON)
    {
        tmp = tmp | (1 << PIN_StartIt_out);
    }

    PORTE = tmp;
}

/********************************************************************/
/* Функция:         Get_StartIt                                     */
/* Описание:        Читает состояние выхода PIN_StartIt_out         */
/********************************************************************/
BOOL Get_StartIt()
{
    unsigned char state;

    state = PORTE;
    state = state & (1 << PIN_StartIt_out);   // Прочитать состояние ИТ
    
    return state > 0;
}

/*********************************************************************/
/* Функция:         Set_StartZU                                      */
/* Описание:        Устанавливает состояние выхода PIN_StartZu_out   */
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

/********************************************************************/
/* Функция:         Get_StartZU                                     */
/* Описание:        Яитает состояние выхода PIN_StartIt_out         */
/* Параметры:       state - новое состояние                         */
/********************************************************************/
BOOL Get_StartZU()
{
    unsigned char state;

    state = PORTE;
    state = state & (1 << PIN_StartZU_out);   // Прочитать состояние ИТ
    
    return state > 0;
    
}

/***************************************************************************/
/* Функция:         Set_Uzt                                                */
/* Описание:        Устанавливает состояние выхода PIN_StartIt_out (Uзт)   */
/* Параметры:       value - значение в мВ с шагом 20 мВ (0...5000)         */
/***************************************************************************/
void Set_Uzt(unsigned int value)
{
	g_Uzt = value;
    OCR3A = ((value / 100) << 8) / 50;
}

/********************************************************/
/* Функция:         StartDiagnostic()                   */
/* Описание:        Called for 'start-diagn' command    */
/* Параметры:                                           */
/********************************************************/
void StartDiagnostic()
{
    g_DiagnosticOn = TRUE;
}

/****************************************************/
/* Функция:         StopDiagnostic()                */
/* Описание:        Called for <Esc> command        */
/* Параметры:                                       */
/****************************************************/
void StopDiagnostic()
{
    if(g_DiagnosticOn == TRUE)
    {
        g_DiagnosticOn = FALSE;
        g_FinishDiagnostic = TRUE;
    }    
}

/********************************************************/
/* Функция:         GetBat1                             */
/* Описание:        Called for 'get-bat1' command       */
/* Параметры:                                           */
/********************************************************/
void GetBat1()
{
    unsigned short i;
    unsigned int bat1 = 0;
    char StrBuf[STRING_BUF_LEN];

    bat1 = PINA + (( PINC & ((1 << PIN_Bat1_09) | (1 << PIN_Bat1_10) | (1 << PIN_Bat1_11) | (1 << PIN_Bat1_12)) ) << 8);
    
    sprintf(StrBuf, "@BAT1 = ");
    
    for(i = 0; i < SECTIONS_COUNT; ++i)
    {
        if( (bat1 & (1 << i)) == 0)
        {
            strcat(StrBuf, "0 ");   //Append string value
        }
        else
        {
            strcat(StrBuf, "1 ");   //Append string value
        }
    }
    
    strcat(StrBuf, ";\r\n");
    USART0_SendStr(StrBuf);
}

/********************************************************/
/* Функция:         GetBat2                             */
/* Описание:        Called for 'get-bat2' command       */
/* Параметры:                                           */
/********************************************************/
void GetBat2()
{
    unsigned short i;
    unsigned int bat2 = 0;
    char StrBuf[STRING_BUF_LEN];

    bat2 = (PINC >> 4)   |   ((PINB & ((1 << PIN_Bat2_05) | (1 << PIN_Bat2_09)  | (1 << PIN_Bat2_10) | (1 << PIN_Bat2_11) | (1 << PIN_Bat2_12))) << 4)  |  ((PING & ( (1 << PIN_Bat2_06) | (1 << PIN_Bat2_07) | (1 << PIN_Bat2_08) )) << 5);

    sprintf(StrBuf, "@BAT2 = ");
    
    for(i = 0; i < SECTIONS_COUNT; ++i)
    {
        if( (bat2 & (1 << i)) == 0)
        {
            strcat(StrBuf, "0 ");
        }
        else
        {
            strcat(StrBuf, "1 ");
        }
    }
    
    strcat(StrBuf, ";\r\n");
    USART0_SendStr(StrBuf);
}

/********************************************************/
/* Функция:         GetUost                             */
/* Описание:        Called for 'get-uost' command       */
/*                  Uost1 -- Iзаб                       */
/*                  Uost2 -- Ik1                        */
/*                  Uost3 -- Ik2                        */
/*                  Uost4 -- Ik3                        */
/*                  Uost5 -- Iюст                       */
/* Параметры:                                           */
/*                                                      */
/* Примечания:                                          */
/* (g_Uost - 250) означает вычесть 2.5В из значения     */
/* измеренного апряжения датчика тока                   */
/********************************************************/
void GetUost()
{
    char StrBuf[STRING_BUF_LEN];
    unsigned char hundreds;

    // Iзаб
    hundreds = g_Izab  % ADC_DIVIDER;
    sprintf(StrBuf, "@Iзаб=%d.%d%d", g_Izab / ADC_DIVIDER, hundreds / 10, hundreds % 10);
    strcat(StrBuf, ";  \t");
    USART0_SendStr(StrBuf);

    // Ik1 
    hundreds = g_Ik1 % ADC_DIVIDER;
    sprintf(StrBuf, "@Ik1=%d.%d%d", g_Ik1 / ADC_DIVIDER, hundreds / 10, hundreds % 10);
    strcat(StrBuf, ";  \t");
    USART0_SendStr(StrBuf);

    // Ik2  
    hundreds = g_Ik2 % ADC_DIVIDER;
    sprintf(StrBuf, "@Ik2=%d.%d%d", g_Ik2 / ADC_DIVIDER, hundreds / 10, hundreds % 10);
    strcat(StrBuf, ";  \t");
    USART0_SendStr(StrBuf);

    // Ik3
    hundreds = g_Ik3 % ADC_DIVIDER;
    sprintf(StrBuf, "@Ik3=%d.%d%d", g_Ik3 / ADC_DIVIDER, hundreds / 10, hundreds % 10);
    strcat(StrBuf, ";  \t");
    USART0_SendStr(StrBuf);

    // Iнагр = Ik1 + Ik2 + Ik3
    hundreds = ( g_Ik1 + g_Ik2 + g_Ik3 ) % ADC_DIVIDER;
    sprintf(StrBuf, "@Iнагр=%d.%d%d", ( g_Ik1 + g_Ik2 + g_Ik3 ) / ADC_DIVIDER, hundreds / 10, hundreds % 10);
    strcat(StrBuf, ";  \t");
    USART0_SendStr(StrBuf);

    // Iюст
    hundreds = g_Iust % ADC_DIVIDER;
    sprintf(StrBuf, "@Iюст=%d.%d%d", g_Iust / ADC_DIVIDER, hundreds / 10, hundreds % 10);
    strcat(StrBuf, ";    \r\n");
    USART0_SendStr(StrBuf);
}

/********************************************************/
/* Функция:         GetUosn                             */
/* Описание:        Called for 'get-uosn' command       */
/* Параметры:                                           */
/********************************************************/
void GetUosn()
{
    char StrBuf[STRING_BUF_LEN];
    unsigned char hundreds;
    
    // Uаб
    hundreds = g_Uab % ADC_DIVIDER;
    sprintf(StrBuf, "@Uаб=%d.%d%d", g_Uab / ADC_DIVIDER, hundreds / 10, hundreds % 10);
    strcat(StrBuf, ";  \t");
    USART0_SendStr(StrBuf);

    // Uнагр
    hundreds = g_Unagr % ADC_DIVIDER;
    sprintf(StrBuf, "@Uнагр=%d.%d%d", g_Unagr / ADC_DIVIDER, hundreds / 10, hundreds % 10);
    strcat(StrBuf, ";   \r\n");
    USART0_SendStr(StrBuf);
}


/********************************************************************/
/* Функция:         SendMessage                                     */
/* Описание:        Отправляет системное сообщение на ПК или ЭБУ    */
/* Параметры:       msgId - идентификатор сообщения                 */
/*                  param - указатель на параметр сообщения         */
/********************************************************************/
void SendMessage(TMsgId msgId, void *param)
{
    // Начало строки сообщения
    USART0_SendStr("\"");
    
    // Основное сообщение строки сообщения
    USART0_SendStr(g_Messages[msgId]);
    
    if(param != NULL)
    {
        switch (msgId)
        {
            case MSG_BATTERY_FAILURE:
                USART0_SendStr( (char *)param);
                break;
            
            default:
                break;
        }
    }
    
    // Завершение строки сообщения
    USART0_SendStr("\"\r\n");
}

/********************************************************************/
/* Функция:         DebugMessage                                    */
/* Описание:        Отправляет отладочные сообщения на ПК по USART0 */
/* Параметры:                                                       */
/********************************************************************/
void DebugMessage(char *msg)
{
    if(g_Debug_1)
    {
        USART0_SendStr("[");
        USART0_SendStr(msg);
        USART0_SendStr("]");
    }
}


/********************************************************************/
/* Функция:         DebugMessageLn                                  */
/* Описание:        Отправляет отладочные сообщения на ПК по USART0 */
/*                  с переводом на новую строку после текста        */
/* Параметры:       msg -- текст сообщения                          */
/********************************************************************/
void DebugMessageLn(char *msg)
{
    if(g_Debug_1)
    {
        USART0_SendStr("[");
        USART0_SendStr(msg);
        USART0_SendStr("]\r\n");
    }
}


/********************************************************************/
/* Функция:         InitializeController*/
/* Описание:        Отправляет отладочные сообщения на ПК по USART0 */
/*                  с переводом на новую строку после текста        */
/* Параметры:       msg -- текст сообщения                          */
/********************************************************************/
void InitializeController()
{
    SendMessage(MSG_READY, NULL);

    Set_StartZU(FALSE);
    Set_StartIt(FALSE);
    Set_Uzt(4000);
}

/********************************************************************/
/* Функция:         Failure                                         */
/* Описание:        Отключает ИТ и ЗУ через время delay             */
/* Параметры:       delay -- время задаржки перед отключением, мс   */
/*                           шаг времени 10 мс                      */
/********************************************************************/
void Failure(unsigned int delay)
{
    char StrBuf[STRING_BUF_LEN];
    
    sprintf(StrBuf, "Failure started for %d ms", delay);
    DebugMessageLn(StrBuf);

    g_TimerFailure = delay / 10;
    
    // Включить процедуру обработки ошибки
    g_FailureOn = TRUE;
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
    char StrBuf[STRING_BUF_LEN];
    char strUp[4];
    strUp[0] = 27;
    strUp[1] = '[';
    strUp[2] = 'A';
    strUp[3] = 0;

    unsigned short cmdIndex;

    // -----------------------------------------------------------
    // INITIALIZE
    // Настроить работу периферии контроллера. Настройка системы.
    // -----------------------------------------------------------

    GPIO_Init();
    Timer3_Init();
    Timer1_Init();

    Toggle_LED2();   
    USART0_Init(MYUBRR);
    
    ADC_Init();
    StartConvAdc();
    
    InitializeController();
    
    sei();

    g_TimerMain = 300; // 3 sec
    DebugMessageLn("Start 3 seconds count");

    while (1) 
    {
        // Проверить, работоспособность системы
        if(g_FailureCounter >= 3)
        {
            // ST_FAILURE
            // Аварийное состоние системы. 
            // Требуется устранение проблемы на аппаратном уровне и перезагрузка Контроллера АБ
            continue;
        }
        
        // Тестовые сообщения при запуске системы
        switch(g_TimerMain)
        {
            case 300:
                DebugMessageLn("3 sec");
                g_TimerMain--;
                break;
                
            case 200:
                DebugMessageLn("2 sec");
                g_TimerMain--;
                break;
                
            case 100:
                DebugMessageLn("1 sec");
                g_TimerMain--;
                break;
                
            case 1:
                DebugMessageLn("0 sec");
                g_TimerMain--;
                break;

            default:                
                break;
        }
        
        // -----------------------------------------------------------
        // HANDLE FAILURE 
        // Сформировать состояние Ошибки через заданное время
        // -----------------------------------------------------------       
        if(g_FailureOn == TRUE)
        {
            if(g_TimerFailure == 0)
            {
                DebugMessageLn("Failure finished ");
                g_FailureCounter++;
        
                if(g_FailureCounter >=3)
                {
                    // Отправить сообщение об ошибке
                    SendMessage(MSG_SYSTEM_ERROR, NULL);
                }

                // Выключить ЗУ и ИТ
                Set_StartZU(FALSE);
                Set_StartIt(FALSE);
            
                // Завершить процедуру сигнализирования об ошибке
                g_FailureOn = FALSE;
            }                
        }

        
        // -----------------------------------------------------------
        // HANDLE COMMANDS (Обработка команд)
        // -----------------------------------------------------------
        
        if(g_ExecuteCommand == TRUE)
        {
            // Найти команду
            for(cmdIndex = 0; g_Commands[cmdIndex].CommandName != NULL; ++cmdIndex)
            {
                if(strcmp(g_CmdToExecute, g_Commands[cmdIndex].CommandName) == 0)
                {
                    switch(g_Commands[cmdIndex].CmdId)
                    {
                        case CMD_START_IT:
                            Set_StartIt(ON);
                            DebugMessageLn("CMD_START_IT выполнено");
                            break;
                            
                        case CMD_STOP_IT:
                            Set_StartIt(OFF);
                            DebugMessageLn("CMD_STOP_IT");
                            break;
                            
                        case CMD_START_ZU:
                            Set_StartZU(ON);
                            DebugMessageLn("CMD_START_ZU выполнено");
                            break;
                            
                        case CMD_STOP_ZU:
                            Set_StartZU(OFF);
                            DebugMessageLn("CMD_STOP_ZU выполнено");
                            break;

                        case CMD_VERSION:
                            USART0_SendStr("@VER="FW_VERSION "\r\n");
                            break;
                            
                        case CMD_GET_BOARD_ID:
                            USART0_SendStr("@ID=" BOARD_ID "\r\n");
                            break;
                            
                        case CMD_GET_BAT1:
                            GetBat1();
                            break;
                            
                        case CMD_GET_BAT2:
                            GetBat2();
                            break;
                            
                        case CMD_GET_UOST:
                            GetUost();
                            break;
                            
                        case CMD_GET_UOSN:
                            GetUosn();
                            break;
                            
                        case CMD_SET_UZT:
                            DebugMessage("Setting Uzt to: ");

                            // Найти поле параметра команды
                            char *param = &(g_CmdToExecute[0]);
                            unsigned int value;
                            while(*param != '\0')
                            {
                                param++;
                            }
                            param++;

                            DebugMessageLn(param);

                            value = atoi(param);
                            sprintf(StrBuf, "value=%d", value);

                            DebugMessageLn(StrBuf);
                            
                            Set_Uzt(value);
                            
                            break;
                            
                        case CMD_USTIR_ON:
                            // PE6
                            break;
                            
                        case CMD_USTIR_OFF:
                            // PE6
                            break;

                        case CMD_START_DIAGN:
                            DebugMessageLn("Starting diagnostic mode!");
                            StartDiagnostic();
                            break;
                        

                        default:
                            break;
                    }

                    break;
                }
            }  
        
            if(g_Commands[cmdIndex].CommandName == NULL)
            {
                 SendMessage(MSG_COMMAND_NOT_FOUND, NULL);
            }
            
            g_ExecuteCommand = FALSE;
        }
        
        
        // -----------------------------------------------------------
        // BATTERIES STATE CONTROL (Контроль состояния батарей)
        // -----------------------------------------------------------
        
        if(Get_StartIt() == TRUE || Get_StartZU() == TRUE)
        {    
            unsigned int bat1, bat2;
            int i;
            unsigned char failedSection = 0;
        
            // Прочесть состояния батарей
            bat1 = PINA + (( PINC & ((1 << PIN_Bat1_09) | (1 << PIN_Bat1_10) | (1 << PIN_Bat1_11) | (1 << PIN_Bat1_12)) ) << 8);
            bat2 = (PINC >> 4) | (PINB << 4);
        
            // Найти неисправные секции в батареях
            for(i = 0; i < SECTIONS_COUNT; ++i)
            {
                if( (bat1 & (1 << i)) == 0)
                {
                    g_BatFailedCounter++;
                    if(g_BatFailedCounter > 10000)
                    {
                        failedSection = i + 1;
                        sprintf(StrBuf, "1:%d", failedSection);
                        SendMessage(MSG_BATTERY_FAILURE, StrBuf);
                    }                        
                }

                if( (bat2 & (1 << i)) == 0)
                {
                    g_BatFailedCounter++;
                    if(g_BatFailedCounter > 10000)
                    {
                        failedSection = i + 1;
                        sprintf(StrBuf, "2:%d", failedSection);
                        SendMessage(MSG_BATTERY_FAILURE, StrBuf);
                    }                        
                }
                
                g_BatFailedCounter = 0;
            }

            if(failedSection != 0)
            {
                // Перейти на следующий цикл работы в случае 
                // обнаруженных неисправных секций
                Failure(0);
                continue;
            }
        }
        
        // -----------------------------------------------------------
        // POWER SUPPLY STATE CONTROL (Контроль ИТ)
        // -----------------------------------------------------------
        if( Get_StartIt() == TRUE)
        {
            if( g_Uzab > 450    ||
                g_Uk1 > 450     ||
                g_Uk2 > 450     || 
                g_Uk3 > 450     || 
                g_Uust > 450)
              {
                  
              }
        }


        // -----------------------------------------------------------
        // CHARGER STATE CONTROL (Контроль ЗУ)
        // -----------------------------------------------------------
        if( Get_StartZU() == TRUE && Get_StartIt() == TRUE)
        {
            // Uab < 80 В
            // Uzt > 3 В
            // Uzab < 2.6 В
            if( (g_Uab < 8000) && (g_Uzt > 300) && (g_Uzab < 260) )
            {
/*            SendMessage(MSG_LOAD_ERROR, NULL);
                sprintf(StrBuf, "Uab = %d; zt = %d; Uzab = %d", g_Uab, g_Uzt, g_Uzab);
                DebugMessageLn(StrBuf);
                
                Failure(0); */
			}
            // Uzab > 4.5 В
			else if(g_Uzab > 450)
			{
			}
        }



        // -----------------------------------------------------------
        // CHARGER LEVEL CONTROL (Контроль уровня заряда)
        // -----------------------------------------------------------
        // Нужно отслеживать значение 
        if( Get_StartIt() == TRUE)
        {
	        if(g_Uab < 4000 || g_Uab > 6000)
			{
				SendMessage(MSG_LOAD_ERROR, NULL);
                sprintf(StrBuf, "Uab = %d", g_Uab);
                DebugMessageLn(StrBuf);
				Failure(0);
			}
        }


        // -----------------------------------------------------------
        // FINISH DIAGNOSTIC (Завершить режим диагностики)
        // -----------------------------------------------------------
        if(g_FinishDiagnostic == TRUE)
        {
            g_FinishDiagnostic = FALSE;
            USART0_SendStr("\r\n");
            USART0_SendStr("\r\n");
            USART0_SendStr("\r\n");
            USART0_SendStr("\r\n");
            USART0_SendStr("\r\n");
        }



        // -----------------------------------------------------------
        // HANDLE DIAGNOSTIC (Диагностика: непрерывное отображение измеренных параметров) 
        // -----------------------------------------------------------
        if(g_DiagnosticOn == TRUE)
        {
            if(g_TimerDiagn == 0)
            {
                g_TimerDiagn = 50;      // Set Timer for 500 ms

                // Get all data 
                GetBat1();
                GetBat2();
                GetUosn();
                GetUost();
            
                USART0_SendStr(strUp);
                USART0_SendStr(strUp);
                USART0_SendStr(strUp);
                USART0_SendStr(strUp);
            }                
        }
                
    } // while(1)
}


//=======================================================================================================================
// INTERRUPT HANDLERS
//=======================================================================================================================

/********************************************************************/
/* Функция:     Обработчик прерывания Timer1 по событию Compare A   */
/* Описание:    счетчик заданного времени в тиках                   */
/*              считает время в переменных g_TimerMain,  g_TimerFailure  и   */
/*              g_TimerDiagn                                            */
/********************************************************************/

ISR(TIMER1_COMPA_vect)
{
    g_led1_flash_cnt--;
    if(g_led1_flash_cnt == 0)
    {
        Toggle_LED2();
        g_led1_flash_cnt = LED1_FLASH_CNT;
    }

    if( g_TimerMain > 0)
    {
        g_TimerMain --;
    }
    
    if( g_TimerFailure > 0)
    {
        g_TimerFailure --;
    }

    if( g_TimerDiagn > 0)
    {
        g_TimerDiagn --;
    }

    if( g_TimerIT > 0)
    {
        g_TimerIT --;
    }

    if( g_TimerZU > 0)
    {
        g_TimerZU --;
    }

}


/****************************************************/
/* Функция:     Обработчик прерывания АЦП           */
/* Описание:    Измеряет значения всех параметров   */
/*              Измеренное на входе АЦП напряжение  */
/*              считаем в Сотых Долях Вольта        */
/****************************************************/

ISR(ADC_vect)
{
    unsigned long adcBuf;

    adcBuf = ADCL;
    adcBuf = (ADCH << 8) | adcBuf;

    switch(g_CurrentChannel)
    {
        case CHANNEL_S8:
            break;

        case CHANNEL_UOSN1:
            g_Uosn1 = (unsigned int)(adcBuf * ADC_COEFF);
            g_Uab =  (g_Uosn1 / 10 ) * U_AB_COEFF;

            sprintf(g_StrBuf, "Uosn1=%d\r\n", g_Uosn1);
            DEBUG2(g_StrBuf);
            break;

        case CHANNEL_UOSN2:
            g_Uosn2 = (unsigned int)(adcBuf * ADC_COEFF);
            g_Unagr = (g_Uosn2 / 10 ) * U_NAGR_COEFF;
            
            sprintf(g_StrBuf, "Uosn2=%d\r\n", g_Uosn2);
            DEBUG2(g_StrBuf);
            break;

        case CHANNEL_UOST5:
            g_Uost5 = (unsigned int)(adcBuf * ADC_COEFF);
            g_Uust  = g_Uost5 - 250;                        // -2.5 V
            g_Iust  = (g_Uust / 10 ) * I_UST_COEFF;

            sprintf(g_StrBuf, "Uost5=%d\r\n", g_Uost5);
            DEBUG2(g_StrBuf);
            break;
            
        case CHANNEL_UOST4:
            g_Uost4 = (unsigned int)(adcBuf * ADC_COEFF) ;
            g_Uk3   = g_Uost4 - 250;                        // -2.5 V
            g_Ik3   = (g_Uk3 / 10 ) * I_K3_COEFF;

            sprintf(g_StrBuf, "Uost4=%d\r\n", g_Uost4);
            DEBUG2(g_StrBuf);
            break;
        
        case CHANNEL_UOST3:
            g_Uost3 = (unsigned int)(adcBuf * ADC_COEFF);
            g_Uk2   = g_Uost3 - 250;                        // -2.5 V
            g_Ik2   = (g_Uk2 / 10 ) * I_K2_COEFF;

            sprintf(g_StrBuf, "Uost3=%d\r\n", g_Uost3);
            DEBUG2(g_StrBuf);
            break;
        
        case CHANNEL_UOST2:
            g_Uost2 = (unsigned int)(adcBuf * ADC_COEFF);
            g_Uk1   = g_Uost2 - 250;                        // -2.5 V
            g_Ik1   = (g_Uk1 / 10 ) * I_K1_COEFF;

            sprintf(g_StrBuf, "Uost2=%d\r\n", g_Uost2);
            DEBUG2(g_StrBuf);
            break;

        case CHANNEL_UOST1:
            g_Uost1 = (unsigned int)(adcBuf * ADC_COEFF);
            g_Uzab  = g_Uost1 - 250;                        // -2.5 V
            g_Izab  = (g_Uzab / 10 ) * I_ZAB_COEFF;

            sprintf(g_StrBuf, "Uost1=%d\r\n", g_Uost1);
            DEBUG2(g_StrBuf);
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
/* Описание:    Принимает символы по USART0 от ПК или другого внешнего устройства и */
/*                передает введенные строки на определение команды в основной цикл  */
/*                через буфер g_CmdToExecute                                        */
/************************************************************************************/

ISR(USART0_RX_vect)
{
    unsigned char ch = UDR0;
    unsigned short ind;

    switch(ch)
    {
        // Остановить режим диапностики
        case KEY_ESC:
            StopDiagnostic();
            break;
           
        // Проверить, была ли введена строка для команды
        case KEY_ENTER:
            if(g_CmdSymbolIndex == 0)
            {
                USART0_SendStr("\r\n");
                break;
            }
            
            g_CmdBuffer[g_CmdSymbolIndex] = '\0';   // Зафиксировать окончание команды
            
            // Скопировать принятую команду в строку команды для исполнения
            for(ind = 0; g_CmdBuffer[ind] != '\0' && ind < CMD_BUF_LEN; ++ind)
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
            USART0_SendStr("\r\n");         // Перевести строку на терминале
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

            if(g_CmdSymbolIndex > CMD_BUF_LEN)
            {
                g_CmdSymbolIndex--;
            }

            UDR0 = ch;
            break;
    
    }
}    
