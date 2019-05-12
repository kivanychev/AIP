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

#define LED1_FLASH_CNT      500

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

#define V_REF               5
#define ADC_DIVIDER         100                             // ����� ���� ������
#define ADC_DIGITS          10                              // ����������� ���
#define ADC_COEFF           5 * ADC_DIVIDER / ((1 << ADC_DIGITS) - 1)   // ��� ��������� �������� � ����� ����� ������

// ������������ ��� ���������� ����������
#define U_OSN1_COEFF    1
#define U_OST1_COEFF    20

#define U_OSN2_COEFF    20
#define U_OST5_COEFF    1
#define U_OST4_COEFF    1
#define U_OST3_COEFF    1
#define U_OST2_COEFF    1

// ���� �������� ����������
#define KEY_ESC         0x1B
#define KEY_ENTER       0x0D
#define KEY_BACKSPACE   0x7F
#define KEY_SPACE       0x20

// ���������
#define ST_NORMAL       0x01
#define ST_FAILURE      0x02


//=======================================================================================================================
// MACROS
//=======================================================================================================================

#define StartConvAdc() ADCSRA |= (1<<ADSC)  

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

typedef enum {
    MSG_READY,
    MSG_POWER_SUPPLY_FAILURE,
    MSG_CHARGER_FAILURE,
    MSG_BATTERY_FAILURE,
    MSG_COMMAND_NOT_FOUND
    
} TMsgId;


// 

typedef enum {
    UART_OK,
    UART_ERROR

} TUartResult;

typedef struct {
    char *CommandName;                              // ��� �������, ������� �������� ������ � ���������
    TCmdId CmdId;                                   // ID ������� ��� ���������� �����

} TCmd;


//=======================================================================================================================
// LOCAL VARIABLES
//=======================================================================================================================

char *g_Messages[] = {
  "00 READY",
  "01 POWER SUPPLY FAILURE!",
  "02 CHARGER FAILURE!",
  "03 BATTERY FAILURE ",
  "04 COMMAND NOT FOUND!"
};

TCmd g_Commands[] = {
    { "start-it",           CMD_START_IT},          // ��������� �������� ����  
    { "stop-it",            CMD_STOP_IT},           // ���������� �������� ����

    { "start-zu",           CMD_START_ZU},          // ��������� �������� ���������� (��)
    { "stop-zu",            CMD_STOP_ZU},           // ���������� ��

    { "version",            CMD_VERSION},           // �������� ������ ��
    { "get-board-id",       CMD_GET_BOARD_ID},      // �������� id �����

    { "get-bat1",           CMD_GET_BAT1},          // �������� ������ �� ������� ��������� ������ ������� 1
    { "get-bat2",           CMD_GET_BAT2},		    // �������� ������ �� ������� ��������� ������ ������� 2
    { "get-uost",           CMD_GET_UOST},          // �������� ������ �� �������  �������� ���������� Uo��1, � U���5
    { "get-uosn",           CMD_GET_UOSN},		    // �������� ������ �� �������  �������� ���������� Uo��1, U���2
    { "set-uzt",            CMD_SET_UZT},           // ���������� ���������� ������� �� ��� (0...5000��) ���������� ��
    { "ustir-on",           CMD_USTIR_ON},          // �������� ����� �����������
    { "ustir-off",          CMD_USTIR_OFF},         // ��������� ����� �����������
    { "start-diagn",        CMD_START_DIAGN},       // ��������� ����� ������������ ������������

    { NULL,                 CMD_LEN}                // ������ ����� �������
};

//-------------------------------------------------
// ����� ��� ���������� ���������� ������
//-------------------------------------------------
BOOL g_Debug_1 = TRUE;
BOOL g_Debug_2 = FALSE;

volatile BOOL g_ExecuteCommand = FALSE;             // ���� ������������ ��� ������� Enter ���
                                                    // ���������� �������� ������� � �������� �����
                                                    
volatile BOOL g_DiagnosticOn = FALSE;
volatile BOOL g_FinishDiagnostic = FALSE;
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

char g_CmdToExecute[CMD_BUF_LEN];                            // ������ ������� �� ����������

//-------------------------------------------------
// ���������� ��� ������ ���
//-------------------------------------------------
unsigned char g_CurrentChannel = 0;                 // ������� ����� ������ ��� (0, 1 ... 4)

//-------------------------------------------------
// ���������, ���������� � �������
//-------------------------------------------------

volatile int  g_Uosn1;
volatile int  g_Uosn2;

volatile int  g_Uost1;
volatile int  g_Uost2;
volatile int  g_Uost3;
volatile int  g_Uost4;
volatile int  g_Uost5;

//-------------------------------------------------
// ��������� �������
//-------------------------------------------------
volatile BOOL g_BatState1[12];
volatile BOOL g_BatState2[12];

volatile volatile long  g_US8;

//-------------------------------------------------
// ��������� � ������ �������
//-------------------------------------------------

volatile int g_State;
volatile int g_Mode;

//=======================================================================================================================
// FUNCTION PROTOTYPES
//=======================================================================================================================

void        USART0_Init();
void        USART0_SendStr(char *str);
void        USART0_StartEcho();
void        USART0_StopEcho();

void        Set_LED1(BOOL state);               // Pass ON/OFF parameter for LED1 on or off
void        Set_LED2(BOOL state);               // Pass ON/OFF parameter for LED2 on or off
void        Toggle_LED1();                      // Toggles LED1
void        Toggle_LED2();                      // Toggles LED2

void        GPIO_Init();

void        Timer1_Init();
void        Timer3_Init();                      // PWM mode for Uzt

int         Command_Receive(unsigned char *cmdStr, unsigned char *param1, unsigned char *param2);

void        ADC_Init();
void        SendMessage(TMsgId msgId, void *param);
void        DebugMessage(char *msg);
void        DebugMessageLn(char *msg);

//------------------------------------------------------
// ������� ��� ��������� ������ �� �������� ��� ��� ��:
//------------------------------------------------------

void        Set_StartIt(BOOL state);            // Called for 'start-it' and 'stop-it' commands
void        Set_StartZU(BOOL state);            // Called for 'start-zu' and 'stop-zu' commands
void        Set_Uzt(unsigned int value);        // Called for 'set-uzt <value>' command
void        StartDiagnostic();                  // Called for 'start-diagn' command
void        StopDiagnostic();                   // Called for <Esc> command or key press
void        GetBat1();                          // Called for 'get-bat1; command
void        GetBat2();                          // Called for 'get-bat2; command
void        GetUost();                          // Called for 'get-uost; command
void        GetUosn();                          // Called for 'get-uosn; command

//------------------------------------------------------
// ������� ��� ��������� ������ �� �������� ��� ��� ��:
//------------------------------------------------------

//=======================================================================================================================
// IMPLEMENTATION
//=======================================================================================================================


/************************************************************************/
/* �������:     USART0_Init                                             */
/* ��������:    ��������� ������ ����������������� ���������� USART     */
/* ���������:   baudrate - �������� ��������, ���/c                     */
/************************************************************************/
void USART0_Init(unsigned int baudrate)
{

    // ��������� ����� ������: 8 Data, 1 Stop, No Parity
    // USART0 Transmitter: On
    // USART0 Mode: Asynchronous
    UCSR0A = 0x00;

    UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) + (1 << UCSZ00); // 8 bit data frame ����� 20.11.4

    UBRR0H = baudrate >> 8;
    UBRR0L = baudrate;
}

/************************************************************************************/
/* �������:     USART0_SendStr                                                      */
/* ��������:    ���������� ��������� ������ �� ����������������� ���������� USART   */
/* ���������:   str -- pointer to the string to be sent                             */
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

/********************************************************/
/* ������� :        USART0_StartEcho                    */
/* ��������:        �������� ���-����� ��� USART0       */
/* ���������:                                           */
/********************************************************/
void USART0_StartEcho()
{
}

/********************************************************/
/* ������� :        USART0_StopEcho                      */
/* ��������:        ��������� ���-����� ��� USART0      */
/* ���������:                                           */
/********************************************************/
void USART0_StopEcho()
{
}

/************************************************************************************/
/* ������� :        Timer1_Init                                                     */
/* ��������:        ����������� ��������� Timer1 ��� ������� ���������� �������     */
/* ���������:                                                                       */
/************************************************************************************/
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

/************************************************************/
/* ������� :        Timer3_Init                             */
/* ��������:        ����������� ��������� ������ Timer3     */
/* ���������:                                               */
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

/************************************************************************************************************/
/* �������:         GPIO_Init                                                                               */
/* ��������:        ����������� ��������� ������� ���������������� � ������������ � ����������� �� �����    */
/************************************************************************************************************/
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
    
    // Set up LEDs as outputs
    tmp = (1 << LED1) | (1 << LED2);
    DDRG = tmp;
    
    tmp = (1 << PIN_TXD0_out) | (1 << PIN_Uzt_out) | (1 << PIN_StartIt_out) | (1 << PIN_Ustir_out) | (1 << PIN_StartZU_out);
    DDRE = tmp;
}

/****************************************************/
/* �������:         Set_LED2                        */
/* ��������:        ������������� ��������� VD1     */
/* ���������:       state - ����� ���������         */
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
/* �������:         Set_LED2                        */
/* ��������:        ������������� ��������� VD2     */
/* ���������:       state - ����� ���������         */
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
/* �������:         Toggle_LED1                                 */
/* ��������:        �������� ��������� VD1 �� ���������������   */
/* ���������:                                                   */
/****************************************************************/
void Toggle_LED1()
{
    unsigned char tmp;

    tmp = PORTG;
    tmp = tmp ^ (1 << LED1);

    PORTG = tmp;
}

/****************************************************************/
/* �������:         Toggle_LED2                                 */
/* ��������:        �������� ��������� VD2 �� ���������������   */
/* ���������:                                                   */
/****************************************************************/
void Toggle_LED2()
{
    unsigned char tmp;

    tmp = PORTG;
    tmp = tmp ^ (1 << LED2);

    PORTG = tmp;
}

/********************************************************************/
/* �������      ADC_Init                                            */
/* ��������:    ��������� ��������� ������ ���                      */
/********************************************************************/
void ADC_Init()
{
    // Initializing ADC:
    // ������� ���������� ���������� � Vcc
    // ����-��������������� ���������,
    // �������� ����� AD0
    ADMUX = (1<<REFS0);

    // Turn on ADC, Single conversion mode, Enable ADC interrupts
    // Set conversion frequency to FCPU/128
    ADCSRA = (1<<ADEN) | (1<<ADSC) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
}


/********************************************************************/
/* �������:         Set_StartIt                                     */
/* ��������:        ������������� ��������� ������ PIN_StartIt_out  */
/* ���������:       state - ����� ���������                         */
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

/*********************************************************************/
/* �������:                                                          */
/* ��������:        ������������� ��������� ������ PIN_StartIt_out   */
/* ���������:       state - ����� ���������                          */
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
/* �������:         Set_Uzt                                                */
/* ��������:        ������������� ��������� ������ PIN_StartIt_out (U��)   */
/* ���������:       value - �������� � �� � ����� 20 �� (0...5000)         */
/***************************************************************************/
void Set_Uzt(unsigned int value)
{
    OCR3A = ((value / 100) << 8) / 50;
}

/********************************************************/
/* �������:         StartDiagnostic()                   */
/* ��������:        Called for 'start-diagn' command    */
/* ���������:                                           */
/********************************************************/
void StartDiagnostic()
{
    g_DiagnosticOn = TRUE;
}

/****************************************************/
/* �������:         StopDiagnostic()                */
/* ��������:        Called for <Esc> command        */
/* ���������:                                       */
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
/* �������:         GetBat1                             */
/* ��������:        Called for 'get-bat1' command       */
/* ���������:                                           */
/********************************************************/
void GetBat1()
{
    unsigned short i;
    unsigned int bat1 = 0;
    char StrBuf[STRING_BUF_LEN];

    bat1 = PINA + (( PINC & ((1 << PIN_Bat1_09) | (1 << PIN_Bat1_10) | (1 << PIN_Bat1_11) | (1 << PIN_Bat1_12)) ) << 8);
    
    sprintf(StrBuf, "@BAT1 = ");
    
    for(i = 0; i < 12; ++i)
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
/* �������:         GetBat2                             */
/* ��������:        Called for 'get-bat2' command       */
/* ���������:                                           */
/********************************************************/
void GetBat2()
{
    unsigned short i;
    unsigned int bat2 = 0;
    char StrBuf[STRING_BUF_LEN];

    bat2 = (PINC >> 4) | (PINB << 4);
    
    sprintf(StrBuf, "@BAT2 = ");
    
    for(i = 0; i < 12; ++i)
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
/* �������:         GetUost                             */
/* ��������:        Called for 'get-uost' command       */
/* ���������:                                           */
/********************************************************/
void GetUost()
{
    char StrBuf[STRING_BUF_LEN];
    unsigned char hundreds;
    
    // Uost1
    hundreds = g_Uost1 % ADC_DIVIDER;
    sprintf(StrBuf, "@Uost1=%d.%d%d", g_Uost1 / ADC_DIVIDER, hundreds / 10, hundreds % 10);
    strcat(StrBuf, ";\t");
    USART0_SendStr(StrBuf);

    // Uost2
    hundreds = g_Uost2 % ADC_DIVIDER;
    sprintf(StrBuf, "@Uost2=%d.%d%d", g_Uost2 / ADC_DIVIDER, hundreds / 10, hundreds % 10);
    strcat(StrBuf, ";\t");
    USART0_SendStr(StrBuf);

    // Uost3
    hundreds = g_Uost3 % ADC_DIVIDER;
    sprintf(StrBuf, "@Uost3=%d.%d%d", g_Uost3 / ADC_DIVIDER, hundreds / 10, hundreds % 10);
    strcat(StrBuf, ";\t");
    USART0_SendStr(StrBuf);

    // Uost4
    hundreds = g_Uost4 % ADC_DIVIDER;
    sprintf(StrBuf, "@Uost4=%d.%d%d", g_Uost4 / ADC_DIVIDER, hundreds / 10, hundreds % 10);
    strcat(StrBuf, ";\t");
    USART0_SendStr(StrBuf);

    // Uost5
    hundreds = g_Uost5 % ADC_DIVIDER;
    sprintf(StrBuf, "@Uost5=%d.%d%d", g_Uost5 / ADC_DIVIDER, hundreds / 10, hundreds % 10);
    strcat(StrBuf, ";\r\n");
    USART0_SendStr(StrBuf);
}

/********************************************************/
/* �������:         GetUosn                             */
/* ��������:        Called for 'get-uosn' command       */
/* ���������:                                           */
/********************************************************/
void GetUosn()
{
    char StrBuf[STRING_BUF_LEN];
    unsigned char hundreds;
    
    // Uosn1
    hundreds = g_Uosn1 % ADC_DIVIDER;
    sprintf(StrBuf, "@Uosn1=%d.%d%d", g_Uosn1 / ADC_DIVIDER, hundreds / 10, hundreds % 10);
    strcat(StrBuf, ";\t");
    USART0_SendStr(StrBuf);

    // Uosn2
    hundreds = g_Uosn2 % ADC_DIVIDER;
    sprintf(StrBuf, "@Uosn2=%d.%d%d", g_Uosn2 / ADC_DIVIDER, hundreds / 10, hundreds % 10);
    strcat(StrBuf, ";\r\n");
    USART0_SendStr(StrBuf);
}


/********************************************************************/
/* �������:         SendMessage                                     */
/* ��������:        ���������� ��������� ��������� �� �� ��� ���    */
/* ���������:       msgId - ������������� ���������                 */
/*                  param - ��������� �� �������� ���������         */
/********************************************************************/
void SendMessage(TMsgId msgId, void *param)
{
    USART0_SendStr("\"");
    USART0_SendStr(g_Messages[msgId]);
    
    if(param != NULL)
    {
        
    }
    USART0_SendStr("\"\r\n");
}

/********************************************************************/
/* �������:         DebugMessage                                    */
/* ��������:        ���������� ���������� ��������� �� �� �� USART0 */
/* ���������:                                                       */
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
/* �������:         DebugMessageLn                                  */
/* ��������:        ���������� ���������� ��������� �� �� �� USART0 */
/*                  � ��������� �� ����� ������ ����� ������        */
/* ���������:       msg -- ����� ���������                          */
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

//=======================================================================================================================
// PROGRAM ENTRY POINT
//=======================================================================================================================

/****************************************/
/* �������:     main                    */
/* ��������:    ����� ����� ���������   */
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

    GPIO_Init();
    Timer3_Init();

    Toggle_LED2();   
    USART0_Init(MYUBRR);
    
    ADC_Init();
    StartConvAdc();
    
    SendMessage(MSG_READY, NULL);

    sei();

    while (1) 
    {
        // -----------------------------------------------------------
        // HANDLE COMMANDS (��������� ������)
        // -----------------------------------------------------------
        
        if(g_ExecuteCommand == TRUE)
        {
            // ����� �������
            for(cmdIndex = 0; g_Commands[cmdIndex].CommandName != NULL; ++cmdIndex)
            {
                if(strcmp(g_CmdToExecute, g_Commands[cmdIndex].CommandName) == 0)
                {
                    switch(g_Commands[cmdIndex].CmdId)
                    {
                        case CMD_START_IT:
                            Set_StartIt(ON);
                            DebugMessageLn("CMD_START_IT finished!");
                            break;
                            
                        case CMD_STOP_IT:
                            Set_StartIt(OFF);
                            DebugMessageLn("CMD_STOP_IT finished!");
                            break;
                            
                        case CMD_START_ZU:
                            Set_StartZU(ON);
                            DebugMessageLn("CMD_START_ZU finished!");
                            break;
                            
                        case CMD_STOP_ZU:
                            Set_StartZU(OFF);
                            DebugMessageLn("CMD_STOP_ZU finished!");
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

                            // ����� ���� ��������� �������
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
        // POWER SUPPLY STATE CONTROL (�������� ��)
        // -----------------------------------------------------------



        // -----------------------------------------------------------
        // CHARGER STATE CONTROL (�������� ��)
        // -----------------------------------------------------------





        // -----------------------------------------------------------
        // CHARGER LEVEL CONTROL (�������� ������ ������)
        // -----------------------------------------------------------
        // ����� ����������� �������� 





        // -----------------------------------------------------------
        // FINISH DIAGNOSTIC
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
        // HANDLE DIAGNOSTIC (�����������)
        // -----------------------------------------------------------
        if(g_DiagnosticOn == TRUE)
        {
            // Check if there is FAILURE STATE
                        
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
}


//=======================================================================================================================
// INTERRUPT HANDLERS
//=======================================================================================================================

/*********************************************************************/
/* �������:     ���������� ���������� Timer1 �� ������� Compare A    */
/* ��������:                                                         */
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
/* �������:     ���������� ���������� ���            */
/* ��������:    �������� �������� ���� ����������    */
/*****************************************************/

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
            g_Uosn1 = (unsigned int)(adcBuf * ADC_COEFF) * U_OSN1_COEFF;
            sprintf(g_StrBuf, "Uosn1=%d\r\n", g_Uosn1);
            if(g_Debug_2) USART0_SendStr(g_StrBuf);
        break;

        case CHANNEL_UOST1:
            g_Uost1 = (unsigned int)(adcBuf * ADC_COEFF) * U_OST1_COEFF;
            sprintf(g_StrBuf, "Uost1=%d\r\n", g_Uost1);
            if(g_Debug_2) USART0_SendStr(g_StrBuf);
            break;

        case CHANNEL_UOSN2:
            g_Uosn2 = (unsigned int)(adcBuf * ADC_COEFF) * U_OSN2_COEFF;
            sprintf(g_StrBuf, "Uosn2=%d\r\n", g_Uosn2);
            if(g_Debug_2) USART0_SendStr(g_StrBuf);
            break;

        case CHANNEL_UOST5:
            g_Uost5 = (unsigned int)(adcBuf * ADC_COEFF) * U_OST5_COEFF;
            sprintf(g_StrBuf, "Uost5=%d\r\n", g_Uost5);
            if(g_Debug_2) USART0_SendStr(g_StrBuf);
            break;
            
        case CHANNEL_UOST4:
            g_Uost4 = (unsigned int)(adcBuf * ADC_COEFF) * U_OST4_COEFF;
            sprintf(g_StrBuf, "Uost4=%d\r\n", g_Uost4);
            if(g_Debug_2) USART0_SendStr(g_StrBuf);
            break;
        
        case CHANNEL_UOST3:
            g_Uost3 = (unsigned int)(adcBuf * ADC_COEFF) * U_OST3_COEFF;
            sprintf(g_StrBuf, "Uost3=%d\r\n", g_Uost3);
            if(g_Debug_2) USART0_SendStr(g_StrBuf);
            break;
        
        case CHANNEL_UOST2:
            g_Uost2 = (unsigned int)(adcBuf * ADC_COEFF) * U_OST2_COEFF;
            sprintf(g_StrBuf, "Uost2=%d\r\n", g_Uost2);
            if(g_Debug_2) USART0_SendStr(g_StrBuf);
            break;

        default:
            break;

    }


    g_CurrentChannel++;
    if(g_CurrentChannel > MAX_ADC_CHANNNEL)
    {
        g_CurrentChannel = 0;
    }

    // ���������� ��������� ���������� ����� �� ������� (�� ����� 0 1 2 3 4 5 0 1 2 3 4 5 0 1 ...)
    ADMUX = ADMUX & 0b11111000;
    ADMUX = ADMUX | g_CurrentChannel;
    
    // Restarting AD conversion defore exit
    StartConvAdc();
}

/************************************************************************************/
/* �������:     ���������� ���������� USART RX Complete                             */
/* ��������:    ��������� ������� �� USART0 �� �� ��� ������� �������� ���������� � */
/*                �������� ��������� ������ �� ����������� ������� � �������� ����  */
/*                ����� ����� g_CmdToExecute                                        */
/************************************************************************************/

ISR(USART0_RX_vect)
{
    unsigned char ch = UDR0;
    unsigned short ind;

    switch(ch)
    {
        // ���������� ����� �����������
        case KEY_ESC:
            StopDiagnostic();
            break;
           
        // ���������, ���� �� ������� ������ ��� �������
        case KEY_ENTER:
            if(g_CmdSymbolIndex == 0)
            {
                USART0_SendStr("\r\n");
                break;
            }
            
            g_CmdBuffer[g_CmdSymbolIndex] = '\0';   // ������������� ��������� �������
            
            // ����������� �������� ������� � ������ ������� ��� ����������
            for(ind = 0; g_CmdBuffer[ind] != '\0' && ind < CMD_BUF_LEN; ++ind)
            {
                g_CmdToExecute[ind] = g_CmdBuffer[ind];
                
                // �������� ����������� ������� � ��������� �� 0
                if(g_CmdToExecute[ind] == KEY_SPACE)
                {
                    g_CmdToExecute[ind] = '\0';
                }
            }
            g_CmdToExecute[ind] = '\0';
            
            g_CmdSymbolIndex = 0;           // ��������� �� ������ ������ ������ �������
            USART0_SendStr("\r\n");         // ��������� ������ �� ��������
            g_ExecuteCommand = TRUE;        // ��������� ���� �� ���������� �������
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
