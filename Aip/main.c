/*
 * Aip.c
 *
 * Created: 04.10.2018 21:58:39
 * Author : KIvanychev
 */ 

/************************************************************************/
/* INCLUDES                                                             */
/************************************************************************/


#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>


/************************************************************************/
/* CONSTANTS                                                            */
/************************************************************************/

#define FOSC 16000000// Clock Speed
#define UART_BR 9600
#define MYUBRR FOSC/16/UART_BR-1

#define TRUE        1
#define FALSE       0

#define ON          1
#define OFF         0

#define LED1            PG3
#define LED2            PG4

#define mcu_RXDO_in     PE0
#define mcu_TXD0_out    PE1
#define Function_in     PE2
#define Uzt_out         PE3
#define StartPWM_out    PE5
#define Ustir_out       PE6
#define StartZU_out     PE7

#define TIMER_PERIOD    100  // us

#define UZT_HALF        0x7F
#define UZT_ZERO        0x01

#define LED1_FLASH_CNT  500

/************************************************************************/
/* LOCAL TYPES                                                          */
/************************************************************************/

typedef unsigned char BOOL;

/* Command codes after conversion from string equivalents */
typedef enum {
    CMD_START_ECHO,
    CMD_STOP_ECHO,
    CMD_START_CHARGER,
    CMD_STOP_CHARGER,
    CMD_START_PWM_CONVERTER,
    CMD_STOP_PWM_CONVERTER,
    CMD_SET_CURRENT,
    CMD_START_ADJUSTMENT,
    CMD_STOP_ADJUSTMENT,

    CMD_LEN

} TCommand;

typedef enum {
    UART_OK,
    UART_ERROR

} TUartResult;

/************************************************************************/
/* LOCAL VARIABLES                                                      */
/************************************************************************/

BOOL g_uart0_initialized = FALSE;
BOOL g_uart0_echo_started = TRUE;

char *g_cmd_string[CMD_LEN];
unsigned int g_led1_flash_cnt = LED1_FLASH_CNT;

/************************************************************************/
/* FUNCTION PROTOTYPES                                                  */
/************************************************************************/

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

void        CmdInit();
int         Command_Receive(unsigned char *cmdStr, unsigned char *param1, unsigned char *param2);

void        StartPWMConverter();
void        StopPWMConverter();

void        StartCharger();
void        StopCharger();

void        SetCurrent(int value);

void        StartAdjustment();
void        StopAdjustment();




/************************************************************************/
/* IMPLEMENTATION                                                       */
/************************************************************************/

/*
 * Turns on Echo mode for UART0
 */
void UART0_StartEcho()
{
}

/*
 * Turns off Echo mode for UART0
 */
void UART0_StopEcho()
{
}

/*
 * Initializes Timer0 for system ticking
 */
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

/*
 * Initializes Timer3 for PWM mode for Uzt
 */
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


/*
 * Initializes IO ports
 */
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
    
//    tmp = (1 << mcu_TXD0_out) | (1 << Uzt_out) | (1 << StartPWM_out) | (1 << Ustir_out) | (1 << StartZU_out);
    DDRE = tmp;
    
}


/*
 * Pass ON/OFF parameter for LED1 on or off
 */
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

/*
 * Pass ON/OFF parameter for LED2 on or off
 */
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

/*
 * Toggles LED1 
 */
void Toggle_LED1()
{
    unsigned char tmp;

    tmp = PORTG;
    tmp = tmp ^ (1 << LED1);

    PORTG = tmp;
}

/*
 * Toggles LED2 
 */
void Toggle_LED2()
{
    unsigned char tmp;

    tmp = PORTG;
    tmp = tmp ^ (1 << LED2);

    PORTG = tmp;
}

/*
 * Initializes g_cmd_string array with the command strings
 */
void CmdInit()
{
    g_cmd_string[CMD_START_ECHO] = "StartEcho";
    g_cmd_string[CMD_STOP_ECHO] = "StopEcho";
    g_cmd_string[CMD_START_CHARGER] = "StartCharger";
    g_cmd_string[CMD_STOP_CHARGER] = "StopCharger";
    g_cmd_string[CMD_START_PWM_CONVERTER] = "StartConverter";
    g_cmd_string[CMD_STOP_PWM_CONVERTER] = "StopConverter";
    g_cmd_string[CMD_SET_CURRENT] = "SetCurrent";
    g_cmd_string[CMD_START_ADJUSTMENT] = "StartAdjustment";
    g_cmd_string[CMD_STOP_ADJUSTMENT] = "StopAdjustment";

}




//---------------------------------------------------------------------------------------------
// DESCRIPTION:     Настройка работы последовательного интерфейса USART
// PARAMETERS:      baudrate - скорость передачи, бит/c

void USART0_Init(unsigned int baudrate)
{

	// параметры кадра данных: 8 Data, 1 Stop, No Parity
	// USART0 Transmitter: On
	// USART0 Mode: Asynchronous
	UCSR0A = 0x00;

    UCSR0B = (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = (1 << UCSZ01) + (1 << UCSZ00); // 8 bit data frame глава 20.11.4

	UBRR0H = baudrate >> 8;
	UBRR0L = baudrate;
}

//---------------------------------------------------------------------------------------------
// Описание:    Передает текстовый символ по последовательному интерфейсу
// Параметры:   ch -- текстовый символ для передачи по USART

inline void USART0_SendChar(unsigned char ch)
{
	// wait for data to be received
	while(!(UCSR0A & (1<<UDRE0)));
	// send data
	UDR0 = ch; 
	
}

//---------------------------------------------------------------------------------------------
// Функция:     USART0_SendStr
// Описание:    Отправляет тектовую строку по последовательному интерфейсу USART
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










/*
 * Main logic
 */
int main(void)
{
    GPIO_Init();

    Toggle_LED2();   
    USART0_Init(103);
    
    USART0_SendStr("Give me battery\r\n");
    
    USART0_SendStr("Power supply Controller ver 0.1\r\n");
    USART0_SendStr("(c) NNTU, 2018\r\n");
    USART0_SendStr("Echo mode ON\r\n");
    USART0_SendStr("Ready\r\n");


    while (1) 
    {
        /* Wait for data to be received */
        if ( (UCSR0A & (1<<RXC0)) )
        {
            unsigned char ch = UDR0;
            USART0_SendChar(ch);
        }            
        
    }
}


/************************************************************************/
/* Timer 1 Compare match Interrupt handler                              */
/*                                                                      */
/*                                                                      */
/************************************************************************/

ISR(TIMER1_COMPA_vect)
{
    g_led1_flash_cnt--;
    if(g_led1_flash_cnt == 0)
    {
        Toggle_LED2();
        g_led1_flash_cnt = LED1_FLASH_CNT;
    }

}
