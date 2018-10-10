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

#include <string.h>


/************************************************************************/
/* CONSTANTS                                                            */
/************************************************************************/

#define TRUE        1
#define FALSE       0

#define LED1            PG3
#define LED2            PG2

#define mcu_RXDO_in     PE0
#define mcu_TXD0_out    PE1
#define Function_in     PE2
#define Uzt_out         PE3
#define StartPWM_out    PE5
#define Ustir_out       PE6
#define StartZU_out     PE7

/************************************************************************/
/* LOCAL TYPES                                                          */
/************************************************************************/

typedef unsigned char BOOL;

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

/************************************************************************/
/* LOCAL VARIABLES                                                      */
/************************************************************************/

BOOL g_uart0_initialized = FALSE;
BOOL g_uart0_echo_started = TRUE;

char g_cmd_string[CMD_LEN];


/************************************************************************/
/* FUNCTION PROTOTYPES                                                  */
/************************************************************************/

void    UART0_Init();
int     UART0_SendMessage(unsigned char *msg);
void    UART0_StartEcho();
void    UART0_StopEcho();

void    GPIO_Init();

void    Timer0_Init();

void    CmdInit();
int     Command_Receive(unsigned char *cmdStr, unsigned char *param1, unsigned char *param2);

void    StartPWMConverter();
void    StopPWMConverter();

void    StartCharger();
void    StopCharger();

void    SetCurrent(int value);

void    StartAdjustment();
void    StopAdjustment();








/************************************************************************/
/* IMPLEMENTATION                                                       */
/************************************************************************/

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
    
    tmp = (1 << mcu_TXD0_out) | (1 << Uzt_out) | (1 << StartPWM_out) | (1 << Ustir_out) | (1 << StartZU_out);
    DDRE = tmp;
    
}


/*
 * Initializes g_cmd_string array
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


/*
 * Main logic
 */
int main(void)
{
    CmdInit();

    while (1) 
    {
    }
}


