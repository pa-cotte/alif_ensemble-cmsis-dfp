/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     Demo_pm_baremetal.c
 * @author   Raj Ranjan
 * @email    raj.ranjan@alifsemi.com
 * @version  V1.0.0
 * @date     23-Feb-2023
 * @brief    .
 ******************************************************************************/

/* System Includes */
#include <stdio.h>
#include <string.h>

/* UART Driver instance (UART0-UART7) */
#if defined(M55_HE)
#define UART                                4
#elif defined(M55_HP)
#define UART                                2
#endif

/*******************************   UART      **********************************/

#include <RTE_Components.h>
#include CMSIS_device_header

/* Project Includes */
#include "Driver_USART_Private.h"   /* ARM UART driver related */
#include "pinconf.h"                /* UART PINs               */

/* UART Related Macros */
#define _UART_BASE_(n)          UART##n##_BASE
#define UART_BASE(n)            _UART_BASE_(n)

/* UART Driver */
extern ARM_DRIVER_USART ARM_Driver_USART_(UART)  ;
static ARM_DRIVER_USART *USARTdrv   = &ARM_Driver_USART_(UART);
UART_Type *uart_reg_ptr             = (UART_Type *)UART_BASE(UART);


#if defined(PRINTF_REDIRECT)
volatile uint32_t   echoEnable;
volatile uint32_t   uartReady ;
FILE                __stdout  ;
FILE                __stdin   ;

/**
 * @function    void uartWrite(char c)
 * @brief       UART write (i.e. fputc) function implementation
 * @param[in]   c      : Input Character
 * @retval      none
 */
static void uartWrite(char c)
{
    /* wait until uart is to ready to send */
    while ( (uart_reg_ptr->UART_USR & UART_USR_TRANSMIT_FIFO_NOT_FULL) == 0 );

    /* write a char to thr transmit holding register */
    uart_reg_ptr->UART_THR = c;
}

/**
 * @function    char uartRead(int c)
 * @brief       UART Read (i.e. fgetc) function implementation
 * @param[in]   c      : Input Character
 * @retval      none
 */
static char uartRead(void)
{
    /* wait until uart is ready to receive */
    while ( (uart_reg_ptr->UART_USR & UART_USR_RECEIVE_FIFO_NOT_EMPTY) == 0 );

    /* read a char from receive buffer register */
    return (char)uart_reg_ptr->UART_RBR;
}

/**
 * @function    int fputc(int c, FILE * stream)
 * @brief       Standard fput function implementation
 * @param[in]   c      : Input Character
 * @param[in]   stream : Target
 * @retval      none
 */
int fputc(int c, FILE * stream)
{
    if(uartReady) {
        uartWrite(c);
        return c; //return the character written to denote a successful write
    }

    *stream = __stdout;
    return 0;
}

/**
 * @function    int fgetc(int c, FILE * stream)
 * @brief       Standard fget function implementation
 * @param[in]   c      : Input Character
 * @param[in]   stream : Target
 * @retval      none
 */
int fgetc(FILE * stream)
{
    if(uartReady) {
        char c = uartRead();
        if(echoEnable)
            uartWrite(c); //To echo Received characters to serial Terminal
        return c;
    }

    *stream = __stdin;
    return 0;
}
#endif  //PRINTF_REDIRECT


/**
 * @function    int pinmux_init()
 * @brief       Init from UART4 instance
 * @note        none
 * @retval      none
 */
static int pinmux_init()
{
    int ret = -1;

#if defined(M55_HE)
    /* UART4_RX_B */
    ret = pinconf_set( PORT_12,  PIN_1,  PINMUX_ALTERNATE_FUNCTION_2,  \
            PADCTRL_READ_ENABLE);
    if(ret != ARM_DRIVER_OK)
        return ret;

    /* UART4_TX_B */
    ret = pinconf_set( PORT_12,  PIN_2,  PINMUX_ALTERNATE_FUNCTION_2,  \
            PADCTRL_READ_ENABLE);
    if(ret != ARM_DRIVER_OK)
        return ret;

#elif defined(M55_HP)
    /* UART2_RX_B */
    ret = pinconf_set( PORT_1,  PIN_0,  PINMUX_ALTERNATE_FUNCTION_1,  \
            PADCTRL_READ_ENABLE);
    if(ret != ARM_DRIVER_OK)
        return ret;

    /* UART2_TX_B */
    ret = pinconf_set( PORT_1,  PIN_1,  PINMUX_ALTERNATE_FUNCTION_1,  \
            PADCTRL_READ_ENABLE);
    if(ret != ARM_DRIVER_OK)
        return ret;
#endif
    return ret;
}


/**
  @fn           void uart_error_uninitialize()
  @brief        UART un-initializtion:
  @return       none
*/
static int uart_error_uninitialize()
{
    int ret = -1;

    /* Un-initialize UART driver */
    ret = USARTdrv->Uninitialize();
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: UART Uninitialize failed.\r\n");
    }
    return ret;
}

/**
  @fn           int uart_error_power_off()
  @brief        UART power-off:
  @return       none
*/
static int uart_error_power_off()
{
    int ret = -1;

    /* Power off UART peripheral */
    ret = USARTdrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: UART Power OFF failed.\r\n");
        return ret;
    }

    ret = uart_error_uninitialize();

    return ret;
}

/**
 * @function    int uart_init()
 * @brief       Init from UART4 instance
 * @note        none
 * @retval      none
 */
static int uart_init()
{
    int32_t ret = -1;

    /*Initialize the USART driver */
    ret = pinmux_init();
    if(ret != ARM_DRIVER_OK)
        return ret;

    /*Initialize the USART driver */
    ret = USARTdrv->Initialize(NULL); //polling without isr callback
    if(ret != ARM_DRIVER_OK)
    {
        printf("UART %d init is failed (%d)\n", UART, ret);
        return ret;
    }

    /* Enable the power for UART */
    ret = USARTdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK)
    {
        printf("UART %d power control is failed (%d) \n", UART, ret);
        ret = uart_error_uninitialize();
        return ret;
    }

    ret =  USARTdrv->Control(ARM_USART_MODE_ASYNCHRONOUS        |
                ARM_USART_DATA_BITS_8 | ARM_USART_PARITY_NONE   |
                ARM_USART_STOP_BITS_1 | ARM_USART_FLOW_CONTROL_NONE, 115200);
    if(ret != ARM_DRIVER_OK)
    {
        printf("UART %d control is failed (%d) \n", UART, ret);
        ret = uart_error_power_off();
        return ret;
    }

    ret =  USARTdrv->Control(ARM_USART_CONTROL_TX, 1);  /* TX must be enable. */
    if(ret != ARM_DRIVER_OK)
    {
        printf("UART %d control is failed (%d) \n", UART, ret);
        ret = uart_error_power_off();
        return ret;
    }
    ret =  USARTdrv->Control(ARM_USART_CONTROL_RX, 1);  /* RX must be enable. */
    if(ret != ARM_DRIVER_OK)
    {
        printf("UART %d control is failed (%d) \n", UART, ret);
        ret = uart_error_power_off();
        return ret;
    }

#if defined(PRINTF_REDIRECT)
    uartReady = 1;
#endif

    printf("UART %d Init is completed ...\n", UART);

    return ret;
}
/*******************************   UART      **********************************/


/*******************************   RTC       **********************************/

/* Project Includes */
#include "Driver_RTC.h"

/* RTC Driver instance 0 */
extern ARM_DRIVER_RTC Driver_RTC0;
static ARM_DRIVER_RTC *RTCdrv = &Driver_RTC0;

/**
  \fn           void alarm_callback(uint32_t event)
  \brief        rtc alarm callback
  \return       none
*/
static void rtc_callback(uint32_t event)
{
    if (event & ARM_RTC_EVENT_ALARM_TRIGGER)
    {
        /* User code for call back */
    }
    return;
}

/**
  @fn           void rtc_error_uninitialize()
  @brief        RTC un-initializtion:
  @return       none
*/
static int rtc_error_uninitialize()
{
    int ret = -1;

    /* Un-initialize RTC driver */
    ret = RTCdrv->Uninitialize();
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: RTC Uninitialize failed.\r\n");
    }
    return ret;
}

/**
  @fn           int rtc_error_power_off()
  @brief        RTC power-off:
  @return       none
*/
static int rtc_error_power_off()
{
    int ret = -1;

    /* Power off RTC peripheral */
    ret = RTCdrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: RTC Power OFF failed.\r\n");
        return ret;
    }

    ret = rtc_error_uninitialize();

    return ret;
}

/**
  @fn           int set_rtc()
  @brief        set RTC timeout value (in second i.e. timeout = 10 means 10 sec)
  @return       none
*/
static int set_rtc(uint32_t  timeout)
{
    uint32_t  val      = 0;
    int ret;

    ret = RTCdrv->ReadCounter(&val);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: RTC read failed\n");
        rtc_error_power_off();
        return ret;
    }

    ret = RTCdrv->Control(ARM_RTC_SET_ALARM, val + timeout);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: RTC Could not set alarm\n");
        rtc_error_power_off();
        return ret;
    }
    printf("\r\n Setting alarm after %d (curr %d, future %d) counts \r\n",
            timeout, val, ( val + timeout));
    return ret;
}

/**
  @fn           int rtc_init()
  @brief        RTC Initialization only:
  @return       none
*/
static int rtc_init()
{
    int32_t   ret = -1;
    ARM_DRIVER_VERSION version;
    ARM_RTC_CAPABILITIES capabilities;

    version = RTCdrv->GetVersion();
    printf("\r\nRTC version = %d.%d \r\n", version.api, version.drv);

    capabilities = RTCdrv->GetCapabilities();
    if(!capabilities.alarm){
        printf("\r\n Error: RTC alarm capability is not available.\n");
        return ret;
    }

    /* Initialize RTC driver */
    ret = RTCdrv->Initialize(rtc_callback);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: RTC init failed\r\n");
        return ret;
    }

    /* Enable the power for RTC */
    ret = RTCdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: RTC Power up failed\n");
        rtc_error_uninitialize();
        return ret;
    }

    printf("RTC Init is done...\r\n");
    return ret;
}

/*******************************   RTC       **********************************/


/*******************************   MHU   **************************************/

/*******************************   MHU   **************************************/


/*******************************   PM   ***************************************/

/* Project Includes */
#include "pm.h"


#define POWER_MSG_CNT       6

/* Data Required for PM */
char   power_msg[POWER_MSG_CNT][64] =
{
    "\r\n\t1. Normal Sleep\r\n"                              ,
    "\r\n\t2. Deep   Sleep\r\n"                              ,
    "\r\n\t3. IWIC   Sleep\r\n"                              ,
    "\r\n\t4. Subsys Off  \r\n"                              ,
    "\r\n\t5. Change Sleep Duration or Infinite loop  \r\n"  ,
};

/**
  @fn           void pm_usage_menu()
  @brief        Power Management Menu
  @return       none
*/
static void pm_usage_menu()
{
    int i;

    printf("\r\nSelect Below Sleep Modes... \r\n");

    for(i = 0; i < POWER_MSG_CNT; i++)
    {
        printf("%s",&power_msg[i][0]);
    }
    printf("\r\n");

    return;
}


/**
  @fn           int main()
  @brief        Testapp for supported power modes.
  @return       none
*/


/* Define main entry point.  */
int main()
{
    PM_SLEEP_TYPE selectedSleepType = PM_SLEEP_TYPE_NORMAL_SLEEP;
    int32_t  ret = -1;
    uint32_t sleepDuration = 10;  /*  Making Default sleep duration as 10s */
    int8_t    ch;

    /* UART Initialization */
    ret = uart_init();
    if(ret != ARM_DRIVER_OK)
    {
        printf(" UART %d Initialization failed (%d)\n", UART, ret);
        return ret;
    }

    /* RTC Initialization */
    ret = rtc_init();
    if(ret != ARM_DRIVER_OK)
    {
        printf(" RTC Initialization failed (%d)\n", ret);
        return ret;
    }

#if defined(PRINTF_REDIRECT)
    echoEnable = 0;
#endif

    while(1)
    {
        pm_usage_menu();
        printf("\r\nChoose Sleep start:  ");
        scanf("%d", &selectedSleepType);
        printf("\r\n\r\nSelected Power Mode :  %d\r\n", selectedSleepType);

        switch(selectedSleepType) {

        case PM_SLEEP_TYPE_NORMAL_SLEEP:

            ret = set_rtc(sleepDuration);
            if( ret != ARM_DRIVER_OK)
                return ret;
            printf("\r\nCore : Going into normal sleep...\r\n");

            //Disable all interrupt
            __disable_irq();

            // Go for Normal Sleep
            pm_core_set_normal_sleep(); // setting wake Up source

            // Enable IRQ
            __enable_irq();

            printf("\r\nCore : Came  out  normal sleep...\r\n");
            break;

        case PM_SLEEP_TYPE_DEEP_SLEEP:

            ret = set_rtc(sleepDuration);
            if( ret != ARM_DRIVER_OK)
                return ret;
            printf("\r\nCore : Going into deep sleep...\r\n");

            //Disable all interrupt
            __disable_irq();

            // Go for Deep Sleep
            pm_core_set_deep_sleep();

            // Enable IRQ
            __enable_irq();
            printf("\r\nCore : Came  out  deep sleep...\r\n");
            break;

        case PM_SLEEP_TYPE_IWIC_SLEEP:

            ret = set_rtc(sleepDuration);
            if( ret != ARM_DRIVER_OK)
                return ret;
            printf("\r\nCore : Going into IWIC sleep...\r\n");

            //Disable all interrupt
            __disable_irq();

            // Go for IWIC Sleep
            pm_core_set_iwic_sleep();

            // Enable IRQ
            __enable_irq();
            printf("\r\nCore : Came  out  IWIC sleep...\r\n");
            break;

        case PM_SLEEP_TYPE_SUBSYS_OFF:

            ret = set_rtc(sleepDuration);
            if( ret != ARM_DRIVER_OK)
                return ret;
            printf("\r\nCore : Going into subsys off...\r\n");

            //Disable all interrupt
            __disable_irq();

            // Go for Sleep
            pm_core_set_subsys_off();

            printf("\r\nCore : Came  out  subsys off...\r\n");
            break;

        default :
            printf("\r\n Default statement (Can be used for miscellaneous " \
                    "purpose) \n");
            printf(" Want to change Sleep duration, press 'y' else it will "\
                    "go into infinite loop : ");
            scanf("%c",  &ch);
            if( (ch == 'y') || (ch == 'Y'))
            {
                printf("\r\n Enter Sleep duration (in sec) : ");
                scanf("%d", &sleepDuration);
                printf("\n");
            }
            else
            {
                printf("\r\nCore : Going into while 1 loop...\r\n");
                while(1);
            }
            break;
        }
    }

    return 0;
}

/********************** (c) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
