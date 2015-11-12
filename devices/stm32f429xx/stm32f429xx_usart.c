/* Copyright 2015 Adam Green (http://mbed.org/users/AdamGreen/)

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as published
   by the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.   
*/
/* Routines used to provide STM32F429ZI USART functionality to the mri debugger. */
#include <string.h>
#include <stdlib.h>
#include "platforms.h"
#include "../../architectures/armv7-m/debug_cm3.h"
#include "stm32f429xx_init.h"
//#include "./inc/serial_api.h"
//#include "./src/serial.c"
#include "stm32f429xx_usart.h"

//start count by 0,1,2,etc
static const UartConfiguration g_uartConfigurations[] = 
{
    {
	USART1,
	//SYSCFG_EXTICR3_EXTI9_PA,
	7,//AF7
	//SYSCFG_EXTICR3_EXTI10_PA,	
	7//AF7
    },
    {
	USART2,
	//SYSCFG_EXTICR2_EXTI5_PD,
	7,//AF7
	//SYSCFG_EXTICR2_EXTI6_PD,
	7//AF7
    },
    {
	USART3,
	//SYSCFG_EXTICR3_EXTI10_PB,
	7,//AF7
	//SYSCFG_EXTICR3_EXTI11_PB,
	7//AF7
    }
};



typedef struct
{
    int      share;
    uint32_t uartIndex;
    uint32_t baudRate;
} UartParameters;

/*
static int serial_mriGet_irq_index(serial_t *obj)
{
    return obj->index;
}
*/


static void     parseUartParameters(Token* pParameterTokens, UartParameters* pParameters);
static void     setManualBaudFlag(void);
static void     setUartSharedFlag(void);
static void     saveUartToBeUsedByDebugger(uint32_t mriUart);
void configureUartForExclusiveUseOfDebugger(UartParameters* pParameters);

void __mriStm32f429xxUart_Init(Token *pParameterTokens)
{
    UartParameters parameters;
    setManualBaudFlag();//STM32f429 does not support auto BaudRate

    parseUartParameters(pParameterTokens, &parameters);
    saveUartToBeUsedByDebugger(parameters.uartIndex);
    if (parameters.share)
        setUartSharedFlag();
    else
        configureUartForExclusiveUseOfDebugger(&parameters);
}


static void parseUartParameters(Token* pParameterTokens, UartParameters* pParameters)
{
    memset(pParameters, 0, sizeof(*pParameters));

    if (Token_MatchingString(pParameterTokens, "MRI_UART_MBED_USB"))
        pParameters->uartIndex = 1;
    if (Token_MatchingString(pParameterTokens, "MRI_UART_1"))
        pParameters->uartIndex = 1;
    if (Token_MatchingString(pParameterTokens, "MRI_UART_2"))
        pParameters->uartIndex = 2;
    if (Token_MatchingString(pParameterTokens, "MRI_UART_3"))
        pParameters->uartIndex = 3;
        
        pParameters->baudRate = 115200;
    
    if (Token_MatchingString(pParameterTokens, "MRI_UART_SHARE"))
        pParameters->share = 1;
}



static void saveUartToBeUsedByDebugger(uint32_t mriUart)
{
    __mriStm32f429xxState.pCurrentUart = &g_uartConfigurations[mriUart-1];//-1 is due to the array start by index 0.However,we start from UART"1".
}



static void setUartSharedFlag(void)
{
    __mriStm32f429xxState.flags |= STM32F429XX_UART_FLAGS_SHARE;
}

static void setUartPeripheralClock(uint32_t uart)
{
/*
 * USART1:APB2ENR
 * USART2:APB1ENR
 * USART3:APB1ENR
 */
    switch(uart)
    {
	case 1://USART1
	    RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
	    //RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	    break;
	case 2://USART2
	    RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST;
	    //RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	    break;
	case 3://USART3
	    RCC->APB1RSTR |= RCC_APB1RSTR_USART3RST;
	    //RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	    break;
	default://USART1
	    RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
	    //RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    }
}

/* Calculates the value for the USART_BRR */
/* TODO: Need more precise algorithm */
static int16_t usart_baud_calc(uint32_t base, uint32_t baud)
{
    uint16_t mantissa;
    uint16_t fraction;

    /* USART1 and USART6 are on APB2 whose frequency is 84MHz,
     * while USART2, USART3, UART4, and UART5 are on APB1 whose
     * frequency is 42 MHz (max).
     */
    if (base == USART1_BASE) {
        mantissa = (84000000) / (16 *  baud);
        fraction = (84000000 / baud) % 16;
    } else {
        mantissa = (42000000) / (16 *  baud);
        fraction = (42000000 / baud) % 16;
    }

    return (mantissa << 4) | fraction;
}


static void enableUART(uint32_t uart)
{
    USART_TypeDef* _uart = g_uartConfigurations[uart-1].pUartRegisters;

    /*
     * Enable GPIO
     */
    _uart->CR1 |= USART_CR1_UE;

    /*
     * 1 Start bit, 8 Data bits, n Stop bit
     */
    _uart->CR1 &= ~USART_CR1_M;

    /*
     * 00 = 1 stop-bit
     */
    _uart->CR2 &= ~USART_CR2_STOP;

    /*
     * Set baud-rate
     */
    uint32_t base_addr = USART1_BASE;
    switch(uart)
    {
	case 1://USART1
	    base_addr = USART1_BASE;
	    break;
	case 2://USART2
	    base_addr = USART2_BASE;
	    break;
	case 3://USART3
	    base_addr = USART3_BASE;
	    break;
    }

    _uart->BRR = usart_baud_calc(base_addr,115200);

    /*
     * Enanle transmit and receive
     */
    _uart->CR1 |= ( USART_CR1_RE | USART_CR1_TE );
}


#define GPIO_PUPDR_M(n)                 (uint32_t) (0x3 << (2*n))          /* Pin mask */
#define GPIO_PUPDR_PIN(n)               (uint32_t) (2*n)                   /* Pin bitshift */
#define GPIO_PUPDR_NONE                 (uint32_t) (0x0)                   /* Port no pull-up, pull-down */
#define GPIO_MODER_M(n)                 (uint32_t) (0x3 << 2*n)            /* Pin mask */
#define GPIO_MODER_PIN(n)               (uint32_t) (2*n)                   /* Pin bitshift */
#define GPIO_MODER_ALT                  (uint32_t) (0x2)                   /* Alternative function mode */
static void enableGPIO(uint32_t uart)//All GPIO(contains USARTs) on AHB1
{
    /* USART1:GPIO_A
     * USART2:GPIO_D
     * USART3:GPIO_B
     */
    GPIO_TypeDef *my_GPIO;
    uint8_t pin_tx = 9;
    uint8_t pin_rx = 10;
    uint32_t GPIO_enable = RCC_AHB1ENR_GPIOAEN;
    switch(uart)//To Do:Using better function
    {
	case 1://USART1
	    GPIO_enable = RCC_AHB1ENR_GPIOAEN;
	    my_GPIO = GPIOA;
	    pin_tx = 9;
	    pin_rx = 10;
	    break;
	case 2://USART2
	    GPIO_enable = RCC_AHB1ENR_GPIODEN;
	    my_GPIO = GPIOD;
	    pin_tx = 5;
	    pin_rx = 6;
	    break;
	case 3://USART3
	    GPIO_enable = RCC_AHB1ENR_GPIOBEN;
	    my_GPIO = GPIOB;
	    pin_tx = 10;
	    pin_rx = 11;
	    break;
	default://USART1
	    GPIO_enable = RCC_AHB1ENR_GPIOAEN;
	    my_GPIO = GPIOA;
	    pin_tx = 9;
	    pin_rx = 10;
	    break;
    }


    /*
     * Enable GPIO
     */
    RCC->AHB1ENR |= GPIO_enable;

    /*
     * Set to be non Push-pull
     */
    uint32_t mode = GPIO_PUPDR_NONE;
    my_GPIO->PUPDR &= ~(GPIO_PUPDR_M(pin_tx));
    my_GPIO->PUPDR |= (mode << GPIO_PUPDR_PIN(pin_tx));
    my_GPIO->PUPDR &= ~(GPIO_PUPDR_M(pin_rx));
    my_GPIO->PUPDR |= (mode << GPIO_PUPDR_PIN(pin_rx));

    /* 
     * Mode type
     * Set to be alternative function
     */
    uint32_t type = GPIO_MODER_ALT;
    my_GPIO->MODER &= ~(GPIO_MODER_M(pin_tx));
    my_GPIO->MODER |= (type << GPIO_MODER_PIN(pin_tx)); 
    my_GPIO->MODER &= ~(GPIO_MODER_M(pin_rx));
    my_GPIO->MODER |= (type << GPIO_MODER_PIN(pin_rx)); 

    /*
     * For Alternative-Function,assign AF
     * USART1/2/3 are all AF7
     */
    switch(uart)//To Do: Using better function
    {
	case 1://USART1
	    my_GPIO->AFR[1] &= ~0xF0;//Pin 9,tx
	    my_GPIO->AFR[1] |= (g_uartConfigurations[uart-1].txFunction << 4);
	    my_GPIO->AFR[1] &= ~0xF00;//Pin 10,rx
	    my_GPIO->AFR[1] |= (g_uartConfigurations[uart-1].rxFunction << 8);
	    break;
	case 2://USART2
	    my_GPIO->AFR[0] &= ~0xF00000;//Pin 5,tx
	    my_GPIO->AFR[0] |= (g_uartConfigurations[uart-1].txFunction << 20);
	    my_GPIO->AFR[0] &= ~0xF000000;//Pin 6,rx
	    my_GPIO->AFR[0] |= (g_uartConfigurations[uart-1].rxFunction << 24);
	    break;
	case 3://USART3
	    my_GPIO->AFR[1] &= ~0xF00;//Pin 10,tx
	    my_GPIO->AFR[1] |= (g_uartConfigurations[uart-1].txFunction << 4);
	    my_GPIO->AFR[1] &= ~0xF000;//Pin 11,rx
	    my_GPIO->AFR[1] |= (g_uartConfigurations[uart-1].rxFunction << 8);
	    break;
	default://USART1
	    my_GPIO->AFR[1] &= ~0xF0;//Pin 9,tx
	    my_GPIO->AFR[1] |= (g_uartConfigurations[uart-1].txFunction << 4);
	    my_GPIO->AFR[1] &= ~0xF00;//Pin 10,rx
	    my_GPIO->AFR[1] |= (g_uartConfigurations[uart-1].rxFunction << 8);
	    break;
    }

    /*
     * GPIO output type
     */
#define GPIO_OTYPER_M(n)                (uint32_t) (1 << n)                  /* Pin mask */
#define GPIO_OTYPER_PIN(n)              (uint32_t) (n)                       /* Pin bitshift */
#define GPIO_OTYPER_OUTPUT_PUSHPULL     0                                    /*Push Pull*/
    my_GPIO->OTYPER &= ~GPIO_OTYPER_M(pin_tx);
    my_GPIO->OTYPER |= (GPIO_OTYPER_OUTPUT_PUSHPULL << GPIO_OTYPER_PIN(pin_tx) );
    my_GPIO->OTYPER &= ~GPIO_OTYPER_M(pin_rx);
    my_GPIO->OTYPER |= (GPIO_OTYPER_OUTPUT_PUSHPULL << GPIO_OTYPER_PIN(pin_rx) );

    /*
     * GPIO speed
     */
#define GPIO_OSPEEDR_M(n)               (uint32_t) (0x3 << (2*n))           /* Pin mask */
#define GPIO_OSPEEDR_PIN(n)             (uint32_t) (2*n)                    /* Pin bitshift */
#define GPIO_OSPEEDR_50M                (uint32_t) (0x2)                    /* Output speed 50MHz */
    uint32_t speed = GPIO_OSPEEDR_50M;
    my_GPIO->OSPEEDR &= ~(GPIO_OSPEEDR_M(pin_tx));
    my_GPIO->OSPEEDR |= (speed << GPIO_OSPEEDR_PIN(pin_tx));
    my_GPIO->OSPEEDR &= ~(GPIO_OSPEEDR_M(pin_rx));
    my_GPIO->OSPEEDR |= (speed << GPIO_OSPEEDR_PIN(pin_rx));
}

static void enableUartToInterruptOnReceivedChar(uint32_t index)
{
    USART_TypeDef *uart = g_uartConfigurations[index-1].pUartRegisters; //Fix it when the struct change order!
    uart->CR1 |= USART_CR1_RXNEIE;
}


static void setManualBaudFlag(void)
{
    __mriStm32f429xxState.flags |= STM32F429XX_UART_FLAGS_MANUAL_BAUD;
    
}


uint32_t Platform_CommHasReceiveData(void)
{
    int current_uart_index = Platform_CommUartIndex();
    USART_TypeDef *uart = g_uartConfigurations[current_uart_index].pUartRegisters;
    int status = uart->SR & USART_SR_RXNE;
    return status;
}


int Platform_CommReceiveChar(void)
{
    int current_uart_index = Platform_CommUartIndex();
    USART_TypeDef *uart = g_uartConfigurations[current_uart_index].pUartRegisters;

    if(uart->SR & USART_SR_RXNE) {
        return (uart->DR & 0xFF);
    }
    return 0;
}



void Platform_CommSendChar(int Character)
{
    int current_uart_index = Platform_CommUartIndex();
    USART_TypeDef *uart = g_uartConfigurations[current_uart_index].pUartRegisters;

    if (Character) {
     while (!(uart->SR & USART_SR_TXE))
	 ;//busy wait

    uart->DR = (Character & 0x1FF);
    }
}


int Platform_CommCausedInterrupt(void)
{
    int interruptSource = (int)getCurrentlyExecutingExceptionNumber();
    int irq_num_base = USART1_IRQn;
    /*
     * For USART1~3,the IRQn are continuous,others need check!
     */
    int currentUartIRQ = irq_num_base + Platform_CommUartIndex();
    return currentUartIRQ==interruptSource;
}


void Platform_CommClearInterrupt(void)
{
    int irq_num_base = USART1_IRQn;
    /*
     * For USART1~3,the IRQn are continuous,others need check!
     */
    int currentUartIRQ = irq_num_base + Platform_CommUartIndex();
    NVIC_ClearPendingIRQ(currentUartIRQ);
}


int Platform_CommSharingWithApplication(void)
{
    return __mriStm32f429xxState.flags & STM32F429XX_UART_FLAGS_SHARE;
}


static int isManualBaudRate(void)
{
    return (int)(__mriStm32f429xxState.flags & STM32F429XX_UART_FLAGS_MANUAL_BAUD);
    //return 1;//stm32f429 does not support auto-baudrate
}


int Platform_CommShouldWaitForGdbConnect(void)
{
    return !isManualBaudRate() && !Platform_CommSharingWithApplication();
}



int Platform_CommIsWaitingForGdbToConnect(void)
{
    if (!Platform_CommShouldWaitForGdbConnect())
        return 0;//stm32f429 does not support auto-baudrate
    else//Add by STM32F429
	    return 1;
}


void Platform_CommPrepareToWaitForGdbConnection(void)
{
  /*
    static const uint32_t   autoBaudStart = 1;
    static const uint32_t   autoBaudModeForStartBitOnly = 1 << 1;
    static const uint32_t   autoBaudAutoRestart = 1 << 2;
    static const uint32_t   autoBaudValue = autoBaudStart | autoBaudModeForStartBitOnly | autoBaudAutoRestart;
  */ 
    //if (!Platform_CommShouldWaitForGdbConnect())
        return;
    
    //__mriStm32f429ziState.pCurrentUart->pUartRegisters->ACR = autoBaudValue;
}


static int hasHostSentDataInLessThan10Milliseconds(void);
static int isNoReceivedCharAndNoTimeout(void);

void Platform_CommWaitForReceiveDataToStop(void)
{
    while (hasHostSentDataInLessThan10Milliseconds())
    {
        Platform_CommReceiveChar();
    }
}



static int hasHostSentDataInLessThan10Milliseconds(void)
{
    uint32_t originalSysTickControlValue = getCurrentSysTickControlValue();
    uint32_t originalSysTickReloadValue = getCurrentSysTickReloadValue();
    start10MillisecondSysTick();

    while (isNoReceivedCharAndNoTimeout())
    {
    }

    setSysTickReloadValue(originalSysTickReloadValue);
    setSysTickControlValue(originalSysTickControlValue);
    
    return Platform_CommHasReceiveData();
}

static int isNoReceivedCharAndNoTimeout(void)
{
    return !Platform_CommHasReceiveData() && !has10MillisecondSysTickExpired();
}

//If the order of g_uartConfigurations changes,fix it!
int Platform_CommUartIndex(void)
{
    return __mriStm32f429xxState.pCurrentUart - g_uartConfigurations;
}

void configureNVICForUartInterrupt(uint32_t index)
{
    IRQn_Type irq_num_base = USART1_IRQn;
    /*
     * For USART1~3,the IRQn are continuous,others need check!
     */
    IRQn_Type currentUartIRQ;
    currentUartIRQ = (IRQn_Type)((int)irq_num_base + Platform_CommUartIndex()) ;
    NVIC_SetPriority(currentUartIRQ, 0); 
    NVIC_EnableIRQ(currentUartIRQ);
}

void configureUartForExclusiveUseOfDebugger(UartParameters* pParameters)
{ 
    uint32_t uart_index = pParameters->uartIndex;
    setUartPeripheralClock(uart_index);
    enableGPIO(uart_index);
    enableUART(uart_index);//baudrate = 115200
    enableUartToInterruptOnReceivedChar(uart_index);
    Platform_CommPrepareToWaitForGdbConnection();
    configureNVICForUartInterrupt(uart_index);
}
