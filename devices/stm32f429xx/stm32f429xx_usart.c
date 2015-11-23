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
#include "stm32f429xx_usart.h"
#include <stdarg.h>

//start count by 0,1,2,etc
static const UartConfiguration g_uartConfigurations[] = {
    {
		/* 
		 * Tx=PA9
		 * Rx=PA10
		 */
        USART1,
        7,//AF7
        7//AF7
    },
    {
		/* 
		 * Tx=PD5
		 * Rx=PD6
		 */
        USART2,
        7,//AF7
        7//AF7
    },
    {
		/* 
		 * Tx=PB10
		 * Rx=PB11
		 */
        USART3,
        7,//AF7
        7//AF7
    }
};



typedef struct {
    int      share;
    uint32_t uartIndex;
    uint32_t baudRate;
} UartParameters;


static void 	configureNVICForUartInterrupt(uint32_t index);
static void     parseUartParameters(Token* pParameterTokens, UartParameters* pParameters);
static void     setManualBaudFlag(void);
static void     setUartSharedFlag(void);
static void     saveUartToBeUsedByDebugger(uint32_t mriUart);
static void  	configureUartForExclusiveUseOfDebugger(UartParameters* pParameters);

void Debug_USART3_INIT(void);
void USART3_SendChar(int Character);
int USART3_ReceiveChar(void);
void USART3_puts(char* s);
void dbg_printf(char *fmt, ...);
void dbg_vprintf(char *fmt, va_list va);
static int dbg_put_hex(const uint32_t val, int width, const char pad);
static void dbg_put_dec(const uint32_t val, const int width, const char pad);
static void dbg_puts_x(char *str, int width, const char pad);



void __mriStm32f429xxUart_Init(Token *pParameterTokens)
{
    UartParameters parameters;
    setManualBaudFlag();//STM32f429 does not support auto BaudRate

    parseUartParameters(pParameterTokens, &parameters);
    saveUartToBeUsedByDebugger(parameters.uartIndex);
    if (parameters.share)
        setUartSharedFlag();
    else {
        configureUartForExclusiveUseOfDebugger(&parameters);
    }
}


static void parseUartParameters(Token* pParameterTokens, UartParameters* pParameters)
{
    memset(pParameters, 0, sizeof(*pParameters));
    pParameters->uartIndex = 1;
    pParameters->baudRate = 115200;

    if (Token_MatchingString(pParameterTokens, "MRI_UART_MBED_USB"))
        pParameters->uartIndex = 1;
    if (Token_MatchingString(pParameterTokens, "MRI_UART_1"))
        pParameters->uartIndex = 1;
    if (Token_MatchingString(pParameterTokens, "MRI_UART_2"))
        pParameters->uartIndex = 2;
    if (Token_MatchingString(pParameterTokens, "MRI_UART_3"))
        pParameters->uartIndex = 3;

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

static void enableUartPeripheralCLOCK(uint32_t uart)
{
    /*
     * USART1:APB2ENR;  GPIOA:AHB1
     * USART2:APB1ENR;  GPIOD:AHB1
     * USART3:APB1ENR;  GPIOB:AHB1
     */
    switch(uart) {
        case 1://USART1
            RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
            break;
        case 2://USART2
            RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
            break;
        case 3://USART3
            RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
            break;
        default://USART1
            RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    }
}

typedef struct {
    uint32_t _mriSYSCLK_Frequency; /*!<  SYSCLK clock frequency expressed in Hz */
    uint32_t _mriHCLK_Frequency;   /*!<  HCLK clock frequency expressed in Hz   */
    uint32_t _mriPCLK1_Frequency;  /*!<  PCLK1 clock frequency expressed in Hz  */
    uint32_t _mriPCLK2_Frequency;  /*!<  PCLK2 clock frequency expressed in Hz  */
} _mriRCC_ClocksTypeDef;

static __I uint8_t _mriAPBAHBPrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};

#define HSI_VALUE ((uint32_t)16000000)
#define HSE_VALUE ((uint32_t)8000000)

static void _mriRCC_GetClocksFreq(_mriRCC_ClocksTypeDef* RCC_Clocks)
{
    uint32_t tmp = 0, presc = 0, pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;

    /* Get SYSCLK source -------------------------------------------------------*/
    tmp = RCC->CFGR & RCC_CFGR_SWS;

    switch (tmp) {
        case 0x00:  /* HSI used as system clock source */
            RCC_Clocks->_mriSYSCLK_Frequency = HSI_VALUE;
            break;
        case 0x04:  /* HSE used as system clock  source */
            RCC_Clocks->_mriSYSCLK_Frequency = HSE_VALUE;
            break;
        case 0x08:  /* PLL used as system clock  source */

            /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN
               SYSCLK = PLL_VCO / PLLP
               */
            pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
            pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;

            if (pllsource != 0) {
                /* HSE used as PLL clock source */
                pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
            } else {
                /* HSI used as PLL clock source */
                pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
            }

            pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >>16) + 1 ) *2;
            RCC_Clocks->_mriSYSCLK_Frequency = pllvco/pllp;
            break;
        default:
            RCC_Clocks->_mriSYSCLK_Frequency = HSI_VALUE;
            break;
    }
    /* Compute HCLK, PCLK1 and PCLK2 clocks frequencies ------------------------*/

    /* Get HCLK prescaler */
    tmp = RCC->CFGR & RCC_CFGR_HPRE;
    tmp = tmp >> 4;
    presc = _mriAPBAHBPrescTable[tmp];
    /* HCLK clock frequency */
    RCC_Clocks->_mriHCLK_Frequency = RCC_Clocks->_mriSYSCLK_Frequency >> presc;

    /* Get PCLK1 prescaler */
    tmp = RCC->CFGR & RCC_CFGR_PPRE1;
    tmp = tmp >> 10;
    presc = _mriAPBAHBPrescTable[tmp];
    /* PCLK1 clock frequency */
    RCC_Clocks->_mriPCLK1_Frequency = RCC_Clocks->_mriHCLK_Frequency >> presc;

    /* Get PCLK2 prescaler */
    tmp = RCC->CFGR & RCC_CFGR_PPRE2;
    tmp = tmp >> 13;
    presc = _mriAPBAHBPrescTable[tmp];
    /* PCLK2 clock frequency */
    RCC_Clocks->_mriPCLK2_Frequency = RCC_Clocks->_mriHCLK_Frequency >> presc;
}

/* Calculates the value for the USART_BRR */
static uint16_t usart_baud_calc(uint32_t base,USART_TypeDef *USARTx,uint32_t baudrate)
{
    uint32_t tmpreg = 0x00, apbclock = 0x00;
    uint32_t integerdivider = 0x00;
    uint32_t fractionaldivider = 0x00;
    _mriRCC_ClocksTypeDef RCC_ClocksStatus;

    /* Configure the USART Baud Rate */
    _mriRCC_GetClocksFreq(&RCC_ClocksStatus);

    if ((base == USART1_BASE) || (base == USART6_BASE)) {
        apbclock = RCC_ClocksStatus._mriPCLK2_Frequency;
    } else {
        apbclock = RCC_ClocksStatus._mriPCLK1_Frequency;
    }

    /* Determine the integer part */
    if ((USARTx->CR1 & USART_CR1_OVER8) != 0) {
        /* Integer part computing in case Oversampling mode is 8 Samples */
        integerdivider = ((25 * apbclock) / (2 * (baudrate)));
    } else { /* if ((USARTx->CR1 & USART_CR1_OVER8) == 0) */
        /* Integer part computing in case Oversampling mode is 16 Samples */
        integerdivider = ((25 * apbclock) / (4 * (baudrate)));
    }
    tmpreg = (integerdivider / 100) << 4;

    /* Determine the fractional part */
    fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

    /* Implement the fractional part in the register */
    if ((USARTx->CR1 & USART_CR1_OVER8) != 0) {
        tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x07);
    } else { /* if ((USARTx->CR1 & USART_CR1_OVER8) == 0) */
        tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t)0x0F);
    }

    /* Write to USART BRR register */
    return (uint16_t)tmpreg;
}

void enableUART(uint32_t uart)
{
    USART_TypeDef* _uart = g_uartConfigurations[uart-1].pUartRegisters;
    /*******************************___CR2___********************************/
    /*
     * 00 = 1 stop-bit
     */
    _uart->CR2 &= ~USART_CR2_STOP;

    /*******************************___CR1___********************************/
    /*
     * Word Length : 8 Data bits
     */
    _uart->CR1 &= ~USART_CR1_M;

    /*
     * Parity bit NO
     */
    _uart->CR1 &= ~USART_CR1_PCE;

    /*
     * USART Mode
     */
    _uart->CR1 |= ( USART_CR1_RE | USART_CR1_TE );

    /*******************************___CR3___********************************/
    /*
     * Flow Control don't need
     */
    _uart->CR3 &= ~USART_CR3_RTSE;//disable RTS flow control
    _uart->CR3 &= ~USART_CR3_CTSE;//disable CTS flow control
    /*******************************___BRR___********************************/
    /*
     * Set baud-rate
     */
    uint32_t base_addr;
    USART_TypeDef *USARTx;
    switch(uart) {
        case 1://USART1
            base_addr = USART1_BASE;
            USARTx = USART1;
            break;
        case 2://USART2
            base_addr = USART2_BASE;
            USARTx = USART2;
            break;
        case 3://USART3
            base_addr = USART3_BASE;
            USARTx = USART3;
            break;
		default:
			base_addr = USART1_BASE;
			USARTx = USART1;
    }

    _uart->BRR = usart_baud_calc(base_addr,USARTx,115200);//baudrate=115200

    /*
     * Enable USART
     */
    _uart->CR1 |= USART_CR1_UE;

}


#define GPIO_PUPDR_M(n)                 (uint32_t) (0x3 << (2*n))          /* Pin mask */
#define GPIO_PUPDR_PIN(n)               (uint32_t) (2*n)                   /* Pin bitshift */
#define GPIO_PUPDR_NONE                 (uint32_t) (0x0)                   /* Port no pull-up, pull-down */
#define GPIO_MODER_M(n)                 (uint32_t) (0x3 << 2*n)            /* Pin mask */
#define GPIO_MODER_PIN(n)               (uint32_t) (2*n)                   /* Pin bitshift */
#define GPIO_MODER_ALT                  (uint32_t) (0x2)                   /* Alternative function mode */
void enableGPIO(uint32_t uart)//All GPIO(contains USARTs) on AHB1
{
    /* USART1:GPIO_A
     * USART2:GPIO_D
     * USART3:GPIO_B
     */
    GPIO_TypeDef *my_GPIO;
    uint8_t pin_tx = 9;
    uint8_t pin_rx = 10;
    switch(uart) { //To Do:Using better function
        case 1://USART1
            my_GPIO = GPIOA;
            pin_tx = 9;
            pin_rx = 10;
            break;
        case 2://USART2
            my_GPIO = GPIOD;
            pin_tx = 5;
            pin_rx = 6;
            break;
        case 3://USART3
            my_GPIO = GPIOB;
            pin_tx = 10;
            pin_rx = 11;
            break;
        default://USART1
            my_GPIO = GPIOA;
            pin_tx = 9;
            pin_rx = 10;
            break;
    }
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
    switch(uart) { //To Do: Using better function
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
    __mriStm32f429xxState.pCurrentUart->pUartRegisters->CR1 |= USART_CR1_RXNEIE;
}


static void setManualBaudFlag(void)
{
    __mriStm32f429xxState.flags |= STM32F429XX_UART_FLAGS_MANUAL_BAUD;
}

//If the order of g_uartConfigurations changes,fix it!
/*  Return 0 for USART1
 *  Return 1 for USART2
 *  Return 2 for USART3
 */
int Platform_CommUartIndex(void)
{
    return __mriStm32f429xxState.pCurrentUart - g_uartConfigurations;
}

uint32_t Platform_CommHasReceiveData(void)
{
    return __mriStm32f429xxState.pCurrentUart->pUartRegisters->SR & USART_SR_RXNE;
}

int Platform_CommReceiveChar(void)
{
    while(!Platform_CommHasReceiveData()) {
        //busy wait
    }
    return (__mriStm32f429xxState.pCurrentUart->pUartRegisters->DR & 0x1FF);
}



void Platform_CommSendChar(int Character)
{
    USART_TypeDef *uart = __mriStm32f429xxState.pCurrentUart->pUartRegisters;
    while (!(uart->SR & USART_SR_TXE)) {
        //busy wait
    }
    uart->DR = (Character & 0x1FF);
}

int Platform_CommCausedInterrupt(void)
{
    uint32_t interruptSource = getCurrentlyExecutingExceptionNumber()-16;

    //For USART1~3,the IRQn are continuous,others need check!
    uint32_t irq_num_base = (uint32_t)USART1_IRQn;
    uint32_t currentUartIRQ = irq_num_base + Platform_CommUartIndex();
    return currentUartIRQ==interruptSource;
}

void Platform_CommClearInterrupt(void)
{
    //Clear Interrupt flag,to avoid infinit loop in USARTx_Handler
    __mriStm32f429xxState.pCurrentUart->pUartRegisters->SR &= ~USART_SR_RXNE;
}

int Platform_CommSharingWithApplication(void)
{
    return __mriStm32f429xxState.flags & STM32F429XX_UART_FLAGS_SHARE;
}

static int isManualBaudRate(void)
{
    return (int)(__mriStm32f429xxState.flags & STM32F429XX_UART_FLAGS_MANUAL_BAUD);
}

int Platform_CommShouldWaitForGdbConnect(void)
{
    return !isManualBaudRate() && !Platform_CommSharingWithApplication();
}

int Platform_CommIsWaitingForGdbToConnect(void)
{
    //if (!Platform_CommShouldWaitForGdbConnect())
    	return 0;//stm32f429 does not support auto-baudrate
}



void Platform_CommPrepareToWaitForGdbConnection(void)
{
    //if (!Platform_CommShouldWaitForGdbConnect())
        return;//Due to the Platform_CommShouldWaitForGdbConnect() always return 0,the if condition always enter!
}


static int hasHostSentDataInLessThan10Milliseconds(void);
static int isNoReceivedCharAndNoTimeout(void);

void Platform_CommWaitForReceiveDataToStop(void)
{
    while (hasHostSentDataInLessThan10Milliseconds()) {
        Platform_CommReceiveChar();
    }
}



static int hasHostSentDataInLessThan10Milliseconds(void)
{
    uint32_t originalSysTickControlValue = getCurrentSysTickControlValue();
    uint32_t originalSysTickReloadValue = getCurrentSysTickReloadValue();
    start10MillisecondSysTick();

    while (isNoReceivedCharAndNoTimeout()) {
    }

    setSysTickReloadValue(originalSysTickReloadValue);
    setSysTickControlValue(originalSysTickControlValue);

    return Platform_CommHasReceiveData();
}

static int isNoReceivedCharAndNoTimeout(void)
{
    return !Platform_CommHasReceiveData() && !has10MillisecondSysTickExpired();
}

static void configureNVICForUartInterrupt(uint32_t index)
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

////////////////////////////////////////////////////////////////////////////////////////

void Debug_USART3_INIT(void)
{
    enableUartPeripheralCLOCK(3);
    enableGPIO(3);
    enableUART(3);//baudrate = 115200
}

void USART3_SendChar(int Character)
{
    USART_TypeDef *uart = USART3;
    while (!(uart->SR & USART_SR_TXE)) {
        //busy wait
    }   
    uart->DR = (Character & 0x1FF);
}

int USART3_ReceiveChar(void)                                                                                                                                     
{
    while( !(USART3->SR & USART_SR_RXNE) ) {
        //busy wait
    }  
    return (USART3->DR & 0x1FF);
}

void USART3_puts(char* s)
{
    while(*s) {
    	USART3_SendChar(*s);
        s++;
    }   
}

static void dbg_puts_x(char *str, int width, const char pad) 
{
    while (*str) {
        if (*str == '\n')
            USART3_SendChar('\r');
        USART3_SendChar(*(str++));
        --width;
    }

    while (width > 0) {
        USART3_SendChar(pad);
        --width;
    }
}


static void dbg_put_dec(const uint32_t val, const int width, const char pad) 
{
    uint32_t divisor;
    int digits;

    // estimate number of spaces and digits
    for (divisor = 1, digits = 1; val / divisor >= 10; divisor *= 10, digits++)
    	;

    // print spaces
    for (; digits < width; digits++)
        USART3_SendChar(pad);

    // print digits 
    do {
        USART3_SendChar(((val / divisor) % 10) + '0');
    } while (divisor /= 10);
}


#define hexchars(x) \
    (((x) < 10) ? \
        ('0' + (x)) : \
        ('a' + ((x) - 10)))

static int dbg_put_hex(const uint32_t val, int width, const char pad)
{
    int i, n = 0;
    int nwidth = 0;

    // Find width of hexnumber 
    while ((val >> (4 * nwidth)) && ((unsigned) nwidth <  2 * sizeof(val)))
        nwidth++;
    if (nwidth == 0)
        nwidth = 1;

    // May need to increase number of printed characters
    if (width == 0 && width < nwidth)
        width = nwidth;

    // Print number with padding 
    for (i = width - nwidth; i > 0; i--, n++)
        USART3_SendChar(pad);
    for (i = 4 * (nwidth - 1); i >= 0; i -= 4, n++)
        USART3_SendChar(hexchars((val >> i) & 0xF));

    return n;
}


void dbg_printf(char *fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    dbg_vprintf(fmt, va);
    va_end(va);
}

void dbg_vprintf(char *fmt, va_list va)
{
    int mode = 0;   // 0: usual char; 1: specifiers
    int width = 0;
    char pad = ' ';
    int size = 16;

    while (*fmt) {
        if (*fmt == '%') {
            mode = 1;
            pad = ' ';
            width = 0;
            size = 32;

            fmt++;
            continue;
        }

        if (!mode) {
            if (*fmt == '\n')
                USART3_SendChar('\r');
            USART3_SendChar(*fmt);
        } else {
            switch (*fmt) {
            case 'c':
                USART3_SendChar(va_arg(va, uint32_t));
                mode = 0;
                break;
            case 's':
                dbg_puts_x(va_arg(va, char *), width, pad);
                mode = 0;
                break;
            case 'l':
            case 'L':
                size = 64;
                break;
            case 'd':
            case 'D':
                dbg_put_dec((size == 32) ?
                             va_arg(va, uint32_t) :
                             va_arg(va, uint64_t),
                             width, pad);
                mode = 0;
                break;
            case 'p':
            case 't':
                size = 32;
                width = 8;
                pad = '0';
            case 'x':
            case 'X':
                dbg_put_hex((size == 32) ?
                             va_arg(va, uint32_t) :
                             va_arg(va, uint64_t),
                             width, pad);
                mode = 0;
                break;
            case '%':
                USART3_SendChar('%');
                mode = 0;
                break;
            case '0':
                if (!width)
                    pad = '0';
                break;
            case ' ':
                pad = ' ';
            }
            if (*fmt >= '0' && *fmt <= '9') {
                width = width * 10 + (*fmt - '0');
            }
        }

        fmt++;
    }
}    


////////////////////////////////////////////////////////////////////////////////
static void configureUartForExclusiveUseOfDebugger(UartParameters* pParameters)
{
    uint32_t uart_index = pParameters->uartIndex;
    enableUartPeripheralCLOCK(uart_index);
    enableGPIO(uart_index);
    enableUART(uart_index);//baudrate = 115200
    enableUartToInterruptOnReceivedChar(uart_index);
    Platform_CommPrepareToWaitForGdbConnection();
    configureNVICForUartInterrupt(uart_index);


	//Debugging
	Debug_USART3_INIT();
	dbg_printf("USART3_Debug Connecting Successfully!\n");
}

