git /*
 * STM32F401CCU6 - LM35 Temperature Sensor via ADC1
 * Sends raw ADC values (e.g., "310\n") to UART2 → ESP32
 */

#include <stdint.h>
#include "stm32401xx.h"



/* ---------------- UART Control Bits ---------------- */
#define USART_SR_TXE      (1 << 7)
#define USART_CR1_UE      (1 << 13)
#define USART_CR1_TE      (1 << 3)

/* ---------------- Clock and Baudrate ---------------- */
#define F_CPU             16000000UL
#define BAUD_RATE         115200

/* ---------------- Function Prototypes ---------------- */
void GPIO_Init(void);
void UART_Init(void);
void UART_SendChar(char ch);
void UART_SendNumber(uint16_t num);
void Adc_Init(void);
uint16_t Adc_Read_LM35(void);
void Delay_ms(volatile uint32_t ms);


int main(void)
{


    uint16_t adc_val;

    // clock enable

    GPIOA_PCLK_EN();
    GPIOB_PCLK_EN();
    USART1_PCLK_EN();
    ADC_PCLK_EN();

    Delay_ms(100);

    GPIO_Init();

    UART_Init();

    Adc_Init();

    while (1)
    {
        adc_val = Adc_Read_LM35();   // print  the adc value

        UART_SendNumber(adc_val);    // send through UART
        UART_SendChar('\n');

        Delay_ms(1000);
    }
}

void GPIO_Init(void)
{


    // PB6 = USART1_TX, PB7 = USART1_RX
    GPIOB->MODER &= ~((3U << 12) | (3U << 14)); // Clear mode
    GPIOB->MODER |= ((2U << 12) | (2U << 14));  // Alternate function mode

    GPIOB->AFR[0] &= ~((0xF << 24) | (0xF << 28)); // Clear AF
    GPIOB->AFR[0] |= ((7 << 24) | (7 << 28));      // AF7 = USART1

    // Optional: high speed & push-pull
    GPIOB->OSPEEDR |= ((3U << 12) | (3U << 14));   // High speed
    GPIOB->OTYPER  &= ~((1 << 6) | (1 << 7));     // Push-pull
    GPIOB->PUPDR   &= ~((3U << 12) | (3U << 14)); // No pull-up/down

    // PA0 = ADC input (analog)
    GPIOA->MODER |= (3U << 0);   // Analog mode
    GPIOA->PUPDR &= ~(3U << 0);  // No pull-up/down
}
// uart init
void UART_Init(void)
{
    USART1->CR1 &= ~USART_CR1_UE;  // clear the uart

    uint32_t divisor = F_CPU / BAUD_RATE;   // cpu - 16mhz and buad rate is 115200
    USART1->BRR = divisor;                  // store the value

    USART1->CR1 = USART_CR1_UE | USART_CR1_TE;  // enable the UART and transmitter bit
}

// uart send character
void UART_SendChar(char ch)
{
    while (!(USART1->SR & USART_SR_TXE)); // TXE wait for TXE = 1
    USART1->DR = ch;
}


// uart send number
void UART_SendNumber(uint16_t num)
{
    char digits[6];
    int i = 0;

    if (num == 0)
    {
        UART_SendChar('0');
        return;
    }

    while (num > 0)
    {
        digits[i++] = (num % 10) + '0';
        num /= 10;
    }

    for (int j = i - 1; j >= 0; j--)
        UART_SendChar(digits[j]);
}

//adc init
void Adc_Init(void)
{
    pADC1->CR2 = 0;
    pADC1->SMPR2 |= (7 << 0);   // Channel 0 we are using PA0 - ch0
    pADC1->CR2 |= (1 << 0);     // ADON  bit   enable the adc on
    Delay_ms(1);
}

// adc read
uint16_t Adc_Read_LM35(void)
{
    pADC1->SQR3 = 0;            // Channel 0
    pADC1->SQR1 = 0;            // 1 conversion single time
    pADC1->CR2 |= (1 << 30);    // Start conversion
    while (!(pADC1->SR & (1 << 1)));
    return pADC1->DR;
}

// delay
void Delay_ms(volatile uint32_t ms)
{
    for (volatile uint32_t i = 0; i < (ms * 4000); i++);
}
