/*
 * stm32401xx.h
 *
 *  Created on: Oct 23, 2025
 *      Author: sanjeeviram
 */

#ifndef STM32401XX_H_
#define STM32401XX_H_


//BASE ADDRESS OF THE MCU MEMMORIES
#include <stdint.h>


#define FLASH_BASEADDR      0x08000000U     //address of flash memmory (256kb)
#define SRAM1_BASEADDR      0x20000000U     // addresss of sram1 (64kbs)
#define ROM_BASEADDR        0x1FFF0000U     //adress of system memory(ROM) (30kb)
#define SRAM                SRAM1_BASEADDR


// AHBx and APBx BUS PHERIPHERAL BASE ADRESS

#define PERIPH_BASEADDR        0x40000000U
#define APB1PERIPH_BASEADDR    PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR    0x40010000U
#define AHB1PERIPH_BASEADDR    0x40020000U
#define AHB2PERIPH_BASEADDR    0x50000000U



//BASE ADRESSS OF PERIPHERALS WHICH IS IN AHB1 BUS
//with base adress and offset;

#define GPIOA_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0000UL)
#define GPIOB_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0400UL)
#define GPIOC_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0800UL)
#define GPIOD_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0C00UL)
#define GPIOE_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1000UL)
#define RCC_BASEADDR            (AHB1PERIPH_BASEADDR + 0x3800UL)


//BASE ADDRESS OF PERIPHERALS WHICH IS IN APB1 BUS

#define I2C1_BASEADDR            (APB1PERIPH_BASEADDR + 0x5400UL)
#define I2C2_BASEADDR            (APB1PERIPH_BASEADDR + 0x5800UL)
#define I2C3_BASEADDR            (APB1PERIPH_BASEADDR + 0x5C00UL)

#define SPI2_BASEADDR           (APB1PERIPH_BASEADDR + 0x3800UL)
#define SPI3_BASEADDR           (APB1PERIPH_BASEADDR + 0x3C00UL)

#define USART2_BASEADDR         (APB1PERIPH_BASEADDR + 0x4400UL)

#define ADC1_BASEADDR            (APB2PERIPH_BASEADDR + 0x2000UL)


//BASE ADDRESS OF PHERIPHERALS WHIC IS IN APB2 BUS

#define SPI1_BASEADDR           (APB2PERIPH_BASEADDR + 0x3000UL)
#define SPI4_BASEADDR           (APB2PERIPH_BASEADDR + 0x3400UL)

#define USART1_BASEADDR         (APB2PERIPH_BASEADDR + 0x1000UL)
#define USART6_BASEADDR         (APB2PERIPH_BASEADDR + 0x1400UL)

#define SYSCFG_BASEADDR        (APB2PERIPH_BASEADDR + 0x3800UL)
#define EXTI_BASEADDR          (APB2PERIPH_BASEADDR + 0x3C00UL)


//PERIPHERAL REGISTER DEFINITION STRUCTURE FOR GPI0

typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];   // AFR[0] - alternate function low register   AFR[1] - alternate function high register
} GPIO_RegDef_t;

#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASEADDR)
GPIO_RegDef_t *pGPIOA =GPIOA;

#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASEADDR)
GPIO_RegDef_t *pGPIOB =GPIOB;

#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASEADDR)



//PERIPHERAL REGISTER DEFINITION STRUCTURE OF ADC

typedef struct
{
    volatile uint32_t SR;        // 0x00 - Status Register
    volatile uint32_t CR1;       // 0x04 - Control Register 1
    volatile uint32_t CR2;       // 0x08 - Control Register 2
    volatile uint32_t SMPR1;     // 0x0C - Sample Time Register 1
    volatile uint32_t SMPR2;     // 0x10 - Sample Time Register 2
    volatile uint32_t JOFR1;     // 0x14 - Injected Channel Offset 1
    volatile uint32_t JOFR2;     // 0x18
    volatile uint32_t JOFR3;     // 0x1C
    volatile uint32_t JOFR4;     // 0x20
    volatile uint32_t HTR;       // 0x24 - High Threshold
    volatile uint32_t LTR;       // 0x28 - Low Threshold
    volatile uint32_t SQR1;      // 0x2C - Regular Sequence Register 1
    volatile uint32_t SQR2;      // 0x30 - Regular Sequence Register 2
    volatile uint32_t SQR3;      // 0x34 - Regular Sequence Register 3
    volatile uint32_t JSQR;      // 0x38 - Injected Sequence Register
    volatile uint32_t JDR1;      // 0x3C - Injected Data Register 1
    volatile uint32_t JDR2;      // 0x40
    volatile uint32_t JDR3;      // 0x44
    volatile uint32_t JDR4;      // 0x48
    volatile uint32_t DR;        // 0x4C - Regular Data Register
} ADC_RegDef_t;

#define ADC1  (ADC_RegDef_t*)ADC1_BASEADDR

ADC_RegDef_t *pADC1 = ADC1;


//PERIPHERAL REGISTER DEFINITION STRUCTURE OF I2C

typedef struct
{
	volatile uint32_t   SR;
	volatile uint32_t   DR;
	volatile uint32_t   BRR;
	volatile uint32_t   CR1;
	volatile uint32_t   CR2;
	volatile uint32_t   CR3;
	volatile uint32_t   GTPR;
}USART_Regdef_t;

#define USART1 ((USART_Regdef_t*)USART1_BASEADDR)




//Peripheral register defintion structure of SPI

typedef struct
{
    volatile uint32_t CR1;       // 0x00 Control register 1
    volatile uint32_t CR2;       // 0x04 Control register 2
    volatile uint32_t SR;        // 0x08 Status register
    volatile uint32_t DR;        // 0x0C Data register
    volatile uint32_t CRCPR;     // 0x10 CRC polynomial register
    volatile uint32_t RXCRCR;    // 0x14 RX CRC register
    volatile uint32_t TXCRCR;    // 0x18 TX CRC register
    volatile uint32_t I2SCFGR;   // 0x1C I2S configuration register
    volatile uint32_t I2SPR;     // 0x20 I2S prescaler register
} SPI_RegDef_t;


#define SPI1   ((SPI_RegDef_t*)SPI1_BASEADDR)

#define SPI2    ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3    ((SPI_RegDef_t*)SPI3_BASEADDR)


//PERIPHERAL REGISTER DEFINITION STRUCTURE OF I2C

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
	volatile uint32_t FLTR;

}I2C_RegDef_t;

#define I2C1 ((I2C_RegDef_t*)I2C1_BASEADDR)




//PERIPHERAL REGISTER DEFINITION STRUCTURE FOR RCC

typedef struct {
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    uint32_t RESERVED0[2];
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    uint32_t RESERVED1[2];
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    uint32_t RESERVED2[2];
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
    uint32_t RESERVED3[2];
    volatile uint32_t AHB1LPENR;
    volatile uint32_t AHB2LPENR;
    uint32_t RESERVED4[2];
    volatile uint32_t APB1LPENR;
    volatile uint32_t APB2LPENR;
    uint32_t RESERVED5[2];
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
    uint32_t RESERVED6[2];
    volatile uint32_t SSCGR;
    volatile uint32_t PLLI2SCFGR;
    volatile uint32_t DCKCFGR;
} RCC_RegDef_t;

#define RCC ((RCC_RegDef_t*)RCC_BASEADDR)


// Peripheral definition structure for EXTI r

typedef struct {
    volatile uint32_t IMR;
    volatile uint32_t EMR;
    volatile uint32_t RTSR;
    volatile uint32_t FTSR;
    volatile uint32_t SWIER;
    volatile uint32_t PR;
} EXTI_RegDef_t;


#define EXTI ((EXTI_RegDef_t*)EXTI_BASEADDR )


// peripheral definintion struct for SYSTEM CONFIGURATION

typedef struct {
    volatile uint32_t MEMRMP;
    volatile uint32_t PMC;
    volatile uint32_t EXTICR[4];
    volatile uint32_t RESERVED[2];
    volatile uint32_t CMPCR;
} SYSCFG_RegDef_t;   // use the same name here

#define SYSCFG  ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)



//CLOCK ENABLE MACROS FOR GPIO PERIPHERALS

#define GPIOA_PCLK_EN()  (RCC->AHB1ENR |= (1<<0) )
#define GPIOB_PCLK_EN()  (RCC->AHB1ENR |= (1<<1) )
#define GPIOC_PCLK_EN()  (RCC->AHB1ENR |= (1<<2) )
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1<<14))
#define SPI2_PCLK_EN()  (RCC->APB1ENR |= (1<<14))
#define I2C1_PCLK_EN()  (RCC->APB1ENR |= (1<<21))
#define USART1_PCLK_EN() (RCC->APB2ENR |= (1<<4))
#define ADC_PCLK_EN()    (RCC->APB2ENR |= (1<<8))



//CLOCK DISABLE MACROS FOR GPIO PERIPHERALS

#define GPIOA_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<0) )
#define GPIOB_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<1) )

#define GPIO_BASEADDR_TO_CODE(x)   ((x == GPIOA) ? 0 : \
                                      (x == GPIOB) ? 1 : \
                                      (x == GPIOC) ? 2 : 0)


// interrupt request for stm32 mcu

#define IRQ_NO_EXTI0  6
#define IRQ_NO_EXTI1  7
#define IRQ_NO_EXTI2  8
#define IRQ_NO_EXTI3  9
#define IRQ_NO_EXTI4  10
#define IRQ_NO_EXTI9_5  23
#define IRQ_NO_EXTI15_10  40


//Arm processor NVIC ISERx register address

#define NVIC_ISER0    ((volatile uint32_t*)0xE000E100 )
#define NVIC_ISER1    ((volatile uint32_t*)0xE000E104 )
#define NVIC_ISER2    ((volatile uint32_t*)0xE000E108 )
#define NVIC_ISER3    ((volatile uint32_t*)0xE000E10c )

// Arm processor NVIC ICERx register Addresses

#define NVIC_ICER0     ((volatile uint32_t*)0XE000180)
#define NVIC_ICER1     ((volatile uint32_t*)0XE000184)
#define NVIC_ICER2     ((volatile uint32_t*)0XE000188)
#define NVIC_ICER3     ((volatile uint32_t*)0XE00018C)

// Arm Cortex mx processor priority register address calculation

#define NVIC_PR_BASE_ADDR  ((volatile uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED    4


#define ENABLE            1
#define DISABLE           0
#define SET               ENABLE
#define RESET             DISABLE
#define GPIO_PIN_SET      SET
#define GPIO_PIN_RESET    RESET



#endif /* STM32401XX_H_ */
