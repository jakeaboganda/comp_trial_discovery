/*
 * adc.c
 *
 *  Created on: Oct 17, 2018
 *      Author: jake
 */

#include "adc.h"
//#include "dpin.h"
#include <stdlib.h>

static adc_buffer_t *adcBuffer = NULL;
static uint32_t adcBufferSize = 0;

struct
{
    ADC_DIFF_HANDLER adcDiffHandler;
    ADC_IS_HIGH_PRIO_HANDLER adcIsHighPrioHandler;
    ADC_IS_STOPPED_HANDLER adcIsStoppedHandler;
} handlers = {NULL, NULL, NULL};

#define MODE0BUFFERSIZE 8

#define ADCMODE0 0
#define ADCMODE1 1

static volatile int adcMode;
static inline void initializeAdcDma(void);
static inline void calibrate(void);
static inline void enableAdcIsr(void);
static inline void enableAdc(void);

uint32_t dma_irq = 0;

static uint32_t calibration_value;

#ifdef DEBUG
uint32_t dmairq = 0U;
#endif
uint32_t dmairqCnt[3] = {0,0,0};

void DMA1_Channel1_IRQHandler(void);
void DMA1_Channel1_IRQHandler(void)
{
#ifdef DEBUG
    //dmairq++;
#endif
    uint32_t dma_status = DMA1->ISR;
    uint32_t dma_setting = DMA1_Channel1->CCR;

    // detect half transfer complete
    if((0 != (dma_status & (DMA_FLAG_HT1))) && (0 != (dma_setting & DMA_IT_HT)))
    {
        DMA1->IFCR |= DMA_FLAG_HT1;
#ifdef DEBUG_WITH_DPIN
        dpin_set(ADC_BUSY_0);
#endif
        handlers.adcDiffHandler(adcBuffer, (int)(adcBufferSize / 2));
        //iebus_adcDiffHandler(adcBuffer, (int)(adcBufferSize / 2));
#ifdef DEBUG_WITH_DPIN
        dpin_reset(ADC_BUSY_0);
#endif
        dmairqCnt[0]++;
    }

    // detect full transfer
    else if((0 != (dma_status & (DMA_FLAG_TC1))) && (0 != (dma_setting & DMA_IT_TC)))
    {
        DMA1->IFCR |= DMA_FLAG_TC1;
#ifdef DEBUG_WITH_DPIN
        dpin_set(ADC_BUSY_1);
#endif
        handlers.adcDiffHandler(adcBuffer + adcBufferSize / 2, (int)(adcBufferSize / 2));
        //iebus_adcDiffHandler(adcBuffer + adcBufferSize / 2, (int)(adcBufferSize / 2));
#ifdef DEBUG_WITH_DPIN
        dpin_reset(ADC_BUSY_1);
#endif
        dmairqCnt[1]++;
    }

    // detect error
    else if((0 != (dma_status & (DMA_FLAG_TE1 << 0))) && (0 != (dma_setting & DMA_IT_TE)))
    {
        DMA1->IFCR |= DMA_FLAG_TE1;
        dmairqCnt[2]++;
    }
}

#define GPIO_MODE_R_ANALOG  0x3
#define GPIO_MODE_R_OUTPUT  0x1
#define GPIO_MODE_R_INPUT   0x0

static inline void setGPIOAMODE(uint8_t modeValue, int gpioN)
{
    uint32_t value = GPIOA->MODER;
    value = value & (~((uint32_t)0x3 << (gpioN * 2)));
    value |= (uint32_t)((modeValue & 0x3) << (gpioN * 2));
    GPIOA->MODER = value;
}

static inline void initializeGpio(void)
{
    // Initialize ADC GPIOs
    // Init PA0 as analog inputs
    setGPIOAMODE(GPIO_MODE_R_ANALOG, 0); // set PA0 as analog
}

static inline void enableAdcIsr(void)
{
    // Enable ADC ISR
    if((ADC1->ISR & ADC_ISR_ADRD) != 0)
    {
        ADC1->ISR |= ADC_ISR_ADRD;
    }
}

static inline void initializeAdcDma(void)
{
    bool isDmaCircular;
    uint32_t bufferSize;
    bool hasHalfTransferInterrupt;

    if(adcMode == ADCMODE0)
    {
        isDmaCircular = false;
        bufferSize = MODE0BUFFERSIZE;
        hasHalfTransferInterrupt = false;
    }
    else
    {
        isDmaCircular = true;
        bufferSize = adcBufferSize;
        hasHalfTransferInterrupt = true;
    }

    // Initialize DMA Module for ADC transfer
    DMA1_Channel1->CCR =
                        (uint32_t)0 |
                        (0x0 << (16))       |
                        (0x0 << (14))       | // Memory to memory disabled
                        (0x3 << (12))       | // Very High priority
#if ADC_RESOLUTION == ADC_RESOLUTION_12
                        (0x1 << (10))       | // Memory is 16 bits
                        (0x2 << (8))        | // Peripheral is 32 bits
#elif ADC_RESOLUTION == ADC_RESOLUTION_8
                        (0x0 << (10))       | // Memory is 8 bits
                        (0x2 << (8))        | // Peripheral is 32 bits
#else
#error "invalid resolution"
#endif
                        (0x1 << (7))        | // Memory Increment Enable
                        (0x0 << (6))        | // Peripheral Increment Disable
                        (isDmaCircular ?
                        (0x1 << (5)) :
                        (0x0 << (5)))       | // Circular Mode is variable
                        (0x0 << (4))        | // Peripheral to Memory
                        (0x1 << (3))        | // Transfer Error interrupt disable
                        (hasHalfTransferInterrupt ?
                        (0x1 << (2)) :
                        (0x0 << (2)))       | // Half transfer interrupt variable
                        (0x1 << (1))        | // Full transfer interrupt enable
                        0;

    DMA1_Channel1->CPAR = (uint32_t) (&(ADC1->DR)); // Data to save is in the ADC data register

    DMA1_Channel1->CMAR = (uint32_t)(adcBuffer);    // Memory to transfer to is the buffer input

    DMA1_Channel1->CNDTR = bufferSize;              // Data to transfer is variable bufferSize
    
    //ADC1_COMMON->CCR &= ~ADC1_CCR_CKMODE_Msk;
    //ADC1_COMMON->CCR |= 0x1U << ADC1_CCR_CKMODE_Pos;

    // Initialize ADC module
    ADC1->CFGR =    0 |
                    ADC_CFGR_CONT       | // Continuous conversion adcMode (CONT=1)
                    ADC_CFGR_OVRMOD     | // Overrun
                    (0x0 << (5))        | // Right-aligned
#if ADC_RESOLUTION == 12
                    (0x0 << (3))        |   // 12-bit ADC
#elif ADC_RESOLUTION == 8
                    (0x2 << (3))        |   // 8-bit ADC
#else
#error "invalid resolution"
#endif
                    (0x0 << (2))        | // Forward scan
                    ADC_CFGR_DMACFG     | // DMA configuration
                    ADC_CFGR_DMAEN      | // DMA access enable
                    0;

#if 0
    ADC1->CHSELR =
                        0 |
                        (0x1 << (5))        | // PA5
                        (0x1 << (6))        | // PA6
                        0;
#endif

#ifdef ENABLE_DIFFERENTIAL
    ADC1->SQR1 =        //(0x2) << 6  |   // Channel 2 is second
                        (0x1) << 6  |   // Channel 1 is first
                        (0x0) << 0;     // 0 = Length of the conversions - 1
#else
    ADC1->SQR1 =        (0x1) << 6  |   // Channel 1 (PA0) is first
                        (0x0) << 0;     // 0 = Length of the conversions - 1
#endif

    ADC1->SQR2 =        0x0;
    ADC1->SQR3 =        0x0;
    ADC1->SQR4 =        0x0;

    ADC1->DIFSEL &=     (uint32_t)~(0xFFFF);
#ifdef ENABLE_DIFFERENTIAL
    ADC1->DIFSEL |=     (0x1) << 1;     // Channel 1 is in differential mode with Channel 2
#endif

    ADC1->SMPR1 = 0;                // 1 sample will take 1.5 ADC clock cycles
   // ADC1->SMPR1 =       (0x1) << 3; // 1 sample will take 2.5 ADC clock cycles
    //ADC1->SMPR2 = 0;                // 1 sample will take 1.5 ADC clock cycles

    ADC1->IER = 0;                  // ADC will have no interrupts:
                                    // We'll use DMA half and full interrupts instead.
}

static inline void enableAdc(void)
{
    // Ready modules for starting

    // Enable DMA
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    // Enable ADC
    ADC1->CFGR |= ADC_CFGR_DMAEN;
}

static inline void disableAdc(void)
{
    // Ready modules for starting

    // Enable DMA
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;

    // Enable ADC
    ADC1->CFGR &= ~ADC_CFGR_DMAEN;
}

static inline void calibrate(void)
{
    // Initialize ADC
    if((ADC1->CR & ADC_CR_ADEN) != 0)
    {
        ADC1->CR |= ADC_CR_ADDIS;
    }

    ADC1->CR &= ~ADC_CR_ADVREGEN;
    __NOP();
    ADC1->CR |= ADC_CR_ADVREGEN_0;
    __NOP();

    while((ADC1->CR & ADC_CR_ADEN) != 0);
    ADC1->CFGR &= ~ADC_CFGR_DMAEN;
    for(int i=0; i < 255; i++)
    {
        __NOP();
    }

#ifdef ENABLE_DIFFERENTIAL
    ADC1->CR |= ADC_CR_ADCALDIF;
    ADC1->CR |= ADC_CR_ADCAL;
    while((ADC1->CR & ADC_CR_ADCAL) != 0);
#else
    ADC1->CR |= ADC_CR_ADCAL;
    while((ADC1->CR & ADC_CR_ADCAL) != 0);
#endif

    calibration_value = ADC1->CALFACT;
}

static inline void gpioAClkEnable(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;      // Enable GPIOA
}

static inline 
void RCCADCCLKConfig(uint32_t RCC_PLLCLK)
{
  uint32_t tmp = 0;

  tmp = (RCC_PLLCLK >> 28);
  
  /* Clears ADCPRE34 bits */
  if (tmp != 0)
  {
    RCC->CFGR2 &= ~((uint32_t)0x000001F0);//~RCC_CFGR2_ADCPRE34;
  }
   /* Clears ADCPRE12 bits */
  else
  {
    RCC->CFGR2 &= ~((uint32_t)0x00003E00);//~RCC_CFGR2_ADCPRE12;
  }
  /* Set ADCPRE bits according to RCC_PLLCLK value */
  RCC->CFGR2 |= RCC_PLLCLK;
}

static inline void adc1ClkEnable(void)
{
    //__HAL_RCC_ADC1_CLK_ENABLE();
    //__HAL_RCC_ADC1_CONFIG(RCC_ADC1PLLCLK_DIV1);RCC_CFGR2_ADC1PRES_DIV1
    RCCADCCLKConfig((RCC_CFGR2_ADCPRE12_DIV1));//RCC_ADC12PLLCLK_Div1);
    RCC->CFGR2 |= ((uint32_t)0x00000110);  // Enable ADC Clock
    RCC->AHBENR |= ((uint32_t)0x10000000);      // Enable ADC1 Clock
    RCC->CR = 0x03035A83;
#if 0
    RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div1);
    RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV2;  // Enable ADC Clock
    RCC->AHBENR |= RCC_AHBENR_ADC12EN;      // Enable ADC1 Clock
#endif
}

static inline void dma1ClkEnable(void)
{
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;       // Enable DMA
}

// This version of single ended ADC will now use PA0
void adc_initialize(
    adc_buffer_t *buffer,
    uint32_t bufferSize,
    ADC_DIFF_HANDLER diffHandler,
    ADC_IS_STOPPED_HANDLER isStoppedHandler)
{
    bufferSize = (bufferSize / 2) * 2; // Ensure even

    // Initialize buffer pointers
    adcBuffer = buffer;
    adcBufferSize = bufferSize;

    // Set ADC diff handlers
    handlers.adcDiffHandler = diffHandler;
    handlers.adcIsStoppedHandler = isStoppedHandler;

    // Initialize Clocks
    gpioAClkEnable();
    adc1ClkEnable();
    dma1ClkEnable();

    // Initialize Interrupts
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    NVIC_SetPriority(DMA1_Channel1_IRQn, 2);

    initializeGpio();

    calibrate();

    adcMode = ADCMODE1;
    initializeAdcDma();

    enableAdcIsr();

    ADC1->CR |= ADC_CR_ADEN;
    while((ADC1->ISR & ADC_ISR_ADRD) == 0);

#ifdef DEBUG_WITH_DPIN
    // Setup debug pin
    dpin_initialize();
#endif
}

void adc_start(void)
{
    enableAdc();
    ADC1->CR |= ADC_CR_ADSTART;
}

void adc_stop(void)
{
    disableAdc();
    ADC1->CR &= ~ADC_CR_ADSTART;
}
