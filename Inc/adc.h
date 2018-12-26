
#ifndef __ADC__
#define __ADC__


#include "stm32f3xx_hal.h"

#include <stdint.h>
#include <stdbool.h>

//#define DEBUG_WITH_DPIN

#ifdef DEBUG_WITH_DPIN
#include "dpin.h"
#define ADC_BUSY_0 DPIN(0)
#define ADC_BUSY_1 DPIN(1)
#define SPI_PROCESSING DPIN(2)
#define IEBUS_READING DPIN(3)
#define IEBUS_SENDING DPIN(4)
#endif

//#define ENABLE_DIFFERENTIAL

#define ADC_RESOLUTION_8        8
#define ADC_RESOLUTION_12       12

#define ADC_RESOLUTION          ADC_RESOLUTION_12

#if ADC_RESOLUTION == ADC_RESOLUTION_12
typedef volatile uint16_t adc_buffer_t;
#elif ADC_RESOLUTION == ADC_RESOLUTION_8
typedef volatile uint8_t adc_buffer_t;
#else
#error
#endif

#define ADC_MAXIMUM_VALUE        ((0x1 << ADC_RESOLUTION) - 1)
#define ADC_REFERENCE_VOLTAGE    (3300) //millivolts
//#define ADC_REFERENCE_VOLTAGE    (3000) //millivolts
#define ADC_VALUE(voltage)       ((voltage) * ADC_MAXIMUM_VALUE / ADC_REFERENCE_VOLTAGE)
//#define ADC_VALUE(voltage)       (((voltage) * ADC_MAXIMUM_VALUE / ADC_REFERENCE_VOLTAGE) + 1660)

#define adc_to_int(adc)                (int)((adc) & ADC_MAXIMUM_VALUE)
//#define get_adc_difference_(valA, valB) ((valA < valB) ? 0 : (adc_to_int((valA)) - adc_to_int((valB))))
#define get_adc_difference_(valA, valB) (adc_to_int((valA)) - adc_to_int((valB)))
#ifdef ENABLE_DIFFERENTIAL
#define get_adc_difference(val) get_adc_difference_(val, ((ADC_MAXIMUM_VALUE)/2))
#else
#define get_adc_difference(valA, valB) get_adc_difference_(valA, valB)
#endif

typedef void (*ADC_DIFF_HANDLER)(adc_buffer_t *data, int dataSize);
typedef void (*ADC_IS_HIGH_PRIO_HANDLER)(adc_buffer_t *data, int dataSize);
typedef void (*ADC_IS_STOPPED_HANDLER)(void);

void adc_initialize(
    adc_buffer_t *buffer,
    uint32_t bufferSize,
    ADC_DIFF_HANDLER diffHandler,
    ADC_IS_STOPPED_HANDLER isStoppedHandler);

void adc_start(void);

void adc_stop(void);


#endif // __ADC__
