
#ifndef __BUFFER__
#define __BUFFER__

#include <stdint.h>
//#include "pwm.h"
#include "ringbuf.h"
#include "adc.h"

#define ADC_BUFFER_SIZE 512
extern adc_buffer_t adcBuffer[ADC_BUFFER_SIZE];

#define SPI_READ_BUFFER_SIZE  1024
extern volatile uint8_t spi_read_buffer[SPI_READ_BUFFER_SIZE];

#define SPI_READ_DMA_SIZE 64
extern volatile uint8_t spi_read_dma[SPI_READ_DMA_SIZE];

#define SPI_WRITE_DMA_SIZE 64
extern volatile uint8_t spi_write_dma[SPI_WRITE_DMA_SIZE];

//#define IEBUS_PULSEMEM_SIZE 1024
//extern volatile pwm_pulse_t iebus_pulsemem[IEBUS_PULSEMEM_SIZE];

#define IEBUS_READ_BUFFER_SIZE 512
extern volatile uint8_t iebus_read_buffer[IEBUS_READ_BUFFER_SIZE];

extern RINGBUF spi_ringbuf;

extern RINGBUF iebus_ringbuf;

#endif //__BUFFER__

