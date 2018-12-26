/*
 * buffers.c
 *
 *  Created on: Nov 1, 2018
 *      Author: jake
 */


#include <stdint.h>
#include "buffers.h"

adc_buffer_t adcBuffer[ADC_BUFFER_SIZE];
volatile uint8_t spi_read_buffer[SPI_READ_BUFFER_SIZE];
volatile uint8_t spi_read_dma[SPI_READ_DMA_SIZE];
volatile uint8_t spi_write_dma[SPI_WRITE_DMA_SIZE];
//volatile pwm_pulse_t iebus_pulsemem[IEBUS_PULSEMEM_SIZE];
volatile uint8_t iebus_read_buffer[IEBUS_READ_BUFFER_SIZE];
RINGBUF spi_ringbuf;
RINGBUF iebus_ringbuf;
