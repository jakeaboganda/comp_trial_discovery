/*
 * iebus.h
 *
 *  Created on: Oct 22, 2018
 *      Author: jake
 */

#ifndef __IEBUS_H__
#define __IEBUS_H__

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "adc.h"
#include "iebus_type.h"

typedef void (*IEBUS_FRAME_RECEIVED_HANDLER)(IEBUS_FRAME *frame);

void iebus_initialize(
        IEBUS_FRAME_RECEIVED_HANDLER received_frame_handler,
        adc_buffer_t *buffer,
        uint32_t bufferSize);

void iebus_start(void);


void iebus_stop(void);

void iebus_service(void);

bool iebus_sendFrame(
        IEBUS_FRAME *frame);

void iebus_pushUs(uint32_t us);

#endif
