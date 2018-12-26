/*
 * IEBUS
 */

#ifndef __IEBUS_TYPE__
#define __IEBUS_TYPE__

//#include "stm32f0xx_hal.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "adc.h"

typedef uint16_t iebus_cnt_t;
typedef volatile struct {
  iebus_cnt_t valid;
  iebus_cnt_t parity_errors[6];
  iebus_cnt_t incomplete;
  uint16_t pad;
  uint32_t overrun;
  uint32_t ignored;
  uint32_t too_long;
  uint32_t bit_ignored;
  uint32_t not_important;
  struct {
    uint32_t not_frame;
    uint32_t not_bit;
    uint32_t high_count;
    uint32_t double_start;
    uint32_t received;
    uint32_t overrun;
    uint32_t invalid_bit;
  } reader;
} IEBUS_STATUS;

typedef int16_t IEBUS_ADDRESS;
#define INVALID_IEBUS_ADDRESS   (IEBUS_ADDRESS)-1

// IEBus control
enum {
    IEBUS_CONTROL_SLAVE_STATUS_READ             = 0x0,  // reads slave status (SSR)
    IEBUS_CONTROL_DATA_READ_AND_LOCK            = 0x3,  // reads and locks data
    IEBUS_CONTROL_LOCK_LSB_ADDRESS_READ         = 0x4,  // reads lock address (lower 8 bits)
    IEBUS_CONTROL_LOCK_MSB_ADDRESS_READ         = 0x5,  // reads lock address (higher 4 bits)
    IEBUS_CONTROL_SLAVE_STATUS_READ_AND_UNLOCK  = 0x6,  // reads and unlocks slave status (SSR)
    IEBUS_CONTROL_DATA_READ                     = 0x7,  // reads data
    IEBUS_CONTROL_COMMAND_WRITE_AND_LOCK        = 0xa,  // writes and locks command
    IEBUS_CONTROL_DATA_WRITE_AND_LOCK           = 0xb,  // writes and locks data
    IEBUS_CONTROL_COMMAND_WRITE                 = 0xe,  // writes command
    IEBUS_CONTROL_DATA_WRITE                    = 0xf   // writes data
};

typedef uint8_t IEBUS_CONTROL;

// IEBus frame
// Frame fields
enum {
    IEBUS_FRAME_FIELD_BROADCAST_BIT,
    IEBUS_FRAME_FIELD_MASTER_ADDRESS,
    IEBUS_FRAME_FIELD_SLAVE_ADDRESS,
    IEBUS_FRAME_FIELD_CONTROL,
    IEBUS_FRAME_FIELD_MESSAGE_LENGTH,
    IEBUS_FRAME_FIELD_DATA
};

// Maximum length of data field in bytes
#define IEBUS_FRAME_MESSAGE_LENGTH_FIELD_MAXIMUM_VALUE      256
#define IEBUS_FRAME_DATA_FIELD_MAXIMUM_LENGTH               32 // Mode 1

// Maximum number of bits in a frame
#define IEBUS_FRAME_MAXIMUM_NUMBER_OF_BITS          (45 + (10 * IEBUS_FRAME_DATA_FIELD_MAXIMUM_LENGTH))

// number of bits (excluding parity and acknowledge) for each field
#define IEBUS_FRAME_FIELD_BITS_BROADCAST_BIT        1
#define IEBUS_FRAME_FIELD_BITS_MASTER_ADDRESS       12
#define IEBUS_FRAME_FIELD_BITS_SLAVE_ADDRESS        12
#define IEBUS_FRAME_FIELD_BITS_CONTROL              4
#define IEBUS_FRAME_FIELD_BITS_MESSAGE_LENGTH       8
#define IEBUS_FRAME_FIELD_BITS_DATA                 8

#define IEBUS_GET_FRAME_STRUCT_LENGTH(reader_frame_p) (reader_frame_p->data_length + 8)
typedef struct {
    uint8_t valid;
    uint8_t not_broadcast;
    uint8_t master_address[2];
    uint8_t slave_address[2];
    uint8_t control;
    uint8_t data_length;
    uint8_t data[IEBUS_FRAME_DATA_FIELD_MAXIMUM_LENGTH];
} IEBUS_FRAME;

enum {
    IEBUS_FRAME_BUFFER_INDEX_BROADCAST_BIT,
    IEBUS_FRAME_BUFFER_INDEX_MASTER_ADDRESS_MSB,
    IEBUS_FRAME_BUFFER_INDEX_MASTER_ADDRESS_LSB,
    IEBUS_FRAME_BUFFER_INDEX_SLAVE_ADDRESS_MSB,
    IEBUS_FRAME_BUFFER_INDEX_SLAVE_ADDRESS_LSB,
    IEBUS_FRAME_BUFFER_INDEX_CONTROL,
    IEBUS_FRAME_BUFFER_INDEX_MESSAGE_LENGTH,
    IEBUS_FRAME_BUFFER_INDEX_DATA
};

// IEBUS configuration start

#ifdef ENABLE_DIFFERENTIAL
#define SAMPLE_COUNT_FACTOR 4
#else
#define SAMPLE_COUNT_FACTOR (2)
#endif

//#define SAMPLE_COUNT(x) (int)((x) * 1 * SAMPLE_COUNT_FACTOR / 4)
#define SAMPLE_COUNT(x) (int)((x) * 7 / 12 * SAMPLE_COUNT_FACTOR)


// IEBUS configuration start

// spike detection
#define SAMPLE_COUNT_SPIKE_LIMIT        SAMPLE_COUNT(3)

// Idle state (5 ~ 10)
#define SAMPLE_COUNT_IDLE_MAX_LIMIT     SAMPLE_COUNT(21)
#define SAMPLE_COUNT_IDLE_MIN_LIMIT     SAMPLE_COUNT(2)

// Sync state (sync)
#define SAMPLE_COUNT_SYNC_MIN_LIMIT     SAMPLE_COUNT(8)

// Data state
#define SAMPLE_COUNT_DATA_LIMIT         SAMPLE_COUNT(8)

#define SAMPLE_COUNT_IDLE               SAMPLE_COUNT(100)
#define SAMPLE_COUNT_NOT_READY          SAMPLE_COUNT(0)

#define SAMPLE_COUNT_START_INDEX_MIN    SAMPLE_COUNT(70)
#define SAMPLE_COUNT_START_INDEX_MAX    SAMPLE_COUNT(91)

// Bit definitions
#define IEBUS_BIT_START   0x2
#define IEBUS_BIT_HIGH    0x1
#define IEBUS_BIT_LOW     0x0
#define IEBUS_BIT_INVALID 0x3

//BD1536SRWB//        #define logic_low_min_threshold       ADC_VALUE(78)
//BFH1736SB//         #define logic_low_min_threshold       ADC_VALUE(56)
//BFH1736TPB//        #define logic_low_min_threshold       ADC_VALUE(40)
//BFH1436SB//         #define logic_low_min_threshold       ADC_VALUE(57)
//BFH1336SB//         #define logic_low_min_threshold       ADC_VALUE(57)
//BFH1236SB//         #define logic_low_min_threshold       ADC_VALUE(57)
//BD1436SB//          #define logic_low_min_threshold       ADC_VALUE(78)
//BD1736SRWB//        #define logic_low_min_threshold       ADC_VALUE(97)
//BD1730SRWB//        #define logic_low_min_threshold       ADC_VALUE(78)
//BD1636SRWB//        #define logic_low_min_threshold       ADC_VALUE(85)
//D255RP2CYLE-HE//    #define logic_low_min_threshold       ADC_VALUE(85)
//F306TP2YFLHBDPA//   #define logic_low_min_threshold       ADC_VALUE(45)
//D205SP2COLE-H//     #define logic_low_min_threshold       ADC_VALUE(78)
//F366PP2YFLE-HDP//   #define logic_low_min_threshold       ADC_VALUE(56)
//D255HP2B//          #define logic_low_min_threshold       ADC_VALUE(78)
//F305NP2YFIN-H//     #define logic_low_min_threshold       ADC_VALUE(78)
//F306UP3YFLEHBDP//   #define logic_low_min_threshold       ADC_VALUE(78)
//D255LP2B-HP//       #define logic_low_min_threshold       ADC_VALUE(78)
//D255MP2LE-H//       #define logic_low_min_threshold       ADC_VALUE(78)
//F305PP2YFLE-H//     #define logic_low_min_threshold       ADC_VALUE(56)

#define LOGIC_LOW_MIN_THRESHOLD       100//ADC_VALUE(200)

#define MAIN_CONTROLLER_ADDRESS_MSB         0x08
#define MAIN_CONTROLLER_ADDRESS_LSB         0x00
#define BUTTON_ADDRESS_START_MSB            0x08
#define BUTTON_ADDRESS_START_LSB            0x80
#define DISPLAY_ADDRESS_1_MSB               0x08
#define DISPLAY_ADDRESS_1_LSB               0xd2
#define DISPLAY_ADDRESS_2_MSB               0x09
#define DISPLAY_ADDRESS_2_LSB               0x01
#define REMOTE_CONTROL_ADDRESS_1_MSB        0x08
#define REMOTE_CONTROL_ADDRESS_1_LSB        0xd0
#define REMOTE_CONTROL_ADDRESS_2_MSB        0x09
#define REMOTE_CONTROL_ADDRESS_2_LSB        0x32
#define REMOTE_CONTROL_ADDRESS_3_MSB        0x0b
#define REMOTE_CONTROL_ADDRESS_3_LSB        0x00


#endif /*__IEBUS_TYPE__*/
