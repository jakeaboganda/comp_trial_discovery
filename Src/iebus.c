/*
 * iebus.c
 *
 *  Created on: Oct 22, 2018
 *      Author: jake
 */

#include "iebus.h"
#include "ringbuf.h"
#include "buffers.h"
//#include "pwm.h"
#include "iebus_type.h"
#include "adc.h"

static IEBUS_FRAME_RECEIVED_HANDLER iebusFrameReceivedHandler;
static adc_buffer_t *iebusAdcBuffer = NULL;
static uint32_t iebusAdcBufferSize = 0;

static bool iebus_is_busy = true;

static inline void analyzeBit(int bit);

#define startBitIsDetected(s) ((s) >= SAMPLE_COUNT_START_INDEX_MIN && (s) <= SAMPLE_COUNT_START_INDEX_MAX)


#ifdef ENABLE_DIFFERENTIAL
#define isAdcDifferenceLevelLow(difference) ((difference) > LOGIC_LOW_MIN_THRESHOLD ) //90
#else
#define isAdcDifferenceLevelLow(difference) ((difference) > LOGIC_LOW_MIN_THRESHOLD)
#endif
//#define TMPFRAME_DEBUG
#ifdef TMPFRAME_DEBUG
IEBUS_FRAME tmpframe;
#endif

IEBUS_STATUS iebus_status = {0, {0}, 0, 0, 0, 0, 0, 0, 0, {0, 0, 0, 0, 0, 0, 0}};

/* Frame buffer. Note we only have one frame*/
static IEBUS_FRAME read_iebus_frame;

// Frame field information
typedef struct {
    uint8_t id;
    bool has_parity;
    bool has_ack;
    uint8_t data_bits;
    uint8_t total_bits;
    uint16_t bit_mask;
} IEBUS_FRAME_FIELD;

#define IEBUS_FRAME_FIELD_INITIALIZER(id, has_parity, has_ack, number_data_bits) \
    [id] = { \
            id, \
            has_parity, \
            has_ack, \
            number_data_bits, \
            number_data_bits + has_parity + has_ack, \
            0x1 << (number_data_bits - 1) \
    }

static const IEBUS_FRAME_FIELD iebusFrameField[] = {
        IEBUS_FRAME_FIELD_INITIALIZER(IEBUS_FRAME_FIELD_BROADCAST_BIT, false, false, IEBUS_FRAME_FIELD_BITS_BROADCAST_BIT),
        IEBUS_FRAME_FIELD_INITIALIZER(IEBUS_FRAME_FIELD_MASTER_ADDRESS, true, false, IEBUS_FRAME_FIELD_BITS_MASTER_ADDRESS),
        IEBUS_FRAME_FIELD_INITIALIZER(IEBUS_FRAME_FIELD_SLAVE_ADDRESS, true, true, IEBUS_FRAME_FIELD_BITS_SLAVE_ADDRESS),
        IEBUS_FRAME_FIELD_INITIALIZER(IEBUS_FRAME_FIELD_CONTROL, true, true, IEBUS_FRAME_FIELD_BITS_CONTROL),
        IEBUS_FRAME_FIELD_INITIALIZER(IEBUS_FRAME_FIELD_MESSAGE_LENGTH, true, true, IEBUS_FRAME_FIELD_BITS_MESSAGE_LENGTH),
        IEBUS_FRAME_FIELD_INITIALIZER(IEBUS_FRAME_FIELD_DATA, true, true, IEBUS_FRAME_FIELD_BITS_DATA)
};

#define DEBUG_LOG_SAMPLE_COUNTS
#ifdef DEBUG_LOG_SAMPLE_COUNTS
#define DEBUG_LOG_SAMPLE_COUNTS_BUFFER_SIZE 256
int log_sample_counts_buffer[DEBUG_LOG_SAMPLE_COUNTS_BUFFER_SIZE];
int log_sample_counts_buffer_index = 0;
#endif

//#define DEBUG_LOG_SAMPLE_COUNTS_FRAME
#ifdef DEBUG_LOG_SAMPLE_COUNTS_FRAME
#define DEBUG_LOG_SAMPLE_COUNTS_FRAME_BUFFER_SIZE 256
int log_sample_counts_frame_buffer[DEBUG_LOG_SAMPLE_COUNTS_FRAME_BUFFER_SIZE];
int log_sample_counts_frame_buffer_index = 0;
#define log_sample_counts_frame_add(new_count) \
{\
    log_sample_counts_frame_buffer[log_sample_counts_frame_buffer_index] = (new_count);\
    log_sample_counts_frame_buffer_index = (log_sample_counts_frame_buffer_index + 1) % DEBUG_LOG_SAMPLE_COUNTS_FRAME_BUFFER_SIZE; \
    log_sample_counts_frame_buffer[log_sample_counts_frame_buffer_index] = -1234567;\
}
#endif

#define DEBUG_LOG_ADC_DIFF
#ifdef DEBUG_LOG_ADC_DIFF
#define DEBUG_LOG_ADC_DIFF_BUFFER_SIZE 512
int log_adc_diff_buffer[DEBUG_LOG_ADC_DIFF_BUFFER_SIZE];
int log_adc_diff_buffer_index = 0;
#endif

//#define DEBUG_LOG_BITS
#ifdef DEBUG_LOG_BITS
#define DEBUG_LOG_BITS_BUFFER_SIZE 32
uint8_t log_bits_buffer[DEBUG_LOG_BITS_BUFFER_SIZE];
uint8_t log_bits_buffer_index = 0;
#endif

/** Is frame important?
 */
static inline bool isAddressMainController(uint8_t address[2])
{
    if( (address[0] == MAIN_CONTROLLER_ADDRESS_MSB) &&
        (address[1] == MAIN_CONTROLLER_ADDRESS_LSB))
    {
        return true;
    }
    return false;
}

static inline bool isAddressButton(uint8_t address[2])
{
    if( (address[0] == BUTTON_ADDRESS_START_MSB) &&
        (address[1] >= BUTTON_ADDRESS_START_LSB) &&
        (address[1] <= BUTTON_ADDRESS_START_LSB + 100))
    {
        return true;
    }
    return false;
}

static inline bool isAddressDisplayAddress(uint8_t address[2])
{
    if( (address[0] == DISPLAY_ADDRESS_1_MSB) &&
        (address[1] == DISPLAY_ADDRESS_1_LSB) &&
        (address[1] == DISPLAY_ADDRESS_2_MSB) &&
        (address[1] == DISPLAY_ADDRESS_2_LSB))
    {
        return true;
    }
    return false;
}

static inline bool isAddressRemoteControl(uint8_t address[2])
{
    if( (address[0] == REMOTE_CONTROL_ADDRESS_1_MSB) &&
        (address[1] == REMOTE_CONTROL_ADDRESS_1_LSB) &&
        (address[1] == REMOTE_CONTROL_ADDRESS_2_MSB) &&
        (address[1] == REMOTE_CONTROL_ADDRESS_2_LSB) &&
        (address[1] == REMOTE_CONTROL_ADDRESS_3_MSB) &&
        (address[1] == REMOTE_CONTROL_ADDRESS_3_LSB))
    {
        return true;
    }
    return false;
}

static inline bool isFrameImportant(IEBUS_FRAME *frame)
{
    if(isAddressMainController(frame->master_address))
    {
        if(isAddressDisplayAddress(frame->slave_address))
        {
            return true;
        }
        if(isAddressButton(frame->slave_address))
        {
            return true;
        }
        return false;
    }

    if(isAddressButton(frame->master_address))
    {
        return true;
    }

    if(isAddressRemoteControl(frame->master_address))
    {
        return true;
    }
    return false;
}

static volatile bool bus_is_busy = true;
//static inline void pushBits(adc_buffer_t *data, int dataLength)
#define HIGH_PRIO_INCREMENT 2 * SAMPLE_COUNT_FACTOR
#define LOW_PRIO_INCREMENT 2 * SAMPLE_COUNT_FACTOR

static inline uint8_t pushBits(bool new_high)
{
	static enum {
		IEBUS_STATE_START,
		IEBUS_STATE_IDLE,
		IEBUS_STATE_SYNC,
		IEBUS_STATE_DATA,
		IEBUS_STATE_STOP
	} iebus_state = IEBUS_STATE_START;

    static int sample_count_backup = 0;
    static int sample_count = 0;

    static bool high = false;
    static bool last_high = false;

    static int sample_count_low = SAMPLE_COUNT_NOT_READY;
    static int sample_count_high = SAMPLE_COUNT_NOT_READY;

    // Determine the current bus state
    high = new_high;
	
	if(iebus_state == IEBUS_STATE_START)
	{
		// Detect a state transition
		if(last_high != high) //delta
		{
			// Spike detection
			if(sample_count < SAMPLE_COUNT_SPIKE_LIMIT)
			{
				// Recover
				sample_count = sample_count_backup + sample_count + 1;
				sample_count_backup = 0;
			}
			else
			{
				// The previous bus state was low. This means that bus has just shifted from low to high.
				if(!last_high)
				{
					// Ready the low sample counts for analysis
					sample_count_low = sample_count;
					// Empty the high sample counts
					sample_count_high = SAMPLE_COUNT_NOT_READY;
				}

				// The previous bus state was high. This means that bus has just shifted from high to low.
				else
				{
					// Empty the low sample counts
					sample_count_low = SAMPLE_COUNT_NOT_READY;
					// Ready the high sample counts
					sample_count_high = sample_count;
                    
                    // flag as busy
                    bus_is_busy = true;
				}

				sample_count_backup = sample_count;
				sample_count = 1;
			}
		}

		// There is no change to the bus state
		else
		{
			// Increment the sample counts
			sample_count++;

			// Decide on data detection whenever
			// we have a constant stream of sample counts
			if(sample_count == SAMPLE_COUNT_SPIKE_LIMIT)
			{
#ifdef DEBUG_LOG_SAMPLE_COUNTS
				// Debug logs.
				// To distinguish between a high and a low count,
				// high sample counts are multiplied by -1
				// low sample counts stay positive.
				log_sample_counts_buffer[log_sample_counts_buffer_index] = high ? sample_count_low : sample_count_high * -1;
				log_sample_counts_buffer_index = (log_sample_counts_buffer_index + 1) % DEBUG_LOG_SAMPLE_COUNTS_BUFFER_SIZE;
				log_sample_counts_buffer[log_sample_counts_buffer_index] = -1234567;
#endif

				if(high)
				{
					// Detect data bits on a high bus
					if(sample_count_low != SAMPLE_COUNT_NOT_READY)
					{
                        if(startBitIsDetected(sample_count_low))
						{
                            ringbuf_putByte(&iebus_ringbuf, IEBUS_BIT_START);
							//iebus_state = IEBUS_STATE_IDLE;
#ifdef GPIO_DEBUG
							HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
#endif
							sample_count = SAMPLE_COUNT_SPIKE_LIMIT;
						}

						sample_count_low = SAMPLE_COUNT_NOT_READY;
					}
				}
			}
#if 1
			// The bus is idle
			// flag the bit analysis as invalid
			if(sample_count >= SAMPLE_COUNT_IDLE)
			{
				//analyzeBit(IEBUS_BIT_INVALID);
                bus_is_busy = false;
			}
#endif
		}
	}
	
	// Expect IDLE state
	// An IDLE bus is a HIGH bus
	else if(iebus_state == IEBUS_STATE_IDLE)
	{
		if(!high && sample_count > SAMPLE_COUNT_IDLE_MIN_LIMIT)
		{
			// transfer to the SYNC state
			sample_count = 1;
			iebus_state = IEBUS_STATE_SYNC;
#ifdef GPIO_DEBUG 
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
#endif
		}
		else
		{
			sample_count++;
          
			if(sample_count >= SAMPLE_COUNT_IDLE_MAX_LIMIT)
			{
				iebus_state = IEBUS_STATE_START;

				high = false;
				last_high = false;
				sample_count = 0;
				sample_count_backup = 0;
#ifdef GPIO_DEBUG
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
#endif
			}
		}
	}
	
	// Expect SYNC state
	else if(iebus_state == IEBUS_STATE_SYNC)
	{
		// Judge our samples
		if(sample_count >= SAMPLE_COUNT_SYNC_MIN_LIMIT)
		{
#ifdef GPIO_DEBUG
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
#endif
			iebus_state = IEBUS_STATE_DATA;
			// Note that we're starting at sample counts ZERO in data land
			sample_count = 0;
		}
		else
		{
			sample_count++;
		}
	}
	
	// Expect DATA state
	else if(iebus_state == IEBUS_STATE_DATA)
	{
		static bool data_drive_lost = 0;

		if(high)
		{
			data_drive_lost = true;
		}
	
		sample_count++;
		
		if(sample_count >= SAMPLE_COUNT_DATA_LIMIT)
		{
			iebus_state = IEBUS_STATE_STOP;
#ifdef GPIO_DEBUG
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
#endif

			sample_count = 1;
            int bit = data_drive_lost ? IEBUS_BIT_HIGH : IEBUS_BIT_LOW;
            
            ringbuf_putByte(&iebus_ringbuf, bit);  
            
			data_drive_lost = false;
			
		}
	}
		
	// Expect STOP
	else if(iebus_state == IEBUS_STATE_STOP)
	{
		if(high)
		{
			iebus_state = IEBUS_STATE_IDLE;
			sample_count = 1;
#ifdef GPIO_DEBUG
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);
#endif
		}
	}
	last_high = high;

    if((iebus_state == IEBUS_STATE_START) && high)
    {
        return LOW_PRIO_INCREMENT;
    }
    return HIGH_PRIO_INCREMENT;
}

static inline uint8_t pushAdcDiff(int adc_diff)
{
#ifdef DEBUG_LOG_ADC_DIFF
  log_adc_diff_buffer[log_adc_diff_buffer_index] = adc_diff;
  log_adc_diff_buffer_index = (log_adc_diff_buffer_index + 1) % DEBUG_LOG_ADC_DIFF_BUFFER_SIZE;
  log_adc_diff_buffer[log_adc_diff_buffer_index] = -12345;
#endif
#define is_high(x) ((x) > 0x100)
    return pushBits(is_high(adc_diff));
}

static inline void analyzeBit(int bit)
{
  static const IEBUS_FRAME_FIELD *current_field;
  static uint8_t remaining_field_bits;

  // Data bit mask for the current field
  static uint16_t field_bit_mask;

  // Data buffer for the current field
  static uint16_t field_data_buffer;

  // Frame high bit count for parity check
  static volatile uint16_t frame_high_bit_count;

  // Number of bytes remaining for the current frame
  static int bytes_remaining;
#ifdef IGNORE_BROADCAST
  static bool frame_is_broadcast;
#endif
  static uint8_t *data;
  static bool is_processing = false;
#ifdef DEBUG_LOG_BITS
  log_bits_buffer[log_bits_buffer_index] = bit;
  log_bits_buffer_index = (log_bits_buffer_index + 1) % DEBUG_LOG_BITS_BUFFER_SIZE;
  log_bits_buffer[log_bits_buffer_index] = -12345;
#endif


  if(bit == IEBUS_BIT_INVALID)
  {
    is_processing = false;
    ++iebus_status.reader.invalid_bit;
    return;
  }

  do
  {
    // Bit detected is a start bit
    if(bit == IEBUS_BIT_START)
    {
      if(is_processing)
      {
        // Currently processing the previous frame,
        // and a start bit was suddenly received.
        // Mark the frame as incomplete
        ++iebus_status.incomplete;
#ifdef TMPFRAME_DEBUG
        if(iebusFrameReceivedHandler)
        {

            memset(&tmpframe, 0, sizeof(IEBUS_FRAME));

            tmpframe.valid = 1;
            tmpframe.slave_address[0] = 0x08;                          // 0x800 (MAIN CONTROLLER)
            tmpframe.slave_address[1] = 0x00;                          // 0x800 (MAIN CONTROLLER)
            tmpframe.master_address[0] = 0x08;                         // 0x880 (BUTTON)
            tmpframe.master_address[1] = 0x80 + 1 - 1;                 // 0x880 (BUTTON)
            tmpframe.data[0] = 0x94;                                   // 0x94 = IEBUS_DATA_BUTTON_PRESSED
            tmpframe.control = 0xe;                                    // 0xe = IEBUS_CONTROL_COMMAND_WRITE
            tmpframe.data_length = 1;
            tmpframe.not_broadcast = 0;
            iebusFrameReceivedHandler(&tmpframe);
        }
#endif
      }

      // Prepare the buffer
      data = &read_iebus_frame.not_broadcast;

      current_field = &(iebusFrameField[0]);
      remaining_field_bits = current_field->total_bits;
      field_bit_mask = current_field->bit_mask;
      field_data_buffer = 0;

      // Start processing
      is_processing = true;
    }

    // Bits should be either a 1 or a 0
    else if(is_processing)
    {
      if(bit == IEBUS_BIT_HIGH)
      {
        // Track the number of high bits for parity checking later
        ++frame_high_bit_count;

        // Save the bit to the field buffer.
        field_data_buffer |= field_bit_mask;
      }

      field_bit_mask >>= 1;

      // Last bit of field
      if(--remaining_field_bits == 0)
      {
        if(current_field->has_ack)
        {
          if(bit == IEBUS_BIT_HIGH)
          {
            --frame_high_bit_count;

            // Ignore frames that are not acknowledged by slave during ordinary communication
#ifdef IGNORE_BROADCAST
            if(frame_is_broadcast)
            {
              is_processing = false;

              ++iebus_status.ignored;
              continue;
            }
#endif
          }
        }

        if(current_field->has_parity)
        {
          if(frame_high_bit_count & 0x1)
          {
            // The data is invalid if high bit count is odd
            is_processing = false;

            ++iebus_status.parity_errors[current_field->id];

            continue;
          }
        }

        // Save the data field
        if(current_field->data_bits > 8)
        {
          // MSB
          *data = (uint8_t)(field_data_buffer >> 8);
          ++data;
        }

        // LSB
        *data = (uint8_t)field_data_buffer;
        ++data;

        if(current_field->id == IEBUS_FRAME_FIELD_DATA)
        {
          if(--bytes_remaining == 0)
          {
            // IE Bus is active

            // Do we need to set activity flag?
            // The activity flag is supposed to make the beacon detect when
            // it is clear to send some data

            read_iebus_frame.valid = true;

            if(iebusFrameReceivedHandler)
            {
                iebusFrameReceivedHandler(&read_iebus_frame);
            }

            is_processing = false;

            // A valid frame is detected
            ++iebus_status.valid;

            continue;
          }
        }
        else
        {
          //
          if(current_field->id == IEBUS_FRAME_FIELD_BROADCAST_BIT)
          {
#ifdef IGNORE_BROADCAST
            frame_is_broadcast = (bit == IEBUS_BIT_HIGH);
#endif
            frame_high_bit_count = 0;
          }

          //
          else if(current_field->id == IEBUS_FRAME_FIELD_MESSAGE_LENGTH)
          {
            // 0 means maximum
            if(field_data_buffer == 0)
            {
              field_data_buffer = IEBUS_FRAME_MESSAGE_LENGTH_FIELD_MAXIMUM_VALUE;
            }

            // Ignore multi-frame data for now;
            // Should be okay since all relevant data are single frame
            if(field_data_buffer > IEBUS_FRAME_DATA_FIELD_MAXIMUM_LENGTH)
            {
              is_processing = false;

              // Frame is too long
              ++iebus_status.too_long;
              continue;
            }

            bytes_remaining = field_data_buffer;
          }
          else if(current_field->id == IEBUS_FRAME_FIELD_SLAVE_ADDRESS)
          {
              if(!isFrameImportant(&read_iebus_frame))
              {
                  is_processing = false;
                  
                  // Frame is not important
                  ++iebus_status.not_important;
                  continue;
              }
          }

          ++current_field;
        }

        frame_high_bit_count = 0;

        remaining_field_bits = current_field->total_bits;
        field_bit_mask = current_field->bit_mask;
        field_data_buffer = 0;
      }
    }
  } while(0);
}

static void adcDiffHandler(adc_buffer_t *data, int datasize)
{
#ifdef ENABLE_DIFFERENTIAL
    for(int i = 0; i < datasize; i++)
    {
        pushAdcDiff(get_adc_difference(data[i]));
    }
#else
#if 1
    int i = 0;
    while(i < datasize)
    {
        //i += pushBits(get_adc_difference(data[i+1], data[i]));
        i += pushAdcDiff((data[i]));
    }
#else
    for(int i = 0; i < datasize; i+=4)
    {
        pushBits(get_adc_difference(data[i+1], data[i]));
    }
#endif
#endif
}

static void adcIsStopped(void)
{
    return;
}
#if 0
#if PWM_PULSE_SIZE == 3
// start bit: 180usec low, 26usec high
#define CREATE_START_BIT(pulse) \
{\
    (pulse)->pulseCount[0] = pwm_usecToPulseCount(180 + 26); \
    (pulse)->pulseCount[1] = 0; \
    (pulse)->pulseCount[2] = pwm_usecToPulseCount(180); \
}


// bit1: 20usec low, 20usec high
#define CREATE_BIT1(pulse) \
{\
    (pulse)->pulseCount[0] = pwm_usecToPulseCount(20 + 20); \
    (pulse)->pulseCount[1] = 0; \
    (pulse)->pulseCount[2] = pwm_usecToPulseCount(20); \
}

// bit1: 32usec low, 8usec high
#define CREATE_BIT0(pulse) \
{\
    (pulse)->pulseCount[0] = pwm_usecToPulseCount(32 + 8); \
    (pulse)->pulseCount[1] = 0; \
    (pulse)->pulseCount[2] = pwm_usecToPulseCount(32); \
}

#define CREATE_FRAME_END(pulse) \
{\
    (pulse)->pulseCount[0] = 0; \
    (pulse)->pulseCount[1] = 0; \
    (pulse)->pulseCount[2] = 0; \
}
#else 
// start bit: 180usec low, 21usec high
#define CREATE_START_BIT(pulse) \
{\
    (pulse)->pulseCount[0] = pwm_usecToPulseCount(180 + 21); \
    (pulse)->pulseCount[1] = 0; \
    (pulse)->pulseCount[2] = pwm_usecToPulseCount(180); \
    (pulse)->pulseCount[3] = pwm_usecToPulseCount(180); \
    (pulse)->pulseCount[4] = pwm_usecToPulseCount(180); \
    (pulse)->pulseCount[5] = pwm_usecToPulseCount(180); \
}


// bit1: 20usec low, 20usec high
#define CREATE_BIT1(pulse) \
{\
    (pulse)->pulseCount[0] = pwm_usecToPulseCount(20 + 20); \
    (pulse)->pulseCount[1] = 0; \
    (pulse)->pulseCount[2] = pwm_usecToPulseCount(20); \
    (pulse)->pulseCount[3] = pwm_usecToPulseCount(20); \
    (pulse)->pulseCount[4] = pwm_usecToPulseCount(20); \
    (pulse)->pulseCount[5] = pwm_usecToPulseCount(20); \
}

// bit1: 32usec low, 8usec high
#define CREATE_BIT0(pulse) \
{\
    (pulse)->pulseCount[0] = pwm_usecToPulseCount(32 + 8); \
    (pulse)->pulseCount[1] = 0; \
    (pulse)->pulseCount[2] = pwm_usecToPulseCount(32); \
    (pulse)->pulseCount[3] = pwm_usecToPulseCount(32); \
    (pulse)->pulseCount[4] = pwm_usecToPulseCount(32); \
    (pulse)->pulseCount[5] = pwm_usecToPulseCount(32); \
}

#define CREATE_FRAME_END(pulse) \
{\
    (pulse)->pulseCount[0] = 0; \
    (pulse)->pulseCount[1] = 0; \
    (pulse)->pulseCount[2] = 0; \
    (pulse)->pulseCount[3] = 0; \
    (pulse)->pulseCount[4] = 0; \
    (pulse)->pulseCount[5] = 0; \
}
#endif

// Convert bits to pulses.
// Maximum data is 12 bits, so we're containing it in a 16bit data
#define WITH_PARITY true
#define WITHOUT_PARITY false
#define WITH_ACKNOWLEDGEMENT true
#define WITHOUT_ACKNOWLEDGEMENT false
static inline int convertBitsToPulses(PWM_PULSE **pulse_addr, uint16_t data,
    int number_of_bits, bool has_parity, bool has_acknowledgement)
{
    int length = 0;
    int high_bits = 0;
    uint16_t bit_mask = 0x1 << (number_of_bits - 1);
    while(bit_mask != 0)
    {
        if(data & bit_mask)
        {
            CREATE_BIT1(*pulse_addr);
            high_bits++;
        }
        else
        {
            CREATE_BIT0(*pulse_addr);
        }

        // Move
        bit_mask >>= 1;
        *pulse_addr = *pulse_addr + 1;
        length++;
    }

    if(has_parity)
    {
        if(high_bits & 1)
        {
            CREATE_BIT1(*pulse_addr);
        }
        else
        {
            CREATE_BIT0(*pulse_addr);
        }
        *pulse_addr = *pulse_addr + 1;
        length++;
    }

    if(has_acknowledgement)
    {
        CREATE_BIT1(*pulse_addr);
        *pulse_addr = *pulse_addr + 1;
        length++;
    }
    return length;
}

static inline int convertStartBitToPulses(PWM_PULSE **pulse_addr)
{
    CREATE_START_BIT(*pulse_addr);
    *pulse_addr = *pulse_addr + 1;
    return 1;
}

static inline int convertSilenceToPulses(PWM_PULSE **pulse_addr)
{
    CREATE_FRAME_END(*pulse_addr);
    *pulse_addr = *pulse_addr + 1;
    return 1;
}

extern int iebus_sendFrameCount[2];

#define addressTo16Bit(addr) (((uint16_t)(((uint8_t *)(addr))[0]) << 8 | (uint16_t)((uint8_t *)(addr))[1]))
#define getPulseLength(frame) (45 + (10 * ((frame)->data_length)))
static void sendFrame(IEBUS_FRAME *frame)
{
    if(!frame->valid)
    {
        return;
    }
    
#if 0
    // Due to buffer limits, we'll be disabling this
    if(frame->data_length > IEBUS_FRAME_DATA_FIELD_MAXIMUM_LENGTH)
#else
    if(frame->data_length > 16)
#endif
    {
        return;
    }
    
    iebus_sendFrameCount[0]++;
    // Convert frame to pulses.
    PWM_PULSE *pulse = (PWM_PULSE *)iebus_pulsemem;

    int pulse_length = 0;
    pulse_length += convertStartBitToPulses(&pulse); // start bit

    pulse_length += convertBitsToPulses(&pulse, frame->not_broadcast ? 1 : 0, 1, WITHOUT_PARITY, WITHOUT_ACKNOWLEDGEMENT); // broadcast bit
    pulse_length += convertBitsToPulses(&pulse, addressTo16Bit(frame->master_address), 12, WITH_PARITY, WITHOUT_ACKNOWLEDGEMENT); // master address
    pulse_length += convertBitsToPulses(&pulse, addressTo16Bit(frame->slave_address), 12, WITH_PARITY, WITH_ACKNOWLEDGEMENT); // slave address
    pulse_length += convertBitsToPulses(&pulse, frame->control, 4, WITH_PARITY, WITH_ACKNOWLEDGEMENT); // control
    pulse_length += convertBitsToPulses(&pulse, frame->data_length, 8, WITH_PARITY, WITH_ACKNOWLEDGEMENT); // data length
#if 1
    for(int i = 0; i < frame->data_length; i++)
    {
        pulse_length += convertBitsToPulses(&pulse, frame->data[i], 8, WITH_PARITY, WITH_ACKNOWLEDGEMENT); // data
    }
#endif

    pulse_length += convertSilenceToPulses(&pulse); // silence
    
    iebus_sendFrameCount[1]++;
    // Start and wait for TIM2
    
    //while busy
    
    while(bus_is_busy)
    {
        HAL_Delay(1);
    }
    
    pwm_sendPulse((PWM_PULSE *)iebus_pulsemem, pulse_length);
    
    return;
}
#endif
// Public API

void iebus_initialize(
        IEBUS_FRAME_RECEIVED_HANDLER received_frame_handler,
        adc_buffer_t *buffer,
        uint32_t bufferSize)
{
    ringbuf_initialize(&iebus_ringbuf, iebus_read_buffer, IEBUS_READ_BUFFER_SIZE);
    iebusFrameReceivedHandler = received_frame_handler;
    iebusAdcBuffer = buffer;
    iebusAdcBufferSize = bufferSize;
}

void iebus_start(void)
{
    adc_initialize(iebusAdcBuffer, iebusAdcBufferSize, adcDiffHandler, adcIsStopped);
    adc_start();
}

void iebus_stop(void)
{
    adc_stop();
}

void iebus_service(void)
{
    do
    {
        //pwm_check();

        uint8_t bit;
        if(ringbuf_getByte(&iebus_ringbuf, &bit))
        {
#ifdef DEBUG_WITH_DPIN
            dpin_set(IEBUS_READING);
#endif
            analyzeBit(bit);
#ifdef DEBUG_WITH_DPIN
            dpin_reset(IEBUS_READING);
#endif
        }
    } while(0);
}
static IEBUS_FRAME temp_frame;
bool iebus_sendFrame(IEBUS_FRAME *frame)
{
#if 0
    if(pwm_isBusy())
    {
        return false;
    }
    if(iebus_is_busy)
    {
        //return false;
    }
#ifdef DEBUG_WITH_DPIN
    dpin_set(IEBUS_SENDING);
#endif
    //iebus_stop();
    memcpy(&temp_frame, frame, sizeof(IEBUS_FRAME));
    sendFrame(&temp_frame);
    //iebus_start();
#ifdef DEBUG_WITH_DPIN
    dpin_reset(IEBUS_SENDING);
#endif
#endif
    return true;
}

#define WITHIN_RANGE(u, l, h) (((u) > (l)) && (u) < (h))

#define is_us_start_bit(x) WITHIN_RANGE(x, 1700, 1810)
#define is_us_data(x) WITHIN_RANGE(x, 100, 350)

void iebus_pushUs(uint32_t us)
{
	static enum {
		IEBUS_STATE_START,
		IEBUS_STATE_DATA0,
		IEBUS_STATE_DATA1
	} iebus_state = IEBUS_STATE_START;
    
    static uint32_t data_pulses[2];
    
    if(iebus_state == IEBUS_STATE_START)
    {
        if(is_us_start_bit(us))
        {
            analyzeBit(IEBUS_BIT_START);
            iebus_state = IEBUS_STATE_DATA0; // SYNC comes first
        }
        else
        {
            // mark ignored
        }
        return;
    }
    
    if(iebus_state == IEBUS_STATE_DATA0)
    {
        if(!is_us_data(us))
        {
            iebus_state = IEBUS_STATE_START;
            return;
        }
        data_pulses[0] = us;
        iebus_state = IEBUS_STATE_DATA1;
        return;
    }
    
    if(iebus_state == IEBUS_STATE_DATA0)
    {
        data_pulses[1] = us;
        
        // analyze data
        
        // case 1
        if(WITHIN_RANGE(data_pulses[1], 190, 250))
        {
            analyzeBit(IEBUS_BIT_HIGH);
        }
        
        // case 2
        else if(WITHIN_RANGE(data_pulses[1], 330, 350))
        {
            analyzeBit(IEBUS_BIT_LOW);
        }
        
        iebus_state = IEBUS_STATE_DATA0;
        return;
    }
    
}
