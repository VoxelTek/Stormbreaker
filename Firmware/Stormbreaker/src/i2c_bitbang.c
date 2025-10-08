/*
 * I2C bit-banging core.
 */
#include "i2c_bitbang.h"
#include "gpio.h"

#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay.h>

// GPIO pins
static const gpio_t PIN_SDA = {&PORTC, 2};
static const gpio_t PIN_SCL = {&PORTC, 3};

// Drive modes for SCL line
enum { I2C_DRIVE_PUSH_PULL, I2C_DRIVE_OPEN_DRAIN };

// Is the I2C bus configured yet?
static bool configured = false;

// Configured I2C timings (in ticks)
#define delay 1250 // Half SCL period
#define half_delay 625 // Quarter SCL period

// The current drive mode of the SCL line
// Defaults to push/pull
static bool drive_mode = I2C_DRIVE_PUSH_PULL;

// Set the state of the SCL line
static inline void i2c_bb_set_scl(int state)
{
  if (state) {
    if (drive_mode == I2C_DRIVE_OPEN_DRAIN) {
      // Set as input, allow pull-up resistor to pull the line high
      gpio_config(PIN_SCL, PORT_PULLUPEN_bm);
      gpio_input(PIN_SCL);

      // Wait for the target to release the SCL line to support clock stretching
      // TODO: Add a timeout
      while (!(gpio_read(PIN_SCL)));
    } else {
      // Set as output, and pull the line high
      gpio_set_high(PIN_SCL);
      gpio_output(PIN_SCL);
    }
  } else {
    gpio_set_low(PIN_SCL);
    gpio_output(PIN_SCL);
  }
}

// Set the state of the SDA line
static inline void i2c_bb_set_sda(int state)
{
  if (state) {
    // Set SDA as input, allow pull-up resistor to pull the line high
    gpio_config(PIN_SDA, PORT_PULLUPEN_bm);
    gpio_input(PIN_SDA);
  } else {
    // Set SDA low, and as an output
    gpio_set_low(PIN_SDA);
    gpio_output(PIN_SDA);
  }
}

// Get the current state of the SDA line
static inline bool i2c_bb_get_sda()
{
  // Set SDA as input and return the state
  gpio_input(PIN_SDA);
  return gpio_read(PIN_SDA);
}

// Delay for a number of ticks
#define i2c_bb_delay(us) _delay_us(us)

/*
static inline void i2c_delay(double microseconds)
{
  _delay_us(microseconds);
}
*/

// I2C start condition
static inline void i2c_bb_start()
{
  i2c_bb_set_sda(0);
  i2c_bb_delay(half_delay);

  i2c_bb_set_scl(0);
  i2c_bb_delay(half_delay);
}

// I2C repeated start condition
static inline void i2c_bb_repeated_start()
{
  i2c_bb_set_sda(1);
  i2c_bb_delay(half_delay);

  i2c_bb_set_scl(1);
  i2c_bb_delay(half_delay);

  i2c_bb_start();
}

// I2C stop condition
static inline void i2c_bb_stop()
{
  i2c_bb_set_sda(0);
  i2c_bb_delay(half_delay);

  i2c_bb_set_scl(1);
  i2c_bb_delay(delay);

  i2c_bb_set_sda(1);
  i2c_bb_delay(delay);
}

// Write a single bit
static inline void i2c_bb_write_bit(int bit)
{
  i2c_bb_set_sda(bit);
  i2c_bb_delay(half_delay);

  i2c_bb_set_scl(1);
  i2c_bb_delay(delay);

  i2c_bb_set_scl(0);
  i2c_bb_delay(half_delay);
}

// Read a single bit
static inline int i2c_bb_read_bit()
{
  i2c_bb_set_sda(1);
  i2c_bb_delay(half_delay);

  i2c_bb_set_scl(1);
  i2c_bb_delay(delay);

  int bit = i2c_bb_get_sda();

  i2c_bb_set_scl(0);
  i2c_bb_delay(half_delay);

  return bit;
}

// Write a single byte
static inline int i2c_bb_write_byte(uint8_t data)
{
  for (uint8_t i = 0; i < 8; i++) {
    i2c_bb_write_bit(data & 0x80); // write the most-significant bit
    data <<= 1;
  }

  // Return inverted ACK bit - 'true' for ACK, 'false' for NACK
  return !i2c_bb_read_bit();
}

// Read a single byte
static inline uint8_t i2c_bb_read_byte()
{
  uint8_t data = 0;
  for (uint8_t i = 0; i < 8; i++) {
    data <<= 1;
    data |= i2c_bb_read_bit();
  }

  return data;
}

// Perform an I2C bit-banged transfer
static inline int i2c_bitbang_transfer(uint8_t addr, struct i2c_bb_msg *msgs, uint8_t num_msgs)
{
  // Always start with a start condition
  unsigned int flags = I2C_BB_MSG_RESTART;

  // Return early if there are no messages
  if (!num_msgs)
    return 0;

  do {
    // Send stop condition from previous message, if needed
    if (flags & I2C_BB_MSG_STOP) {
      i2c_bb_stop();
    }

    // Forget old flags, except for start
    flags &= I2C_BB_MSG_RESTART;

    // Send start or repeated start condition
    if (flags & I2C_BB_MSG_RESTART) {
      i2c_bb_start();
    } else if (msgs->flags & I2C_BB_MSG_RESTART) {
      i2c_bb_repeated_start();
    }

    // Get flags for new message
    flags |= msgs->flags;

    // Send address after start condition
    if (flags & I2C_BB_MSG_RESTART) {
      // Adjust address to include read/write bit
      uint8_t addr_rw = (addr << 1) | (flags & I2C_BB_MSG_READ);

      // Send address
      int ack = i2c_bb_write_byte(addr_rw);

      // Check for NACK
      if (!ack) {
        i2c_bb_stop();
        return -I2C_BB_ERR;
      }

      flags &= ~I2C_BB_MSG_RESTART;
    }

    // Transfer data
    uint8_t *buf     = msgs->buf;
    uint8_t *buf_end = buf + msgs->len;
    if (flags & I2C_BB_MSG_READ) {
      // Read
      while (buf < buf_end) {
        // Read byte
        *buf++ = i2c_bb_read_byte();

        // ACK the byte, except for the last one
        i2c_bb_write_bit(buf == buf_end);
      }
    } else {
      // Write
      while (buf < buf_end) {
        // Write byte
        int ack = i2c_bb_write_byte(*buf++);

        // Check for NACK
        if (!ack) {
          i2c_bb_stop();
          return -I2C_BB_ERR;
        }
      }
    }

    // Next message
    msgs++;
    num_msgs--;
  } while (num_msgs);

  // Send final stop condition
  i2c_bb_stop();
  return 0;
}

// Determine the drive mode of the SCL line
static bool i2c_bb_is_open_drain()
{
  // Store gpio directions
  uint8_t dir = PIN_SCL.port->DIR;

  // Set SCL as input, wait for the line to settle
  gpio_config(PIN_SCL, PORT_PULLUPEN_bm);
  gpio_input(PIN_SCL);
  i2c_bb_delay(half_delay);

  // Read the SCL line multiple times, if it is ever low, it is not open-drain
  bool open_drain = true;
  for (int i = 0; i < 100; i++) {
    if (!(gpio_read(PIN_SCL))) {
      open_drain = false;
      break;
    }

    i2c_bb_delay(half_delay);
  }

  // Restore original gpio directions
  PIN_SCL.port->DIR = dir;

  return open_drain;
}

int i2c_bb_configure(uint8_t mode)
{
  // Set the I2C timings
  /*
  switch (mode) {
    case I2C_MODE_STANDARD: // 100 KHz
      delay      = 5000;
      half_delay = 2500;
      break;

    case I2C_MODE_FAST: // 400 KHz
      delay      = 1250;
      half_delay = 625;
      break;

    default:
      return -I2C_BB_ERR;
  }
  */

  // Send a stop condition to ensure the SCL and SDA lines are high
  i2c_bb_stop();

  // Enable open-drain mode if supported
  if (i2c_bb_is_open_drain())
    drive_mode = I2C_DRIVE_OPEN_DRAIN;

  // Set the configured flag
  configured = true;

  return 0;
}

int i2c_bb_transfer(uint8_t addr, struct i2c_bb_msg *msgs, uint8_t num_msgs)
{
  // Check if the I2C bus is configured
  if (!configured)
    return -I2C_BB_ERR;

  // Disable interrupts
  //uint32_t level;
  //_CPU_ISR_Disable(level);

  // Perform the transfer
  int result;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    result = i2c_bitbang_transfer(addr, msgs, num_msgs);
  }

  // Re-enable interrupts
  //_CPU_ISR_Restore(level);

  return result;
}
