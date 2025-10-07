#include <avr/io.h>
#include <stddef.h>
#include "gpio.h"
#include "button.h"
#include "i2c_target.h"

static const gpio_t SDA2        = {&PORTC, 2};
static const gpio_t SCL2        = {&PORTC, 3};

static const gpio_t BQ_INT      = {&PORTC, 0};

static const gpio_t FAN         = {&PORTB, 2};

static const gpio_t LED         = {&PORTA, 5};

static const gpio_t PWR_ON      = {&PORTA, 1}; // Output
static const gpio_t BUTTON      = {&PORTA, 2};

struct button pwr_button;

static const gpio_t TEMP_ALERT  = {&PORTA, 7};

static const gpio_t SOFT_PWR    = {&PORTB, 3}; //Out to Wii
static const gpio_t SOFT_SHUT   = {&PORTB, 4}; //In from Wii

int main();

bool setup();
void loop();

//int I2CWriteRegister(BBI2C *pI2C, unsigned char iAddr, unsigned char reg, unsigned char value);
void getEEPROM();
void overTemp();
void buttonHold();
void triggerShutdown();
void softShutdown();
void chargingStatus();
void initFan();
void setFan(bool active, uint8_t speed);
void powerLED(uint8_t mode);
void consoleOn();
void consoleOff();
void enableShipping();
void setupBQ();
void writeToEEPROM();
void applyChanges();
void getBattVoltage();
int handle_register_read(uint8_t reg_addr, uint8_t *value);
int handle_register_write(uint8_t reg_addr, uint8_t value);
bool i2c_bitbang_write(uint16_t addr, uint8_t reg, void const* buf, size_t len, void* context);
bool i2c_bitbang_read(uint16_t addr, uint8_t reg, void const* buf, size_t len, void* context);
void setLED(uint8_t r, uint8_t g, uint8_t b, float bright, bool enabled);
void battChargeStatus();
void monitorBatt();
void setupHUSB(float voltage, float current);