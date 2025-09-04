#include "main.h"

#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/power.h>

#include <util/delay.h>

#include "aled.h"       // Include several of loopj's useful utility libraries
#include "i2c_target.h" // Some of these have been partially modified to suit my needs
#include "button.h"
#include "console.h"
#include "rtc.h"
#include "gpio.h"

#include "i2c_bitbang.h"

#include "bq25895.h"

#define BAUD_RATE 115200

#define STORM_I2C 0x50

#define ADDR_VER 0x00
#define ADDR_CHRGCURRENT 0x01
#define ADDR_PRECURRENT 0x03
#define ADDR_TERMCURRENT 0x05
#define ADDR_CHRGVOLTAGE 0x07
#define ADDR_FANSPEED 0x09

const uint8_t ver = 0x02; // v0.2 (ver / 10)

//BBI2C bbi2c;
// TODO: re-implement second I2C 

bool isPowered = false; // Is the Wii powered?
bool isCharging = false; // Is the BQ charging the batteries?

//bool buttonTriggered = false; // Has the button been triggered?
bool triggeredSoftShutdown = false; // Did we trigger the soft shutdown?

bool isOverTemp = false;

//Much of this is specific to the BQ chip I'm using, see the datasheet for more info.
//const uint8_t bqAddr = 0x6A;

uint8_t battCharge = 0x00; // 0x00-0xFF, representing 0-100% charge
uint16_t battVolt = 3700;
const uint16_t minBattVolt  = 2700;
const uint16_t maxInCurrent = 3250;
const uint16_t battVoltLevels[5] = {4200, 3700, 3000, 2800, 2700};
const uint16_t battChrgLevels[9] = {2684, 2864, 3064, 3264, 3444, 3644, 3824, 4024, 4204};
bq25895_fault_t pwrErrorStatus = 0x00;
bq25895_charge_state_t chargeStatus = 0x00;

uint16_t chrgCurrent = 4096; // 4096mA
uint16_t preCurrent = 128; // 128mA
uint16_t termCurrent = 256; // 256mA
uint16_t chrgVoltage = 4208; // 4.208V
uint8_t fanSpeed = 0xFF; // 100% speed

const bool ilimEnabled = false;

void getEEPROM() {
  if (eeprom_read_byte(ADDR_VER) == 0) { // Check if there's no data in the EEPROM
    writeToEEPROM(); // Leave at defaults, write to EEPROM
  }
  else {
    chrgCurrent = eeprom_read_word(ADDR_CHRGCURRENT);
    preCurrent = eeprom_read_word(ADDR_PRECURRENT);
    termCurrent = eeprom_read_word(ADDR_TERMCURRENT);
    chrgVoltage = eeprom_read_word(ADDR_CHRGVOLTAGE);
    fanSpeed = eeprom_read_byte(ADDR_FANSPEED);
  }
  eeprom_write_byte(ADDR_VER, ver);
}

bool i2c_bitbang_write(uint16_t addr, uint8_t reg, void const* buf, size_t len, void* context) {
    // TODO: write `len` bytes of `buf` to device with i2c addr `addr` to device register `reg` and return boolean result
    struct i2c_msg msg;
    msg.buf = buf;
    msg.len = len;
    if (i2c_transfer(addr, &msg, 1) == 0) {
      return true;
    }
    return false;
}

bool i2c_bitbang_read(uint16_t addr, uint8_t reg, void const* buf, size_t len, void* context) {
    // TODO: read `len` bytes into `buf` from device with i2c addr `addr` from device register `reg` and return boolean result
    if (i2c_read(addr, buf, len) == 0) {
      return true;
    }
    return false;
}

bq25895_t bq = {
    .write = i2c_bitbang_write,
    .read = i2c_bitbang_read,
};

int handle_register_read(uint8_t reg_addr, uint8_t *value) {
  switch (reg_addr) {
    case 0x00: // Get version
      *value = ver;
    break;

    case 0x02:
      applyChanges(); // Apply and store current settings
    break;

    case 0x04:
      *value = fanSpeed;
    break;

    case 0x0B:
      enableShipping(); // Don't need to bother doing anything, we'll be losing power soon anyway
      _delay_ms(1000);
    break;

    case 0x10:
      *value = chrgCurrent;
    break;

    case 0x11:
      *value = termCurrent;
    break;

    case 0x12:
      *value = preCurrent;
    break;

    case 0x13:
      *value = chrgVoltage;
    break;

    case 0x15:
      chargingStatus();
      *value = chargeStatus;
    break;

    case 0x24:
      battChargeStatus();
      *value = battCharge;
    break;

    case 0x26:
      getBattVoltage();
      *value = battVolt;
    break;
  }
  return 0;
}

int handle_register_write(uint8_t reg_addr, uint8_t value) {
  switch (reg_addr) {
    case 0x02:
      applyChanges(); // Apply and store current settings
    break;

    case 0x04:
      fanSpeed = value;
    break;

    case 0x0B:
      enableShipping(); // Don't need to bother doing anything, we'll be losing power soon anyway
      _delay_ms(1000);
    break;

    case 0x10:
      chrgCurrent = value;
    break;

    case 0x11:
      termCurrent = value;
    break;

    case 0x12:
      preCurrent = value;
    break;

    case 0x13:
      chrgVoltage = value;
    break;
  }
  setupBQ();
  return 0;
}

bool setup() {
  button_init(&pwr_button, &BUTTON.port, BUTTON.num, NULL, buttonHold);
  rtc_init();
  console_init(BAUD_RATE);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Set sleep to low power mode
  sleep_enable(); // Enable sleeping, don't activate sleep yet though

  // Set unused pins to outputs
  PORTA.DIRSET |= (1 << 3) + (1 << 4) + (1 << 6);
  PORTB.DIRSET |= (1 << 5);
  PORTC.DIRSET |= (1 << 1);

  getEEPROM(); // Get settings from EEPROM

  gpio_input(BUTTON);
  gpio_config(BUTTON, PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc);

  gpio_input(CHRG_STAT);
  gpio_config(CHRG_STAT, PORT_ISC_BOTHEDGES_gc);

  gpio_input(TEMP_ALERT);
  gpio_config(TEMP_ALERT, PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc);

  gpio_input(SOFT_SHUT);
  gpio_config(SOFT_SHUT, PORT_ISC_RISING_gc);

  gpio_set_low(SOFT_PWR);
  gpio_output(SOFT_PWR);

  // TODO: Fan PWM 
  gpio_output(FAN);
  //gpio_config(FAN, 0);
  gpio_set_low(FAN);

  gpio_set_high(PWR_ON); // Makes sure console is off
  gpio_output(PWR_ON);
  
  setFan(false, fanSpeed); // Turn off fan

  i2c_target_init(STORM_I2C, handle_register_read, handle_register_write);

  led_init();

  i2c_configure(I2C_MODE_FAST);
  if (!bq25895_is_present(&bq)) {
    return false;
  }
  setupBQ();
  return true;
}

void loop() {
  button_update(&pwr_button, rtc_millis());
  if (isPowered) {
    monitorBatt();
    _delay_ms(500);
  }
  else if (isCharging) {
    chargingStatus();
    _delay_ms(500);
  }
  else if (gpio_read(BUTTON) != false) {
    rtc_deinit();
    sleep_cpu(); // The console isn't on, nor is it charging. Enter sleep to save power.
  }
}

void overTemp() {
  triggerShutdown(); // Trigger shutdown process
  //analogWrite(FAN, 0xff); // fan at max speed, keep console cool
  powerLED(5);
  isOverTemp = true;
  for (int i = 0; i < 120; i++) { // 2 min to cool down
    _delay_ms(1000);
  }
  isOverTemp = false;
  setFan(false, fanSpeed); // disable cooling fan
}


void buttonHold() {
  if (isPowered) {
    triggerShutdown(); // Is it on? Turn it off.
  }
  else {
    getBattVoltage();
    if (((battVolt > minBattVolt) || isCharging) && !isOverTemp && (pwrErrorStatus == 0x00)) { 
    // Check that either the battery is charged enough, or console is charging, 
    // AND make sure there's no over-temp issues
    // AND make sure there's no power errors
      consoleOn(); // Battery is charged enough or currently charging, turn on console
    }
    else {
      powerLED(5); // Flash red light, battery too low OR over temp OR misc power error
    }
  }
  while (pwr_button.button_state != BUTTON_RELEASED) {
    button_update(&pwr_button, rtc_millis());
  }
}

void triggerShutdown() {
  if (!triggeredSoftShutdown && isPowered) {
    gpio_set_high(SOFT_PWR); // Trigger the soft shutdown sequence on the Wii
    _delay_ms(30);
    gpio_set_low(SOFT_PWR);
    triggeredSoftShutdown = true;
    _delay_ms(2500);
    if (isPowered && triggeredSoftShutdown) { // Wii did not shut itself down safely, timeout and force a power-off
      consoleOff();
      triggeredSoftShutdown = false;
    }
  }
  else if (!isPowered) {
    consoleOff(); // If some bug happens, and the console is on while this thinks it isn't, might as well make sure it's shut down.
  }
}

void softShutdown() {
  if (gpio_read(SOFT_SHUT) == true) { // Check that the pin is actually high
    _delay_ms(100);
    consoleOff();
    triggeredSoftShutdown = false;
  }
}

void chargingStatus() {
  bq25895_is_charger_connected(&bq, &isCharging);
  bq25895_get_charge_state(&bq, &chargeStatus);
  bq25895_check_faults(&bq, &pwrErrorStatus);
  if (pwrErrorStatus != 0x00) { // Uh oh, *something* is wrong
    triggerShutdown();
    if (pwrErrorStatus & BQ_FAULT_THERM) { // oh jeez stuff is hot this is really bad
      overTemp();
      return;
    }
    else if (pwrErrorStatus & BQ_FAULT_BAT) { // ???? the battery is TOO charged????
      enableShipping();
      return; // Unnecessary since the console's gonna power off anyway
    }
  }

  if (chargeStatus == BQ_STATE_NOT_CHARGING) {
    isCharging = false;
    if (isPowered) {
      monitorBatt();
    }
    else {
      powerLED(0); // Not on, not charging
    }
  }
  else {
    isCharging = true;
    if (chargeStatus == BQ_STATE_TERMINATED) { // Finished charging
      powerLED(7);
    }
    else {
      powerLED(6); // Charging, not complete
    }
  }
  _delay_ms(100);
}

void initFan() {
  // TODO: PWM stuff
}

void setFan(bool active, uint8_t speed) {
  if (active) {
    //analogWrite(FAN, speed);
  }
  else {
    gpio_set_low(FAN);
  }
}


void powerLED(uint8_t mode) {
  /*
  0 = All LEDs off
  1 = Full charge -- Green
  2 = Medium charge -- Yellow
  3 = Low charge -- Orange
  4 = About to run out -- Red
  5 = Low-power shutdown, or other error -- Couple red flashes
  6 = Charging -- Soft blue
  7 = Charging, full -- Soft pink
  */
  switch (mode) {
    case 0:
      setLED(0x00, 0x00, 0x00, 0.0, false);
    break;

    case 1:
      setLED(0x00, 0xFF, 0x00, 0.5, true); // Green
    break;

    case 2:
      setLED(0xFF, 0xFF, 0x00, 0.5, true); // Yellow
    break;

    case 3:
      setLED(0xFF, 0x80, 0x00, 0.5, true); // Orange
    break;

    case 4:
      setLED(0xFF, 0x00, 0x00, 0.5, true); // Red
    break;

    case 5:
      for (int i = 0; i < 5; i++) { // Flash red
        setLED(0xFF, 0x00, 0x00, 0.5, true);
        _delay_ms(100);
        setLED(0x00, 0x00, 0x00, 0.0, false);
        _delay_ms(100);
      }
      powerLED(0);
    break;

    case 6:
      setLED(0x00, 0x00, 0xFF, 0.25, true); // Dim blue
    break;

    case 7:
      setLED(0xFF, 0xB7, 0xC5, 0.25, true); // Dim sakura pink
    break;

    default:
      powerLED(0); // off
    break;
  }
}

void consoleOn() {
  bq25895_set_adc_cont(&bq, true);

  gpio_set_low(PWR_ON); // Activate MOSFET
  isPowered = true;

  setFan(true, fanSpeed); // Enable fan

  monitorBatt();
}

void consoleOff() {
  bq25895_set_adc_cont(&bq, false);

  gpio_set_high(PWR_ON); // Deactivate MOSFET
  isPowered = false;

  setFan(false, fanSpeed);

  monitorBatt();
}

void enableShipping() {
  bq25895_set_batfet_enabled(&bq, false);
}

void setupBQ() {
  bq25895_set_iin_max(&bq, maxInCurrent);
  bq25895_set_vsys_min(&bq, 3500);
  bq25895_set_charge_config(&bq, BQ_CHG_CONFIG_ENABLE);
  bq25895_set_charge_current(&bq, chrgCurrent);
  bq25895_set_term_current(&bq, termCurrent);
  bq25895_set_precharge_current(&bq, preCurrent);
  bq25895_set_max_charge_voltage(&bq, chrgVoltage);
  bq25895_set_recharge_offset(&bq, BQ_VRECHG_100MV);
  bq25895_set_batlow_voltage(&bq, BQ_VBATLOW_3000MV);
  bq25895_set_charge_termination(&bq, true);
  bq25895_set_max_temp(&bq, BQ_MAX_TEMP_100C);
  bq25895_set_adc_cont(&bq, isPowered);
}

void writeToEEPROM() {
  eeprom_write_word(ADDR_CHRGCURRENT, chrgCurrent);
  eeprom_write_word(ADDR_PRECURRENT, preCurrent);
  eeprom_write_word(ADDR_TERMCURRENT, termCurrent);
  eeprom_write_word(ADDR_CHRGVOLTAGE, chrgVoltage);
  eeprom_write_byte(ADDR_FANSPEED, fanSpeed);
}

void applyChanges() {
  writeToEEPROM();
  setFan(isPowered, fanSpeed);
  setupBQ();
}


void getBattVoltage() {
  if (!isPowered) {
    bq25895_trigger_adc_read(&bq);
    _delay_ms(200);
  }
  bq25895_get_adc_batt(&bq, &battVolt);
}


void setLED(uint8_t r, uint8_t g, uint8_t b, float bright, bool enabled) {
  if (!enabled) {
    led_clear_all();
  }
  else {
    uint32_t color = 0x000000;
    r = (uint8_t)(r * bright);
    g = (uint8_t)(g * bright);
    b = (uint8_t)(b * bright);
    color = (g << 16) + (r << 8) + b; // WS2812 and compatible use GRB
    led_set_all(color);
  }
  led_refresh();
}

void battChargeStatus() {
  chargingStatus();
  if (chargeStatus == 0b11) { // Fully-charged
    battCharge = 0xff;
    return;
  }
  getBattVoltage();
  _delay_ms(100);
  
  float tmp;

  for (int i = 0; i < 8; i++) {
    if ((battVolt >= battChrgLevels[i]) && (battVolt < battChrgLevels[i+1])) {
      tmp = i + ((battVolt - battChrgLevels[i]) / (battChrgLevels[i+1] - battChrgLevels[i]));
      tmp *= 0x20;
      battCharge = (int)tmp;
      return;
    }
  }
}

void monitorBatt() {
  getBattVoltage();
  _delay_ms(100);
  if (isCharging || !isPowered) { 
    // If not turned on, or if charging, let the chargingStatus function handle it
    powerLED(0);
    chargingStatus();
    return;
  }
  if (battVolt < minBattVolt) {
    triggerShutdown(); // Battery too low, emergency shutdown
    powerLED(5); // Show flashing red for error
    return;
  }
  if (battVolt > battVoltLevels[1]) {
    powerLED(1); // High charge
  }
  else if (battVolt > battVoltLevels[2]) {
    powerLED(2); // Medium charge
  }
  else if (battVolt > battVoltLevels[3]) {
    powerLED(3); // Low charge
  }
  else if (battVolt <= battVoltLevels[3]) {
    powerLED(4); // ABOUT TO RUN OUT
  }
}

int main() {
  if (!setup()) {
    return 0;
  }
  while (1) {
    loop();
  }
  return 1;
}

ISR(PORTA_PORT_vect) {
  rtc_init();
  button_update(&pwr_button, rtc_millis());
  if (gpio_read_intflag(TEMP_ALERT)) {
    overTemp();
  }
  PORTA.INTFLAGS = 0xFF;
}

ISR(PORTB_PORT_vect) {
  if (gpio_read_intflag(SOFT_SHUT)) {
    softShutdown();
  }
  PORTB.INTFLAGS = 0xFF;
}

ISR(PORTC_PORT_vect) {
  if (gpio_read_intflag(CHRG_STAT)) {
    chargingStatus();
  }
  PORTC.INTFLAGS = 0xFF;
}