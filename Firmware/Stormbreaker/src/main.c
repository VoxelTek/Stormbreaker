#include "main.h"

#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/power.h>

#include <util/delay.h>

#include "led.h"
#include "i2c_target.h"

#include "gpio.h"

#define STORM_I2C 0x50

#define ADDR_VER 0x00
#define ADDR_CHRGCURRENT 0x01
#define ADDR_PRECURRENT 0x02
#define ADDR_TERMCURRENT 0x03
#define ADDR_CHRGVOLTAGE 0x04
#define ADDR_FANSPEED 0x05

const uint8_t ver = 0x01; // v0.1 (ver / 10)

//BBI2C bbi2c;

bool isPowered = false; // Is the Wii powered?
bool isCharging = false; // Is the BQ charging the batteries?

bool buttonTriggered = false; // Has the button been triggered?
bool triggeredSoftShutdown = false; // Did we trigger the soft shutdown?

bool isOverTemp = false;

//Much of this is specific to the BQ chip I'm using, see the datasheet for more info.
const uint8_t bqAddr = 0x6A;

uint8_t battCharge;
uint8_t battVolt = 0b1000101; //~3.7V
const uint8_t minBattVolt = 0b0010011; //~2.7V
uint8_t battVoltLevels[5] = {0b01011111, 0b1000101, 0b0100010, 0b0011000, 0b0010011}; // 4.2, 3.7, 3.0, 2.8, 2.7
uint8_t battChrgLevels[9] = {0x13, 0x1c, 0x26, 0x30, 0x39, 0x43, 0x4c, 0x56, 0x5f};
uint8_t pwrErrorStatus = 0x00;
uint8_t chargeStatus = 0b00;
uint8_t maxCurrent = 0x3F; // 3.25A

uint8_t chrgCurrent;
uint8_t preCurrent;
uint8_t termCurrent;
uint8_t chrgVoltage;
uint8_t fanSpeed;

const bool ilimEnabled = false;

//tinyNeoPixel pixels = tinyNeoPixel(NUMPIXELS, PIN_PA5, NEO_GRB);

/*
int I2CWriteRegister(BBI2C *pI2C, unsigned char iAddr, unsigned char reg, unsigned char value) {
    unsigned char buffer[2];
    buffer[0] = reg;   // Register address
    buffer[1] = value; // Data to write

    return I2CWrite(pI2C, iAddr, buffer, 2);
}
*/

void getEEPROM() {
  if (eeprom_read_byte(ADDR_VER) == 0) { // Check if there's no data in the EEPROM
    chrgCurrent = 0b1000010; // 4224mA
    preCurrent = 0b0001;
    termCurrent = 0b0011;
    chrgVoltage = 0b010111;
    fanSpeed = 0xFF;
    writeToEEPROM(); 
  }
  else {
    chrgCurrent = eeprom_read_byte(ADDR_CHRGCURRENT);
    preCurrent = eeprom_read_byte(ADDR_PRECURRENT);
    termCurrent = eeprom_read_byte(ADDR_TERMCURRENT);
    chrgVoltage = eeprom_read_byte(ADDR_CHRGVOLTAGE);
    fanSpeed = eeprom_read_byte(ADDR_FANSPEED);
  }
  eeprom_write_byte(ADDR_VER, ver);
}

int handle_register_read(uint8_t reg_addr, uint8_t *value) {
  switch (reg_addr) {
    case 0x00: // Get version
      *value = ver;
    break;

    case 0x02:
      applyChanges(); // Apply and store current settings
      return;
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
      battChargeStatus();
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
}

int handle_register_write(uint8_t reg_addr, uint8_t value) {
  switch (reg_addr) {
    case 0x02:
      applyChanges(); // Apply and store current settings
      return;
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
}

void setup() {
  //power_adc_disable(); // We don't need the ATtiny ADC, turn it off
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Set sleep to low power mode
  sleep_enable(); // Enable sleeping, don't activate sleep yet though

  // Set unused pins to outputs
  /*
  pinMode(PIN_PA3, OUTPUT);
  pinMode(PIN_PA4, OUTPUT);
  pinMode(PIN_PA6, OUTPUT);
  pinMode(PIN_PB5, OUTPUT);
  pinMode(PIN_PC1, OUTPUT);
  */

  getEEPROM(); // Get settings from EEPROM

  gpio_input(BUTTON);
  gpio_config(BUTTON, PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc);

  gpio_input(CHRG_STAT);
  gpio_config(CHRG_STAT, PORT_ISC_BOTHEDGES_gc);

  gpio_input(TEMP_ALERT);
  gpio_config(TEMP_ALERT, PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc);

  gpio_input(SOFT_SHUT);
  gpio_config(SOFT_SHUT, PORT_ISC_RISING_gc);

  gpio_output(SOFT_PWR);
  gpio_set_low(SOFT_PWR);

  // TODO: Get fan PWM 
  gpio_output(FAN);
  //gpio_config(FAN, 0);
  gpio_set_low(FAN);


  gpio_output(PWR_ON);
  gpio_set_high(PWR_ON); // Makes sure console is off
  
  setFan(false); // Turn off fan

  /*
  memset(&bbi2c, 0, sizeof(bbi2c));
  bbi2c.bWire = false;
  bbi2c.iSDA = SDA2;
  bbi2c.iSCL = SCL2;
  I2CInit(&bbi2c, 0xFFFF);
  */

  i2c_target_init(STORM_I2C, handle_register_read, handle_register_write);

  //pixels.begin();

  setupBQ();
}

void loop() {
  if (isPowered) {
    monitorBatt();
    _delay_ms(500);
  }
  else if (isCharging) {
    chargingStatus();
    _delay_ms(500);
  }
  else {
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
  setFan(false); // disable cooling fan
}

void powerButton() { // Power button has been pressed
  if (!buttonTriggered && gpio_read(BUTTON) == false) {
    buttonTriggered = true; // Make sure other instances of this function don't run. They shouldn't, hopefully, but eh.
    _delay_ms(1000); // Wait in order to prevent bouncing and accidental presses
    if (gpio_read(BUTTON) == false) {
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
          powerLED(5); // Flash red light, battery too low OR over temp
        }
      }
    }
    while (gpio_read(BUTTON) == false) { // Wait for button to stop being pressed
      _delay_ms(100);
    }
    _delay_ms(250); // Debounce protection...kinda
    buttonTriggered = false;
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
  unsigned char chrgStat = 0x00;
  //I2CReadRegister(&bbi2c, bqAddr, 0x0B, &chrgStat, 0x8);
  chargeStatus = ((chrgStat & 0b00011000) >> 3);

  unsigned char errorStat = 0x00;
  //I2CReadRegister(&bbi2c, bqAddr, 0x0C, &errorStat, 0x8);
  pwrErrorStatus = errorStat;
  if (pwrErrorStatus != 0x00) { // Uh oh, *something* is wrong
    triggerShutdown();
    if (pwrErrorStatus & (1 << 5)) { // oh jeez stuff is hot this is really bad
      overTemp();
      return;
    }
    else if (pwrErrorStatus & (1 << 3)) { // ???? the battery is TOO charged????
      enableShipping();
    }
  }

  if (chargeStatus == 0b00) {
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
    if (chargeStatus == 0b11) { // Finished charging
      powerLED(7);
    }
    else {
      powerLED(6); // Charging, not complete
    }
  }
  _delay_ms(100);
}

void setFan(bool active) {
  if (active) {
    //analogWrite(FAN, fanSpeed);
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
  /*REG02*/
  unsigned char reg02 = (0b11111100); // Disable D+/D- detection and such
  // Continuous battery voltage reading if console is on
  //I2CWriteRegister(&bbi2c, bqAddr, 0x02, reg02);

  gpio_set_low(PWR_ON); // Activate MOSFET
  isPowered = true;

  setFan(true); // Enable fan

  monitorBatt();
}

void consoleOff() {
  /*REG02*/
  unsigned char reg02 = (0b00111100); // Disable D+/D- detection and such
  // Disable continuous battery voltage reading
  //I2CWriteRegister(&bbi2c, bqAddr, 0x02, reg02);

  gpio_set_high(PWR_ON); // Deactivate MOSFET
  isPowered = false;

  setFan(false);

  monitorBatt();
}

void enableShipping() {
  /*REG09*/
  unsigned char reg09 = (1 << 5); // Enable shipping mode
  //I2CWriteRegister(&bbi2c, bqAddr, 0x09, reg09);
}

void setupBQ() {
  /*REG00*/
  unsigned char reg00 = ((maxCurrent) | (ilimEnabled << 6)); // Set max current and ILIM.
  //I2CWriteRegister(&bbi2c, bqAddr, 0x00, reg00);

  /*REG02*/
  unsigned char reg02 = (0b00111100); // Disable D+/D- detection and such
  if (isPowered){
    reg02 |= 0b11000000; // Continuous battery voltage reading if console is on
  }
  //I2CWriteRegister(&bbi2c, bqAddr, 0x02, reg02);

  /*REG03*/
  unsigned char reg03 = (0b00011010); // Disable OTG
  //I2CWriteRegister(&bbi2c, bqAddr, 0x03, reg03);

  /*REG04*/
  unsigned char reg04 = (0x00 | chrgCurrent); // Set fast charge current
  //I2CWriteRegister(&bbi2c, bqAddr, 0x04, reg04);

  /*REG05*/
  unsigned char reg05 = ((preCurrent << 4) | (0b1111 & termCurrent)); // Set term charge current
  //I2CWriteRegister(&bbi2c, bqAddr, 0x05, reg05);

  /*REG06*/
  unsigned char reg06 = (0b00000011 | (chrgVoltage << 2)); // Set max battery voltage
  //I2CWriteRegister(&bbi2c, bqAddr, 0x06, reg06);
}

void writeToEEPROM() {
  eeprom_write_byte(ADDR_CHRGCURRENT, chrgCurrent);
  eeprom_write_byte(ADDR_PRECURRENT, preCurrent);
  eeprom_write_byte(ADDR_TERMCURRENT, termCurrent);
  eeprom_write_byte(ADDR_CHRGVOLTAGE, chrgVoltage);
  eeprom_write_byte(ADDR_FANSPEED, fanSpeed);
}

void applyChanges() {
  writeToEEPROM();
  setFan(isPowered);
  setupBQ();
}


void getBattVoltage() {
  if (!isPowered) {
    unsigned char reg02 = (0b10111100); // Disable D+/D- detection and such
    // Get single voltage reading
    //I2CWriteRegister(&bbi2c, bqAddr, 0x02, reg02);
    _delay_ms(200);
  }
  unsigned char adcRegStatus = 0;
  //I2CReadRegister(&bbi2c, bqAddr, 0x0E, &adcRegStatus, 0x8);
  adcRegStatus &= 0b1111111;
  battVolt = adcRegStatus;
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
    color = (g << 16) + (r << 8) + b;
    led_set_all(color);
  }
}

void battChargeStatus() {
  if (chargeStatus == 0b11) {
    battCharge = 0xff;
    return;
  }
  getBattVoltage();
  _delay_ms(100);
  
  float tmp;

  for (int i = 0; i < 8; i++) {
    if ((battVolt >= battChrgLevels[i]) && (battVolt < battChrgLevels[i+1])) {
      tmp = i + ((battVolt - battChrgLevels[i])/ (battChrgLevels[i+1] - battChrgLevels[i]));
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
  if ((battVolt <= battVoltLevels[0]) && (battVolt > battVoltLevels[1])) {
    powerLED(1); // High charge
  }
  else if ((battVolt <= battVoltLevels[1]) && (battVolt > battVoltLevels[2])) {
    powerLED(2); // Medium charge
  }
  else if ((battVolt <= battVoltLevels[2]) && (battVolt > battVoltLevels[3])) {
    powerLED(3); // Low charge
  }
  else if ((battVolt <= battVoltLevels[3]) && (battVolt > battVoltLevels[4])) {
    powerLED(4); // ABOUT TO RUN OUT
  }
}

int main() {
  setup();
  while (1) {
    loop();
  }
  return 1;
}

ISR(PORTA_PORT_vect) {
  if (gpio_read(BUTTON) == false) {
    powerButton();
  }
  if (gpio_read(TEMP_ALERT) == false) {
    overTemp();
  }
}

ISR(PORTB_PORT_vect) {
  if (gpio_read(SOFT_SHUT) == true) {
    softShutdown();
  }
}

ISR(PORTC_PORT_vect) {
  if (gpio_read(CHRG_STAT) == true) {
    chargingStatus();
  }
}