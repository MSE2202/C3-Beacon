/*
 *  MSE 2202 ESP32-C3 Beacon Code
 *
 *  Language: Arduino (C++)
 *  Target:   ESP32-C3
 *  Author:   Michael Naish
 *  Date:     2024 10 23
 *
 *  This program is used by the beacon to transmit the ASCII character "U" at 2400 baud (this can be 
 *  changed in cTXData). At 2400 baud, each bit is ~417 usec.
 *
 *  Output at IR LED of ASCII byte is:
 *  if byte bit is a 1 (high) LED is off
 *  if byte bit is a 0 (low) LED is pulsed at ~38 kHz
 *  Output flow: 1 start bit (low), 8 character bits, 1 stop bit. Repeat.
 *
 *  Beacon will transmit cNumBytes characters, then pause until the start of the next burst.
 *  The timing is set by cBurstSpacing.
 *
 *  The low bits pulsing at ~38 kHz can be detected by a TSOP IR receiver and drive its output low
 *  If the output of a TSOP IR receiver is connected to the Rx pin on the MSEduino and the MSEduino 
 *  is onfigured to read this serial port at 2400 baud, the MSEduino will read a "U" (0x55), or 
 *  whatever ASCII character is defined.
 *
 *  TSOP 32338 IR Receiver pin 1 is output, 2 is Vcc, 3 is Gnd; pin one is on left side when looking 
 *  at front of part.
 *
 *  While running, the built-in LED (GPIO8) will toggle on and off with each transmission burst.
 * 
 *  If a limit switch connected to GPIO9 is pressed, the beacon will stop tramsitting for ~30 s and 
 *  the built-in LED will stop blinking.
 *
 *  The supply voltage is monitored on GPIO1 (which must be connected to 3V3 pin). If the voltage 
 *  drops below ~3V, the ESP32 will will enter deep sleep to protect the battery. The battery will 
 *  have to be recharged and reattached to reenable the beacon.
 *
 *  Required settings for USB serial output to function properly (In Tools menu)
 *   USB CDC on Boot: Enabled
 *   JTAG Adapter: Disabled
 *
 */

#include <Arduino.h>
#include <esp_task_wdt.h>
#include "hal/wdt_types.h"
#include "hal/wdt_hal.h"

// Constants
const uint8_t cVCCPin = 1;                            // GPIO used to monitor input voltage
const uint8_t cCarrierPin = 5;                        // GPIO for 38 kHz carrier
const uint8_t cTXPin = 6;                             // GPIO for TX1
const uint8_t cRXPin = 7;                             // GPIO for RX1 (assigned, but unused)
const uint8_t cLEDPin = 8;                            // built-in LED (ON on LOW)
const uint8_t cLimitSwitchPin = 9;                    // GPIO for limit switch input (active LOW)
const uint8_t cLimitSwitchGndPin = 10;                // GPIO used as ground for limit switch
const uint8_t cPWMResolution = 10;                    // bit resolution for carrier PWM (1-14 bits)
const uint8_t cTXData = 0x55;                         // byte to be sent: U = 0b01010101
const uint16_t cNumBytes = 1;                         // number of bytes to transmit in a burst
const uint16_t cVCCThreshold = 3700;                  // ~3.0 VDC
const uint32_t cBurstSpacing = 100;                   // milliseconds between bursts
const uint32_t cOffTime = 30000;                      // duration of transmission deactivation, in milliseconds
const uint32_t cPWMFrequency = 38000;                 // carrier frequency in Hz
const float cDutyCycle = 0.5;                         // duty cycle of carrier (0-1)

// Variables
uint32_t nextBurst;                                   // time of next tranmission burst
uint32_t restartTime;                                 // time to restart transmission after deactivation
bool runState = true;                                 // 0 = stopped; 1 = running

void setup() {
  pinMode(cVCCPin, INPUT);                            // Configure voltage monitoring pin as input
  pinMode(cLimitSwitchPin, INPUT_PULLUP);             // Configure limit switch input
  pinMode(cLimitSwitchGndPin, OUTPUT);                // Configure limit switch ground pin
  digitalWrite(cLimitSwitchGndPin, LOW);              // Set limit switch ground reference
  pinMode(cLEDPin, OUTPUT);                           // Configure built-in LED pin as output
  pinMode(cTXPin, OUTPUT);                            // Configure TX pin as output

  // disable watchdog timer
  wdt_hal_context_t rtc_wdt_ctx = RWDT_HAL_CONTEXT_DEFAULT();
  wdt_hal_write_protect_disable(&rtc_wdt_ctx);
  wdt_hal_disable(&rtc_wdt_ctx);
  wdt_hal_write_protect_enable(&rtc_wdt_ctx);
  esp_task_wdt_deinit();

  // configure LEDC to generate carrier signal with specified frequency, resolution and duty cycle
  ledcAttach(cCarrierPin, cPWMFrequency, cPWMResolution);
  ledcWrite(cCarrierPin, (1 << cPWMResolution) * cDutyCycle); // 2^cPWMResolution * cDutyCycle

  // configure UART1 with desired parameters: 2400 baud rate, 8 data bits, no parity, 1 stop bit
  Serial1.begin(2400, SERIAL_8N1, cRXPin, cTXPin);
  nextBurst = millis();                               // initialize burst timing
}

void loop() {
  uint32_t curTime = millis();                        // get current time
  
  // disable transmission when limit switch is engaged
  if (!digitalRead(cLimitSwitchPin)) {                // if the limit switch is LOW, switch is engaged
    runState = false;                                 // deactivate transmission
    restartTime = curTime + cOffTime;                 // set time for transmission reactivation
    digitalWrite(cLEDPin, HIGH);                      // turn off built-in LED
  }

  // transmit character(s) periodically
  if (curTime - nextBurst < cBurstSpacing) {          // wait number of milliseconds to next burst
    if (runState) {                                   // if transmission is active
      for (uint16_t i = 0; i < cNumBytes; i++) {           
        Serial1.write(cTXData);                       // load UART1 TX FIFO buffer with data
      }
      digitalWrite(cLEDPin, !digitalRead(cLEDPin));   // toggle built-in LED
    }
    nextBurst += cBurstSpacing;                       // update time for next burst
    uint16_t vcc = analogRead(cVCCPin);               // read input voltage
    if (vcc <= cVCCThreshold) {                       // check whether battery voltage is below threshold; if so
      ledcDetach(cCarrierPin);                        // stop carrier signal      
      esp_deep_sleep_start();                         // and put device to sleep -> unplug and recharge
    }
  }

  // if transmission has been deactivated long enough
  if (!runState && (curTime - restartTime < cOffTime)) {
    runState = true;                                  // reactivate transmission
  }
}