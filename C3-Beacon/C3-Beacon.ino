/*
  MSE 2202 ESP-C3 Beacon Code

  Language: Arduino (C++)
  Target:   ESP32-C3
  Author:   Michael Naish
  Date:     2024 10 21

  This program is used by the beacon to transmit the ASCII character "U" (this can be changed at cTXData) at 2400 baud
 
  Output at IR LED of ASCII byte is:
  if byte bit is a 1 (high) LED is off
  if byte bit is a 0 (low) LED is pulsed at ~38 kHz

  The low bits pulsing at ~38 kHz can be detected by a TSOP IR receiver and drive its output low
  If the output of a TSOP IR receiver is connected to the Rx pin on the MSEduino and the MSEduino 
  configured to read this serial port at 2400 baud, the MSEduino will read a "U" (0x55), or whatever ASCII character is defined

  At 2400 baud, each bit is ~417 usec
  Output flow: 1 start bit (low), 8 character bits, 1 stop bit. Repeat 

  Beacon will transmit cNumBytes characters, then pause until the start of the next burst. The timing is set by cBurstSpacing
  
  TSOP 32338 IR Receiver pin 1 is output, 2 is Vcc, 3 is Gnd; pin one is on left side when looking at front of part 

  While running, the built-in LED (GPIO8) will toggle on and off with each transmission burst
 
  If a limit switch connected to GPIO9 is pressed, the beacon will stop tramsitting for ~30 s and the built-in 
  LED will stop blinking 

  The supply voltage is monitored on GPIO1 (must be connected to 3V3). If the voltage drops below ~3V, the ESP32 will
  will enter deep sleep to protect the battery. The battery will have to be recharged and reattached to reenable the beacon.

  Required settings for USB serial output to function properly (In Tools menu)
    USB CDC on Boot: Enabled
    JTAG Adapter: Integrated USB JTAG

*/

#include <Arduino.h>
#include <esp_task_wdt.h>
#include "hal/wdt_types.h"
#include "hal/wdt_hal.h"

void ARDUINO_ISR_ATTR timerISR();

// Constants
const int cVCCPin = 1;                                // GPIO used to monitor input voltage
const int cCarrierPin = 5;                            // GPIO for 38 kHz carrier
const int cTXPin = 6;                                 // GPIO for TX1
const int cRXPin = 7;                                 // GPIO for RX1 (assigned, but unused)
const int cLEDPin = 8;                                // built-in LED (ON on LOW)
const int cLimitSwitchPin = 9;                        // GPIO for limit switch input (active LOW)
const int cLimitSwitchGndPin = 10;                    // GPIO used as ground for limit switch
const int cFreq = 38000;                              // carrier frequency in Hz
const int cNumBytes = 1;                              // number of bytes to transmit in a burst
const int cBurstSpacing = 100;                        // milliseconds between bursts
const int cOffTime = 30000;                           // duration of transmission deactivation, in milliseconds
const int cVCCThreshold = 3700;                       // ~3.0 VDC
const char cTXData = 0x55;                            // U = 0b01010101

// Variables
uint32_t nextBurst;                                   // time of next tranmission burst
uint32_t restartTime;                                 // time to restart transmission after deactivation
hw_timer_t * pTimer = NULL;                           // pointer to timer used by timer interrupt
boolean runState = true;                              // 0 = stopped; 1 = running
boolean carrierState = true;                          // 0 = LOW; 1 = HIGH

void setup() {
  pinMode(cVCCPin, INPUT);                            // Configure voltage monitoring pin as input
  pinMode(cLimitSwitchPin, INPUT_PULLUP);             // Configure limit switch input
  pinMode(cLimitSwitchGndPin, OUTPUT);                // Configure limit switch ground pin
  digitalWrite(cLimitSwitchGndPin, LOW);              // Set limit switch ground reference
  pinMode(cLEDPin, OUTPUT);                           // Configure built-in LED pin as output
  pinMode(cCarrierPin, OUTPUT);                       // Configure carrier pin as output
  pinMode(cTXPin, OUTPUT);                            // Configure TX pin as output

  // disable watchdog timer
  wdt_hal_context_t rtc_wdt_ctx = RWDT_HAL_CONTEXT_DEFAULT();
  wdt_hal_write_protect_disable(&rtc_wdt_ctx);
  wdt_hal_disable(&rtc_wdt_ctx);
  wdt_hal_write_protect_enable(&rtc_wdt_ctx);
  esp_task_wdt_deinit();

  // configure hardware timer to generate carrier signal
  pTimer = timerBegin(1000000);                       // start timer with 1 MHz frequency
  timerAttachInterrupt(pTimer, &timerISR);            // configure timer ISR
  uint32_t cycleTime = 1000000 / cFreq;               // calculate number of microseconds per period
  // calcuate number of microseconds for half period
  uint32_t halfPeriod = (uint32_t)(0.5 * cycleTime) % cycleTime;
  timerAlarm(pTimer, halfPeriod, true, 0);            // interrupt every half period, set to repeat indefinitely
 
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
      for (int i = 0; i < cNumBytes; i++) {           
        Serial1.write(cTXData);                       // load UART1 TX FIFO buffer with data
      }
      digitalWrite(cLEDPin, !digitalRead(cLEDPin));   // toggle built-in LED
    }
    nextBurst += cBurstSpacing;                       // update time for next burst
    uint16_t vcc = analogRead(cVCCPin);               // read input voltage
    if (vcc <= cVCCThreshold) {                       // check whether battery voltage is below threshold; if so
      timerStop(pTimer);                              // stop timer for carrier signal
      esp_deep_sleep_start();                         // and put device to sleep -> unplug and recharge
    }
  }

  // if transmission has been deactivated long enough
  if (!runState && (curTime - restartTime < cOffTime)) {
    runState = true;                                  // reactivate transmission
  }
}

// timer interrupt service routine
// generates a high frequency carrier signal by toggling state of carrier pin
void ARDUINO_ISR_ATTR timerISR() {
  digitalWrite(cCarrierPin, carrierState);          // set state of carrier pin
  carrierState = !carrierState;                     // toggle carrierState for next cycle
}