/*
 *  MSE 2202 ESP32-C3 Beacon Code
 *
 *  Language: Arduino (C++)
 *  Target:   ESP32-C3
 *  Author:   Michael Naish
 *  Date:     2025 01 29
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
 *  is configured to read this serial port at 2400 baud, the MSEduino will read a "U" (0x55), or 
 *  whatever ASCII character is defined.
 *
 *  TSOP 32338 IR Receiver pin 1 is output, 2 is Vcc, 3 is Gnd; pin one is on left side when looking 
 *  at front of part.
 *
 *  While running, the built-in LED (GPIO8) will toggle on and off with each transmission burst.
 * 
 *  If a source of magnetic flux (i.e., a magnet) is not near the Hall effect sensor, the beacon
 *  will stop transmitting and the built-in LED will stop blinking. 
 *  
 *  If necessary, power consumption can be increased by enabling the onboard WiFi peripheral and
 *  periodically scanning for networks. This may be required to keep some rechargable batteries
 *  from turning off due to insufficient load.
 *   
 *  Required settings for USB serial output to function properly (In Tools menu)
 *   USB CDC on Boot: Enabled
 *
 */

#define INCREASE_POWER_USE                            // Enable if necessary to keep battery on
#define SELF_TEST                                     // Enable to test with connected IR detector

#include <Arduino.h>
#ifdef INCREASE_POWER_USE
#include "WiFi.h"
#endif

// Constants
const uint8_t cHallPin = 4;                           // GPIO for Hall effect sensor
const uint8_t cCarrierPin = 5;                        // GPIO for 38 kHz carrier
const uint8_t cTXPin = 6;                             // GPIO for TX1
const uint8_t cRXPin = 21;                            // GPIO for RX1 (for testing with IR receiver only)
const uint8_t cLEDPin = 8;                            // built-in LED (ON on LOW)
const uint8_t cPWMResolution = 10;                    // bit resolution for carrier PWM (1-14 bits)
const uint8_t cTXData = 0x55;                         // byte to be sent: U = 0b01010101
const uint16_t cMinADC = 500;                         // minimum acceptable ADC value (removes ADC artifacts)
const uint16_t cLowerHallLimit = 2100;                // lowest acceptable raw ADC value from Hall sensor
const uint16_t cUpperHallLimit = 2700;                // highest acceptable raw ADC value from Hall sensor
const uint16_t cMaxMissingHallCounts = 5;             // allowable out of range Hall sensor measurements
const uint16_t cNumBytes = 1;                         // number of bytes to transmit in a burst
const uint32_t cBurstInterval = 100;                  // milliseconds between bursts
const uint32_t cPWMFrequency = 38000;                 // carrier frequency in Hz
const float cDutyCycle = 0.5;                         // duty cycle of carrier (0-1)

#ifdef INCREASE_POWER_USE
const uint16_t cScanInterval = 3000;                  // WiFi network scan interval in milliseconds
#endif
#ifdef SELF_TEST
const uint8_t cIRGndPin = 10;                         // GPIO used for IR receiver ground (testing only)
const uint8_t cIRVCCPin = 20;                         // GPIO used for IR receiver supply (testing only)
#endif

// Variables
uint32_t nextBurst;                                   // time of next tranmission burst
uint16_t hallVal;                                     // raw ADC value from Hall effect sensor
uint16_t hallMissingCount = 0;                        // number of 
bool runState = true;                                 // 0 = stopped; 1 = running
#ifdef INCREASE_POWER_USE
uint32_t nextScan;                                    // time of next scan for WiFi networks
bool scanState = false;                               // 0  = clear network list; 1 = scan for networks
#endif
#ifdef SELF_TEST
uint8_t receivedData = 0;                             // data received from IR receiver (byte)
#endif

void setup() {
  pinMode(cHallPin, INPUT);                           // configure Hall monitoring pin as input
  pinMode(cLEDPin, OUTPUT);                           // configure built-in LED pin as output
  pinMode(cTXPin, OUTPUT);                            // configure TX pin as output

#ifdef INCREASE_POWER_USE
  WiFi.mode(WIFI_STA);                                // initialize WiFi in station mode
  WiFi.disconnect();                                  // disconnect from all access points
  delay(100);                                         // ensure WiFi peripheral has started
#endif
#ifdef SELF_TEST
  Serial.begin(9600);                                 // enable standard serial to output received bytes
  pinMode(cIRGndPin, OUTPUT);                         // configure IR receiver ground pin as output
  digitalWrite(cIRGndPin, LOW);                       // set low (ground)
  pinMode(cIRVCCPin, OUTPUT);                         // configure IR receiver supply pin as output
  digitalWrite(cIRVCCPin, HIGH);                      // set high (VCC)
#endif

  // configure LEDC to generate carrier signal with specified frequency, resolution, and duty cycle
  ledcAttach(cCarrierPin, cPWMFrequency, cPWMResolution);
  ledcWrite(cCarrierPin, (1 << cPWMResolution) * cDutyCycle); // 2^cPWMResolution * cDutyCycle

  // configure UART2 with desired parameters: 2400 baud rate, 8 data bits, no parity, 1 stop bit
  Serial1.begin(2400, SERIAL_8N1, cRXPin, cTXPin);
  nextBurst = millis();                               // initialize burst timing
}

void loop() {
  uint32_t curTime = millis();                        // get current time

  // update Hall effect sensor reading and transmit character(s) periodically
  if (curTime - nextBurst < cBurstInterval) {         // wait number of milliseconds to next burst
    hallVal = analogRead(cHallPin);                   // read Hall effect sensor
    // if Hall effect sensor measurement is within allowable limits (i.e., magnet is detected)
    if (((hallVal >= cMinADC) && (hallVal <= cLowerHallLimit)) || (hallVal >= cUpperHallLimit)) {
      if (!runState) {                                // if transmission is disabled
        runState = true;                              // enable transmission
        hallMissingCount = 0;                         // reset missing Hall measurement count
      }
    }
    else {
      // if too many Hall effect sensor readings are out of range (i.e., magnet is not detected)
      if (hallMissingCount++ > cMaxMissingHallCounts) {
        runState = false;                             // disable transmission
        digitalWrite(cLEDPin, HIGH);                  // turn off built-in LED
      }
    }
    if (runState) {                                   // if transmission is active
      for (uint16_t i = 0; i < cNumBytes; i++) {           
        Serial1.write(cTXData);                       // load UART1 TX FIFO buffer with data
      }
      digitalWrite(cLEDPin, !digitalRead(cLEDPin));   // toggle built-in LED
    }
    nextBurst += cBurstInterval;                      // update time for next burst
  }

#ifdef SELF_TEST
  while (Serial1.available() > 0) {                   // if received data in buffer
    receivedData = Serial1.read();                    // read a byte (character)
    Serial.printf("%c\n", receivedData);              // output to serial
  }
#endif
#ifdef INCREASE_POWER_USE  
  if ((curTime - nextScan) < cScanInterval) {         // wait for next scan
    nextScan += cScanInterval;                        // update time for next scan
    // alternate between scanning for WiFi networks and deleting list
    if (!scanState) {                                  
      WiFi.scanNetworks(true);                        // scan for WiFi networks
      scanState = true;
    }
    else {
      WiFi.scanDelete();                              // delete scan result to free memory
      scanState = false;
    }
  }
#endif
}