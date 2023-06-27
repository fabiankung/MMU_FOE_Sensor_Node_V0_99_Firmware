// 1. Sample program to illustrate interfacing NRF24L01+ radio module with
// Arduino and sensors. Also sleep mode in the micro-controller is activated.
// Branch_wSleep_Binary
// 2. This version supports multiple sensor nodes to 1 receiver.
//    Up to 6 sensor nodes can communicate with 1 receiver using the same
//    frequency channel. Each sensor node is assigned a distint datapipe,
//    value from 0 to 5. We can imagine the datapipe as time-division 
//    multiple access.
// 3. The connected sensors are
//  a) A soil moisture sensor to pin A1 of Arduino.
//  b) An analog output sensor of choice connected to pin A2 of Arduino.
//  c) A humidity/temperature sensor, DHT11/22, connected to pin 4 of Arduino.
//  d) An internal 2:1 voltage divider with input connected to VBAT 
//     (positive terminal of BAT1) and output to pin A0 of Arduino. This is
//     mainly used to measure the connected battery voltage level if we are
//     using LiPo rechargable battery to power the sensor node.    
// 4. Transmitting text string to the receiver.
// 5. DC motor driver to pin 5 of Arduino.
//
// Author        : Fabian Kung
// Last modified : 27 June 2023
// Arduino Board : Pro-micro
// 
// Resources:
// For tutorial on nRF24L01+ module and the supporting Arduino
// library, please visit:
// https://lastminuteengineers.com/nrf24l01-arduino-wireless-communication/
// For DHT11, DHT22 sensor, please visit:
// https://learn.adafruit.com/dht/overview

//Include Libraries
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN      4   // Signal pin to DHT11 or DHT22 sensor module.

#define PMOTDRIVE   5
#define PDEBUG_LED  6
#define PBAT_STAT   7
#define PBAT_LEVEL  0    // AN0 - Connected to a 1:2 resistive voltage divider.

// Set Sensor Node ID and Address
//#define _SENSOR_ID       'A'
//#define _SENSOR_ID       'B'
#define _SENSOR_ID       'C'
// Set the address of the datapipe. 
#define   _DATAPIPE_NUMBER  1     // Valid value: 0 to 5

// --- Objects and variables for nRF24L01+ radio module ---
// Create an RF24 object
RF24 radio(9, 8);  // CE, CSN

// Address through which two modules communicate.
uint64_t address[6] = {0x7878787878LL,
                       0xB3B4B5B6F1LL,
                       0xB3B4B5B6CDLL,
                       0xB3B4B5B6A3LL,
                       0xB3B4B5B60FLL,
                       0xB3B4B5B605LL
                      };

char  strTX[16];          // TX string buffer  

// --- Objects and variables for DHT11, DHT22 sensor ---
// Uncomment the type of sensor in use:
//#define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;

typedef struct Struct4BCD
{
  byte  bytDigit;
  byte  bytTen;
  byte  bytHundred;
  byte  bytThousand;
} OBJ_4BCD;

OBJ_4BCD obj4BCDSensor;
OBJ_4BCD obj4BCDTemp;

void setup()
{
   
  pinMode(5, OUTPUT); // Set all unused digital pins to output, and set level to '0'.
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);

  pinMode(PDEBUG_LED, OUTPUT);  // Initialize LED driver pin.
  pinMode(PBAT_STAT, INPUT);    // Battery charger status monitoring pin.

  // --- nRF24L01+ radio module setup ---
  radio.begin();
  // Set the channel, carrier frequency = 2400 + [Channel No.] MHz
  radio.setChannel(100); // Use 2500 MHz, to avoid interference with WiFi 2.4GHz system.
  // Set data rate.
  radio.setDataRate(RF24_250KBPS); // Set to lower datarate, this is also most sensitive
                                   // at -94 dBm. Default is RF24_1MBPS, with lower
                                   // sensitivity.
                                   // If we use 2000 kbps baud rate, the channel interval 
                                   // should be 2 MHz to prevent interference.                                 
  
  
  // According to the datasheet, the auto-retry features's delay value should
  // be "skewed" to allow the RX node to receive 1 transmission at a time.
  // So, use varying delay between retry attempts and 15 (at most) retry attempts.

  #if  _DATAPIPE_NUMBER == 0
    radio.openWritingPipe(address[0]);
    radio.setRetries(0, 15);          // Delay 0 interval, max 15 retries.
  #elif _DATAPIPE_NUMBER == 1
    radio.openWritingPipe(address[1]);
    radio.setRetries(2, 15);          // Delay 2 intervals, max 15 retries.
  #elif _DATAPIPE_NUMBER == 2
    radio.openWritingPipe(address[2]);
    radio.setRetries(4, 15);          // Delay 4 intervals, max 15 retries.
  #elif _DATAPIPE_NUMBER == 3
    radio.openWritingPipe(address[3]);
    radio.setRetries(6, 15);          // Delay 6 intervals, max 15 retries.
  #elif _DATAPIPE_NUMBER == 4
    radio.openWritingPipe(address[4]);
    radio.setRetries(8, 15);          // Delay 8 intervals, max 15 retries.
  #else _DATAPIPE_NUMBER == 5
    radio.openWritingPipe(address[5]);
    radio.setRetries(10, 15);          // Delay 10 intervals, max 15 retries.
  #endif
  //Set module as transmitter
  radio.stopListening();

  // --- DHT11/22 sensor setup ---
  Serial.begin(9600);
  delay(1000);        // I discovered that if we immediately send to serial port
                      // the computer sometimes missed this.
  // Initialize device.
  dht.begin();
  /*
  Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;
  */
  obj4BCDSensor.bytThousand = 0;
  obj4BCDSensor.bytHundred = 0;
  obj4BCDSensor.bytTen = 0;
  obj4BCDSensor.bytDigit = 0;
  obj4BCDTemp.bytThousand = 0;
  obj4BCDTemp.bytHundred = 0;
  obj4BCDTemp.bytTen = 0;
  obj4BCDTemp.bytDigit = 0; 
  
  sleep_enable();                       // Enable sleep mode.
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Set sleep mode option.
                                        // Power down the CPU completely 
                                        // during sleep, with only the
                                        // 128 kHz internal RC oscillator running
                                        // in Atmega micro-controller.
  wdt_enable(WDTO_8S);                  // Enable WDT with roughly 8s timeout.  

}

// Sample application.
// Read Analog Channel 2, convert the value to BCD (binary coded decimal), and
// send the value to remote receiver. 
// Data format:
// [Data pipe number] + [Sensor ID] + [4 digits BCD value]
//
// The 4 digits BCD value corresponds to millivolts.
// Thus for instance if we get 1750, it means Analog Channel 2
// measures a voltage of 1.750 V or 1750 mV at the last sample.
// Example:
// 1C1750

void loop()
{

  // Send a message to the receiver.
  
  unsigned int unSensor_mV;
  unsigned int unInputVoltage_mV;
  unsigned int unSensorSoilMois_mV;
  
  sensors_event_t event;
    
  int nIndex;

  radio.powerUp();  // Powerup nRF24L01+ radio.
  delay(500);       // A delay for the radio to initialize.
  Serial.println("Sending Radio Packet");
  digitalWrite(PDEBUG_LED,HIGH);     // Turn on debug LED.

  // --- Read and convert analog sensor inputs to BCD ---
  unInputVoltage_mV = analogRead(0);      // Read input voltage VBAT.
  unSensorSoilMois_mV = analogRead(1);    // Read soil moisture sensor output.  
  unInputVoltage_mV = (unInputVoltage_mV*49)/10;
  unInputVoltage_mV = 2*unInputVoltage_mV; // The input voltage is divide-by-2 via
                                           // a 10k resistive voltage divider.
  Serial.println("Input voltage: ");
  Serial.print(unInputVoltage_mV);
  Serial.println(" mV");
  
  unSensor_mV = analogRead(2); // Read value of analog sensor.
                               // NOTE: The built-in ADC in Arduino Uno, Nano and
                               // Micro and Pro-micro are 10 bits. With reference
                               // voltage at 5V, each unit interval is 
                               // 5/1023 = 4.888mV.                           
  unSensor_mV = (unSensor_mV*49)/10;  // We use this trick so that the conversion
                                      // works entirely in integer, and within the
                                      // 16-bits limit of the integer datatype used
                                      // by Arduino.   
  UINTto4BCD(unSensor_mV, &obj4BCDSensor);

  // --- Read DHT11, DHT22 sensor signal line ---
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    FLOATto4BCD(event.temperature, &obj4BCDTemp);  // Convert float to 4-digits
                                                   // BCD.
    Serial.println("Temperature:"); 
    Serial.print(obj4BCDTemp.bytThousand); 
    Serial.print(obj4BCDTemp.bytHundred);  
    Serial.print(obj4BCDTemp.bytTen); 
    Serial.print(".");
    Serial.print(obj4BCDTemp.bytDigit);
    Serial.println(F("°C"));
  }
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    FLOATto4BCD(event.relative_humidity, &obj4BCDTemp);  // Convert float to 4-digits
                                                   // BCD.   
    Serial.println("Relative Humidity:"); 
    Serial.print(obj4BCDTemp.bytThousand); 
    Serial.print(obj4BCDTemp.bytHundred);  
    Serial.print(obj4BCDTemp.bytTen); 
    Serial.print(".");
    Serial.print(obj4BCDTemp.bytDigit);                                                    
    Serial.println(F("%"));
  }  
 
  // --- Transmit data packet over nRF24L01+ radio ---
  strTX[0] = _SENSOR_ID;          // Sensor ID
  strTX[1] = 'h';                 // Sensor type, humidity.
  strTX[1] = obj4BCDSensor.bytThousand + 0x30;   // Convert each digit to ASCII.
  strTX[2] = obj4BCDSensor.bytHundred + 0x30;
  strTX[3] = obj4BCDSensor.bytTen + 0x30;
  strTX[4] = obj4BCDSensor.bytDigit + 0x30;
  
  //strTX[2] = obj4BCDTemp.bytThousand + 0x30;   // Convert each digit to ASCII.
  //strTX[3] = obj4BCDTemp.bytHundred + 0x30;
  //strTX[4] = obj4BCDTemp.bytTen + 0x30;
  //strTX[5] = obj4BCDTemp.bytDigit + 0x30;  
  radio.write(&strTX, 6); // Transmit data string to receiver, 6 bytes.
  radio.powerDown(); // Powerdown nRF24L01+ radio to save power.
  digitalWrite(PDEBUG_LED,LOW);   // Turn off debug LED.
  //delay(1500);

  delay(600);                      // Delay is needed in order for PC to recognize the USB
                                   // device during programming. 
  wdt_reset();                     // Reset WDT counter before going to sleep.
  sleep_cpu();                  // Start sleep mode. Power down AVR micro-controller.
}

// Function to convert an unsigned integer value to
// 4-digits BCD (binary-coded decimal) format. The 
// BCD value is stored as a structure.
void  UINTto4BCD(unsigned int unValue, OBJ_4BCD* ptrObj4BCD)
{
  byte  bytTen = 0;
  byte  bytHundred = 0;
  byte  bytThousand = 0;

  if (unValue > 9999)   // Check for overflow.
  {
    ptrObj4BCD->bytDigit = 9;      
    ptrObj4BCD->bytTen = 9;
    ptrObj4BCD->bytHundred = 9;
    ptrObj4BCD->bytThousand = 9;
    return;
  }
  
  ptrObj4BCD->bytDigit = 0;      // Clear all digits first.
  ptrObj4BCD->bytTen = 0;
  ptrObj4BCD->bytHundred = 0;
  ptrObj4BCD->bytThousand = 0;
    
  while (unValue > 999) // Update thousand.
  {
    unValue = unValue - 1000;
    bytThousand++;
  }
  ptrObj4BCD->bytThousand = bytThousand;
  
  while (unValue > 99)  // Update hundred.
  {
    unValue = unValue - 100;
    bytHundred++;
  }
  ptrObj4BCD->bytHundred = bytHundred;
  
  while (unValue > 9)  // Update ten.
  {
    unValue = unValue - 10;
    bytTen++;
  }
  ptrObj4BCD->bytTen = bytTen;
  
  ptrObj4BCD->bytDigit = unValue;   // Update digit.  
}

// Function to convert a positive float value to
// 4-digits BCD (binary-coded decimal) format. The 
// BCD value is stored as a structure, in this format:
// XXX.X (1 decimal place).
//
void  FLOATto4BCD(float fValue, OBJ_4BCD* ptrObj4BCD)
{
  unsigned int unValue;
  byte  bytTen = 0;
  byte  bytHundred = 0;
  byte  bytThousand = 0;

  unValue = 10*fValue;  // We want to preserve the 1 digit
                        // after the decimal, so we multiply
                        // by 10. 
  
  if (unValue > 9999)   // Check for overflow.
  {
    ptrObj4BCD->bytDigit = 9;      
    ptrObj4BCD->bytTen = 9;
    ptrObj4BCD->bytHundred = 9;
    ptrObj4BCD->bytThousand = 9;
    return;
  }
  
  ptrObj4BCD->bytDigit = 0;      // Clear all digits first.
  ptrObj4BCD->bytTen = 0;
  ptrObj4BCD->bytHundred = 0;
  ptrObj4BCD->bytThousand = 0;
  
  while (unValue > 999) // Update thousand.
  {
    unValue = unValue - 1000;
    bytThousand++;
  }
  ptrObj4BCD->bytThousand = bytThousand;
  
  while (unValue > 99)  // Update hundred.
  {
    unValue = unValue - 100;
    bytHundred++;
  }
  ptrObj4BCD->bytHundred = bytHundred;
  
  while (unValue > 9)  // Update ten.
  {
    unValue = unValue - 10;
    bytTen++;
  }
  ptrObj4BCD->bytTen = bytTen;
  
  ptrObj4BCD->bytDigit = unValue;   // Update digit.  
}
