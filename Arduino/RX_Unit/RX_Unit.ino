// Sample program for interfacing nRF24L01+ radio module with Arduino,
// receiving text string from multiple transmitters using different datapipes.
// on same RF channel.
// Note that serial port of 115200 bps is used to improve throughput.
//
// Author         : Fabian Kung
// Last modified  : 6 June 2022
// Arduino Board  : Pro-micro
// For tutorial on nRF24L01+ module and the supporting Arduino
// library, please visit:
// https://lastminuteengineers.com/nrf24l01-arduino-wireless-communication/
//
// Usage:
// 1. Make sure the address[6]=....   matches both transmit and receive 
//    Sensor Nodes.
// 2. Arduino needs to be connected to computer via USB port.
// 3. Call up Arduino IDE, then invoke the Serial Monitor.
// 4. Set baud rate to 115200 bps, Both NL & CR for line ending.
// 5. To start, set a character 's' to the Arduino via Serial Monitor to start 
//    the software in Arduino.
// 6. If another Sensor Node in transmit mode is nearby (with matching address), 
//    the string send by the Sensor Node should appear in the Serial Monitor
//    display. 

// Include libraries
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define PDEBUG_LED 6
#define PBAT_STAT   7
#define PBAT_LEVEL 0    // AN0 - Connected to a 1:2 resistive voltage divider.

//create an RF24 object
RF24 radio(9, 8);  // CE, CSN

//address through which two modules communicate.
uint64_t address[6] = {0x7878787878LL,
                       0xB3B4B5B6F1LL,
                       0xB3B4B5B6CDLL,
                       0xB3B4B5B6A3LL,
                       0xB3B4B5B60FLL,
                       0xB3B4B5B605LL
                      };
char strRX[16] = {0};             // RX string buffer.

void setup() {
  pinMode(4, OUTPUT);           // Set all unused digital pins to output, and set level to '0'.
  pinMode(5, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);   
 
  pinMode(PDEBUG_LED,OUTPUT);    // Driver pin for external debug LED.
  pinMode(PBAT_STAT, INPUT);    // Battery charger status monitoring pin.
  
  while (!Serial);              // Initialized on-board UART port.
    Serial.begin(115200);
    
  Serial.println("Input 's' to start");
  while(1)                          // Infinite loop, wait for start character.    
  {
    if (Serial.available() > 0)     // Check if any data coming from serial port.
    {
      if (Serial.read() == 's')     // Check if it is 's'.
      {
        Serial.println("Starting...");
        break;
      }
    }
  }
      
  radio.begin();
  // Set the channel, carrier frequency = 2400 + [Channel No.] MHz
  // Each channel occupies 1 MHz bandwidth. 
  radio.setChannel(100); // Use 2500 MHz, to avoid interference with WiFi 2.4GHz system.
  // Maximum channel = 125.  
  // Set data rate.
  radio.setDataRate(RF24_250KBPS); // Set to lower datarate, this is also most sensitive
                                   // at -94 dBm. Default is RF24_1MBPS with lower 
                                   // sensitivity.
                                   // If we use 2000 kbps baud rate, the channel interval 
                                   // should be 2 MHz to prevent interference.
  //set the address
  radio.openReadingPipe(0, address[0]); 
  radio.openReadingPipe(1, address[1]);
  radio.openReadingPipe(2, address[2]);
  radio.openReadingPipe(3, address[3]);
  radio.openReadingPipe(4, address[4]);
  radio.openReadingPipe(5, address[5]);
  //Set module as receiver
  radio.startListening();  
  Serial.println("IOT Sensor System ready");
}

void loop() {
  int nIndex;
  byte bytPipeNo;
  
  //Read the data if available in buffer.
  if (radio.available(&bytPipeNo))
  {
    digitalWrite(PDEBUG_LED,HIGH);     // Light up debug LED.
    radio.read(&strRX, sizeof(strRX)); // Read data. This will attempt to read
                                       // up to the size of strRX.
    Serial.print(bytPipeNo);           // Print datapipe number.
    Serial.println(strRX);             // Send receive text string to display.
    digitalWrite(PDEBUG_LED,LOW);      // Turn off debug LED.
  }
}
