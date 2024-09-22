// Include required libraries
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <string>
#include <SPI.h>
#include <LoRa.h>
 
// Define the pins used by the LoRa module
const int csPin = 5;     // LoRa radio chip select
const int resetPin = 14;  // LoRa radio reset
const int irqPin = 3;    // Must be a hardware interrupt pin

// RX and TX pins for the GPS 
#define GPS_RX 16
#define GPS_TX 17

// address of this device (must be unique, 0xAA or 0xBB)
byte localAddress = 0xAA;  
byte destination = 0x01;

// Createing a serial object. 
SoftwareSerial serial(GPS_RX, GPS_TX);

// Defining the GPS serial ports.
#define GPS_SERIAL serial

// Creating a TinyGPS object. 
TinyGPSPlus gps;

// Message counter
byte msgCount = 0;
String gps_data = "";
 
void setup() {

  Serial.begin(9600);
  GPS_SERIAL.begin(9600);

  while (!Serial)
    ;
 
  // Setup LoRa module
  LoRa.setPins(csPin, resetPin, irqPin);
 
  Serial.println("LoRa Sender Test");
 
  // Start LoRa module at local frequency
  // 915E6 for North America
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
}

void loop() {

  while(GPS_SERIAL.available())
  {
    int c = GPS_SERIAL.read();
    Serial.write(c);

    if(gps.encode(c))
    {
      Serial.println();
      float lat = gps.location.lat();
      float lng = gps.location.lng();
      
      // Convert the float values to strings
      String gps_data = String(lat) + " " + String(lng);
    
      Serial.print("Sending packet: ");
      Serial.println(msgCount);
    
      // Send packet
      LoRa.beginPacket();
      LoRa.print("Packet ");
      LoRa.print(gps_data );
      LoRa.endPacket();
    
      // Increment packet counter
      msgCount++;
    
      // 5-second delay
      delay(5000);
    }
  }
}