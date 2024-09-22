
/*
  Joe Laporte - RObot Tank
  LoRa Demo 1 Receiver
  lora-demo1-receive.ino
  Receives and displays contents of test packet
  Requires LoRa Library by Sandeep Mistry - https://github.com/sandeepmistry/arduino-LoRa

  Credits to  https://dronebotworkshop.com
*/
 
// Include required libraries
#include <SPI.h>
#include <LoRa.h>
 
// Define the pins used by the LoRa module
const int csPin = 4;     // LoRa radio chip select
const int resetPin = 2;  // LoRa radio reset
const int irqPin = 3;    // Must be a hardware interrupt pin
 
// byte localAddress = 0x01;    // Address of this device (Controller = 0x01)
// byte senderAddress = 0xAA;

 
void setup() {
  Serial.begin(9600);
  while (!Serial)
    ;
 
  // Setup LoRa module
  LoRa.setPins(csPin, resetPin, irqPin);
 
  Serial.println("LoRa Receiver Test");
 
  // Start LoRa module at local frequency
  // 915E6 for North America
 
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
}
 
void loop() {
 
  // Try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // Received a packet
    Serial.print("Received '");
 
    // Read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }
 
    // Print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
}