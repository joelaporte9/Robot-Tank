#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <LoRa.h>

SoftwareSerial serialConnection(3, 1);

// GPS instance // '2' corresponds to UART2
TinyGPSPlus gps;             

// Define the pins used by the LoRa module
const int csPin = 5;     // LoRa radio chip select
const int resetPin = 14;  // LoRa radio reset
const int irqPin = 27;    // Must be a hardware interrupt pin
// const byte rxPin = 3;
// const byte txPin = 1;
const int ledPin = 2; // Onboard LED pin for ESP32

// Set up a new SoftwareSerial object

// SoftwareSerial mySerial (rxPin, txPin);
// Outgoing message variable
String outMessage;
 
// Controller data variable
String inMessage;
 
// Previous value Controller data variable
String inMessageOld;
 
// Outgoing Message counter
byte msgCount = 0;
 
// Source and destination addresses
byte localAddress = 0xAA;  // address of this device (must be unique, 0xAA or 0xBB)
byte destination = 0x01;   // destination to send to (controller = 0x01)
 

// Receive Callback Function
void onReceive(int packetSize) {
  if (packetSize == 0) return;  // if there's no packet, return
 
  // Read packet header bytes:
  int recipient = LoRa.read();        // recipient address
  byte sender = LoRa.read();          // sender address
  byte incomingMsgId = LoRa.read();   // incoming msg ID
  byte incomingLength = LoRa.read();  // incoming msg length
 
  String incoming = "";  // payload of packet
 
  while (LoRa.available()) {        // can't use readString() in callback, so
    incoming += (char)LoRa.read();  // add bytes one by one
  }
 
  if (incomingLength != incoming.length()) {  // check length for error
    Serial.println("error: message length does not match length");
    return;  // skip rest of function
  }
 
  // If the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return;  // skip rest of function
  }
 
  // If we are this far then this message is for us
  // Update the controller data variable
  inMessage = incoming;
}
 
// Send LoRa Packet
void sendMessage(String outgoing) {
  LoRa.beginPacket();             // start packet
  LoRa.write(destination);        // add destination address
  LoRa.write(localAddress);       // add sender address
  LoRa.write(msgCount);           // add message ID
  LoRa.write(outgoing.length());  // add payload length
  LoRa.print(outgoing);           // add payload
  LoRa.endPacket();               //
  digitalWrite(ledPin, LOW);      // finish packet and send it
  msgCount++;                     // increment message ID
}
 
void setup() {
  // Initialize the built-in Serial for debugging (UART0)
  Serial.begin(9600);
  // Initialize UART2 with custom TX and RX pins
  serialConnection.begin(9600); // Set baud rate to 9600, with TX
  Serial.println("GT-U& GPS connected - Ready to start");
}

void loop() {
  while(serialConnection.available())
  {
    gps.encode(serialConnection.read());
  }
  if(gps.location.isUpdated())
  {
    int satelliteValue = (gps.satellites.value());
    double lat = (gps.location.lat());
    double lng = (gps.location.lng());
    float mph = (gps.speed.mph());
    float altitude = (gps.altitude.feet());

    Serial.println("Satellite count:");
    Serial.println(satelliteValue);
    Serial.println("Latitude:");
    Serial.println(lat, 6);
    Serial.println("Longitude:");
    Serial.println(lng, 6);
    Serial.println("MPH:");
    Serial.println(mph);
    Serial.println("Altitude:");
    Serial.println(altitude);
    Serial.println(" ");

     // Format the outgoing message string
    String outMsg = "";
    outMsg = String(satelliteValue) + ":" + String(lat, 6) + ":" + String(lng, 6) + ":" + String(mph) + ":" + String(altitude); 
    // Send data as LoRa packet
    sendMessage(outMsg);
 
    // Print controller variables
    Serial.print("Old Controller Data = ");
    Serial.println(inMessageOld);
    Serial.print("New Controller Data = ");
    Serial.println(inMessage);
 
    // Update the"old" data variable
    inMessageOld = inMessage;
 
    // Place LoRa in Receive Mode
    LoRa.receive();

    // 2-second delay for GPS 
    delay(2000);

  }

}
