#include <Arduino.h>
#include <SoftwareSerial.h>
// Packet Structure: START BYTE(0x55) | COMMAND | PAYLOAD LENGTH | PAYLOAD | CRC
// Example 6 - Receiving binary data

const byte numBytes = 32;

// Software Serial
const byte rxPin = 2;
const byte txPin = 3;
SoftwareSerial softwareSerial (rxPin, txPin);

byte receivedBytes[numBytes];
byte numReceived = 0;

boolean newData = false;

void setup() {
    Serial.begin(115200);
    Serial.println("<Serial is ready>");
    softwareSerial.begin(115200);
    softwareSerial.println("<Software Serial is ready>");
}

void receiveSerial() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  byte startMarker = 0x55;
  byte endMarker = 0x55;
  byte rb;


  while (Serial.available() > 0 && newData == false) {
    rb = Serial.read();

    if (recvInProgress == true) {
      if (rb != endMarker) {
        receivedBytes[ndx] = rb;
        ndx++;
        if (ndx >= numBytes) {
          ndx = numBytes - 1;
        }
      }
      else {
        receivedBytes[ndx] = '\0'; // terminate the string
        // recvInProgress = false;
        numReceived = ndx;  // save the number for use when printing
        ndx = 0;
        newData = true;
      }
    }
    else if (rb == startMarker) {
      recvInProgress = true;
    }
  }
}

void receiveSoftwareSerial() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  byte startMarker = 0x55;
  byte endMarker = 0x55;
  byte rb;


  while (softwareSerial.available() > 0 && newData == false) {
    rb = softwareSerial.read();

    if (recvInProgress == true) {
      if (rb != endMarker) {
        receivedBytes[ndx] = rb;
        ndx++;
        if (ndx >= numBytes) {
          ndx = numBytes - 1;
        }
      }
      else {
        receivedBytes[ndx] = '\0'; // terminate the string
        // recvInProgress = false;
        numReceived = ndx;  // save the number for use when printing
        ndx = 0;
        newData = true;
      }
    }
    else if (rb == startMarker) {
      recvInProgress = true;
    }
  }
}

void printSerial() {
    if (newData == true) {
        Serial.print("HEX values: ");
        for (byte n = 0; n < numReceived; n++) {
            Serial.print(receivedBytes[n], HEX);
            Serial.print(' ');
        }
        Serial.println();
        newData = false;
    }
}

void printSoftwareSerial() {
    if (newData == true) {
        softwareSerial.print("HEX values: ");
        for (byte n = 0; n < numReceived; n++) {
            Serial.print(receivedBytes[n], HEX);
            Serial.print(' ');
        }
        softwareSerial.println();
        newData = false;
    }
}

void loop() {
  receiveSerial();
  printSoftwareSerial();
  receiveSoftwareSerial();
  printSerial();
}