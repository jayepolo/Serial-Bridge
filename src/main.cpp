/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-one-to-many-esp8266-nodemcu/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

int LED = 2;
bool Received = false;
bool blinkingFlag = false;
unsigned long startTime;
unsigned long now;
//unsigned long onTime;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  unsigned long sample;
  bool z1;
  bool z2;
  bool z3;
  bool z4;
  bool z5;
  bool z6;
  bool boiler;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&myData, incomingData, sizeof(myData));

  // Serial.print(myData.sample);
  // Serial.print(" ");
  // Serial.print(myData.z1);
  // Serial.print(myData.z2);
  // Serial.print(myData.z3);
  // Serial.print(myData.z4);
  // Serial.print(myData.z5);
  // Serial.print(myData.z6);
  // Serial.print(" ");
  // Serial.println(myData.boiler);

  Serial.print("{\"boilerMsg\": ");
    Serial.print("{");
    Serial.print("\"sample\": ");
    Serial.print(myData.sample);
    // Serial.print(",\"MAC\": \"");
    // Serial.print(myData.MAC);
    //Serial.print("18:FE:34:D7:7D:90");
    //Serial.print("\"},");
    Serial.print("},");
  Serial.print("\"data\": ");
    Serial.print("{");
    Serial.print("\"z1\": ");
    Serial.print(myData.z1);
    Serial.print(",\"z2\": ");
    Serial.print(myData.z2);
    Serial.print(",\"z3\": ");
    Serial.print(myData.z3);
    Serial.print(",\"z4\": ");
    Serial.print(myData.z4);
    Serial.print(",\"z5\": ");
    Serial.print(myData.z5);
    Serial.print(",\"z6\": ");
    Serial.print(myData.z6);
    Serial.print(",\"boiler\": ");
    Serial.print(myData.boiler);
    Serial.print("}");
  Serial.print("}");
  Serial.println();

  Received = true;
  // digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
  // delay(50);                // Wait for a bit
  // digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
}
 
void setup() {
  pinMode(2, OUTPUT);     // Initialize the LED_BUILTIN pin as an output

  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  now = millis();
  if (Received == true) {
    if (blinkingFlag == true) {
      Serial.println("Data received while still processing last data.");
    }
    Received = false;
    digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
    blinkingFlag = true;
    startTime = now;
  }
  if (blinkingFlag == true) {
    if (now >= startTime + 50) {
      digitalWrite(LED_BUILTIN, HIGH);   // Turn the LED on (Note that LOW is the voltage level
      blinkingFlag = false;
    }
  }
}
