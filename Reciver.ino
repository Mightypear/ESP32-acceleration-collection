/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  float ax;
  float ay;
  float az;
  float tid;  // New variable to store time
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const esp_now_recv_info *recvInfo, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  
  // Serial.print("Bytes received: ");
  // Serial.println(len);
  //Serial.print("Time : ");
  Serial.println(" ");

  Serial.print(myData.tid,2 );  // Print time with 1 decimal place
  Serial.print(",");
  //Serial.print("Acceleration X: ");
  // Serial.print(myData.ax);
  // Serial.print(",");
  //Serial.print("Acceleration Y: ");
  // Serial.print(myData.ay);
  // Serial.print(",");
  //Serial.print("Acceleration Z: ");
  Serial.print(myData.az);
  
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register the receive callback function
  esp_now_register_recv_cb(OnDataRecv);
  
  Serial.println("ESP-NOW Receiver ready.");
}

void loop() {
  // The loop is intentionally left empty as data reception is handled in the callback
}
