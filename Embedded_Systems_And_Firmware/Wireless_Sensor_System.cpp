/**
 * imu_wireless_node.cpp
 * --------------------------------------
 * ESP32-Based Multi-IMU Wireless Sensor Node
 *
 * This embedded firmware collects real-time data from multiple BNO055 IMUs
 * connected through a TCA9548A I2C multiplexer and transmits it using ESP-NOW
 * to a remote receiver. It supports persistent calibration storage, real-time
 * orientation and motion data collection, and GPIO-interrupt-based session control.
 *
 * Features:
 * - Initialization of up to 6 IMU sensors
 * - Persistent storage of calibration data using Preferences
 * - Wireless peer-to-peer communication using ESP-NOW
 * - Real-time sensor fusion data: Euler angles, acceleration, gravity, quaternion
 * - Manual calibration and trigger buttons
 * - Modular struct-based data handling for scalable multi-IMU streaming
 *
 * Author: Reim Yousef
 * Platform: ESP32
 * Date: [Fill me in when you're less stressed]
 */

 #include <Preferences.h>
 #include <Wire.h>
 #include <Adafruit_Sensor.h>
 #include <Adafruit_BNO055.h>
 #include <utility/imumaths.h>
 #include <esp_now.h>
 #include <WiFi.h>
 #include <TCA9548A.h>
 
 // Pin definitions
 const int buttonPin = 5;  
 const int calbutton = 25; 
 
 // Timing and calibration
 unsigned long curr_time;  
 unsigned long mill_prev;
 unsigned long micro_prev;
 bool fully_calib = false;
 
 // Button states
 int button_state = -1;
 int button_cal = -1; 
 int button_press;
 int button_cal_check;
 
 Preferences preferences; // Persistent storage
 TCA9548A tca;             // I2C multiplexer
 
 #define BNO055_SAMPLERATE_DELAY_MS (10)
 #define TCA9548A_ADDRESS 0x70
 
 Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
 
 // ESP-NOW MAC addresses
 uint8_t broadcastAddress[]  = {0x7C, 0xDF, 0xA1, 0xDE, 0xE5, 0x78};
 uint8_t broadcastAddress2[] = {0x68, 0x67, 0x25, 0x4E, 0x78, 0xC0};
 uint8_t broadcastAddress3[] = {0x58, 0xCF, 0x79, 0x05, 0x4C, 0x40};
 
 // Primary data structure for IMU readings
 typedef struct struct_message {
   int id, calib, system, gyro, accel, mag;
   double acc_x, acc_y, acc_z;
   double euler_x, euler_y, euler_z;
   double grav_x, grav_y, grav_z;
   double or_est_y, or_est_z;
   int curr_time;
 } struct_message;
 
 struct_message myData;
 struct_message board2, board3, board4, board5, board6, board7;
 struct_message boardsStruct[6] = {board2, board3, board4, board5, board6, board7};
 
 // Minimal struct for button state
 typedef struct struct_message2 {
   int d;
 } struct_message2;
 
 struct_message2 myData2, myData3;
 esp_now_peer_info_t peerInfo;
 bool sensorInitialized[6] = {false};
 
 void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
   memcpy(&myData, incomingData, sizeof(myData));
   boardsStruct[myData.id-2] = myData;
 }
 
 void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
   Serial.print("\r\nSend Status: ");
   Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
 }
 
 // --- setup, loop, and other core functions would follow ---
 // Skipped here for brevity but you'd include full versions with spacing, comments, and modular breakdown.
 