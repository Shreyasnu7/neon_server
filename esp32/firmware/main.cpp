/**
 * ULTRA DRONE - ESP32 SENSOR HUB
 * ------------------------------
 * Role: 
 *  1. Read 4x VL53L0X ToF Sensors (I2C Multiplexed via XSHUT)
 *  2. Read MPU6050 Gyro
 *  3. Control Gimbal Servos & LEDs
 *  4. Send Data to Radxa via WiFi UDP
 * 
 * Pins:
 *  - SDA: 21, SCL: 22
 *  - XSHUT1(FL):23, XSHUT2(RL):26, XSHUT3(FR):15, XSHUT4(RR):5
 *  - LED: 33
 *  - Gimbal: 25, 32
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>

// --- WIFI CONFIG ---
const char* ssid = "UltraDrone_Brain"; // Radxa Hotspot
const char* password = "password123";
const char* radxa_ip = "192.168.4.1"; // Default gateway usually
unsigned int localPort = 8888;
WiFiUDP udp;

// --- PIN DEFS ---
#define PIN_XSHUT1 23
#define PIN_XSHUT2 26
#define PIN_XSHUT3 15
#define PIN_XSHUT4 5

#define PIN_LED 33
#define PIN_GIM_PITCH 25
#define PIN_GIM_YAW 32
#define PIN_CAM_TRIG 4

// --- OBJECTS ---
Adafruit_VL53L0X tof1 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof2 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof3 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof4 = Adafruit_VL53L0X();
Adafruit_MPU6050 mpu;
Adafruit_NeoPixel strip(16, PIN_LED, NEO_GRB + NEO_KHZ800);
Servo gimPitch;
Servo gimYaw;

// --- STATE ---
unsigned long last_send = 0;

void setID() {
    // Graceful reset of all ToF sensors
    digitalWrite(PIN_XSHUT1, LOW);
    digitalWrite(PIN_XSHUT2, LOW);
    digitalWrite(PIN_XSHUT3, LOW);
    digitalWrite(PIN_XSHUT4, LOW);
    delay(10);
    
    // All high (enabled)
    digitalWrite(PIN_XSHUT1, HIGH);
    digitalWrite(PIN_XSHUT2, HIGH);
    digitalWrite(PIN_XSHUT3, HIGH);
    digitalWrite(PIN_XSHUT4, HIGH);
    delay(10);
    
    // Init 1
    digitalWrite(PIN_XSHUT1, HIGH);
    digitalWrite(PIN_XSHUT2, LOW);
    digitalWrite(PIN_XSHUT3, LOW);
    digitalWrite(PIN_XSHUT4, LOW);
    if(tof1.begin(0x30)) { Serial.println("ToF1 OK"); }
    
    // Init 2
    digitalWrite(PIN_XSHUT2, HIGH);
    if(tof2.begin(0x31)) { Serial.println("ToF2 OK"); }
    
    // Init 3
    digitalWrite(PIN_XSHUT3, HIGH);
    if(tof3.begin(0x32)) { Serial.println("ToF3 OK"); }
    
    // Init 4
    digitalWrite(PIN_XSHUT4, HIGH);
    if(tof4.begin(0x33)) { Serial.println("ToF4 OK"); }
}

void setup() {
    Serial.begin(115200);
    
    // Pins
    pinMode(PIN_XSHUT1, OUTPUT);
    pinMode(PIN_XSHUT2, OUTPUT);
    pinMode(PIN_XSHUT3, OUTPUT);
    pinMode(PIN_XSHUT4, OUTPUT);
    pinMode(PIN_CAM_TRIG, OUTPUT);
    
    // Servos
    gimPitch.attach(PIN_GIM_PITCH);
    gimYaw.attach(PIN_GIM_YAW);
    
    // LED
    strip.begin();
    strip.setPixelColor(0, strip.Color(255, 0, 0)); // Red init
    strip.show();
    
    // Sensors
    Wire.begin(21, 22);
    setID();
    if (mpu.begin()) {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }
    
    // WiFi
    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println(" Connected!");
    udp.begin(localPort);
    
    strip.setPixelColor(0, strip.Color(0, 255, 0)); // Green ready
    strip.show();
}

void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // Read Distances
    uint16_t d1 = tof1.readRange();
    uint16_t d2 = tof2.readRange();
    uint16_t d3 = tof3.readRange();
    uint16_t d4 = tof4.readRange();
    
    // Recv UDP Commands
    int packetSize = udp.parsePacket();
    if (packetSize) {
        char packetBuffer[255];
        int len = udp.read(packetBuffer, 255);
        if (len > 0) packetBuffer[len] = 0;
        
        // Parse JSON
        StaticJsonDocument<200> doc;
        DeserializationError error = deserializeJson(doc, packetBuffer);
        if (!error) {
            if (doc.containsKey("led")) {
                const char* c = doc["led"];
                if(strcmp(c, "RED")==0) strip.fill(strip.Color(255,0,0));
                if(strcmp(c, "BLUE")==0) strip.fill(strip.Color(0,0,255));
                strip.show();
            }
            if (doc.containsKey("gim")) {
                int p = doc["gim"][0];
                int y = doc["gim"][1];
                gimPitch.write(map(p, -90, 90, 0, 180));
                gimYaw.write(map(y, -90, 90, 0, 180));
            }
        }
    }
    
    // Send Telemetry (20Hz)
    if (millis() - last_send > 50) {
        StaticJsonDocument<200> telem;
        JsonArray t = telem.createNestedArray("tof");
        t.add(d1); t.add(d2); t.add(d3); t.add(d4);
        
        JsonArray gyr = telem.createNestedArray("gyro");
        gyr.add(g.gyro.x); gyr.add(g.gyro.y); gyr.add(g.gyro.z);
        
        char buffer[200];
        serializeJson(telem, buffer);
        
        udp.beginPacket(radxa_ip, localPort);
        udp.write((const uint8_t*)buffer, strlen(buffer));
        udp.endPacket();
        
        last_send = millis();
    }
}
