/*#include <ESP8266WiFi.h>
#include <Wire.h>
#include <MPU6050.h>
#include <WiFiClient.h>
#include <ESP8266HTTPClient.h>

MPU6050 mpu;
const int S0 = D5;
const int S1 = D6;
const int S2 = D7;
const int analogPin = A0;

const char* ssid = "MCA_IOT";
const char* password = "23471482024";

const char* server = "api.thingspeak.com";
const String apiKey = "SJVQUTQELHREHI2B";

WiFiClient client;

void setup() {
    Wire.begin();
    Serial.begin(9600);
    mpu.initialize();
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
}

void loop() {
    int sensorValue1 = readSensor(0);
    int sensorValue2 = readSensor(1);

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);

    float accelX = ax / 16384.0;
    float accelY = ay / 16384.0;
    float accelZ = az / 16384.0;

    float gyroX = gx / 131.0;
    float gyroY = gy / 131.0;
    float gyroZ = gz / 131.0;

    Serial.print("Sensor 1: ");
    Serial.println(sensorValue1);
    Serial.print("Sensor 2: ");
    Serial.println(sensorValue2);
    Serial.print("Acceleration: ");
    Serial.print("X="); Serial.print(accelX);
    Serial.print(" Y="); Serial.print(accelY);
    Serial.print(" Z="); Serial.println(accelZ);
    Serial.print("Gyroscope: ");
    Serial.print("X="); Serial.print(gyroX);
    Serial.print(" Y="); Serial.print(gyroY);
    Serial.print(" Z="); Serial.println(gyroZ);

    // Upload data to ThingSpeak
    uploadToThingSpeak(sensorValue1, sensorValue2, accelX, accelY, accelZ, gyroX, gyroY, gyroZ);

    delay(1000);  // Adjust delay as per your requirement
}

int readSensor(int channel) {
    digitalWrite(S0, bitRead(channel, 0));
    digitalWrite(S1, bitRead(channel, 1));
    digitalWrite(S2, bitRead(channel, 2));
    return analogRead(analogPin);
}

void uploadToThingSpeak(int sensorValue1, int sensorValue2, float accelX, float accelY, float accelZ, float gyroX, float gyroY, float gyroZ) {
    if (client.connect(server, 80)) {
        String url = "/update?api_key=" + apiKey +
                     "&field1=" + String(sensorValue1) +
                     "&field2=" + String(sensorValue2) +
                     "&field3=" + String(accelX) +
                     "&field4=" + String(accelY) +
                     "&field5=" + String(accelZ) +
                     "&field6=" + String(gyroX) +
                     "&field7=" + String(gyroY) +
                     "&field8=" + String(gyroZ);

        Serial.print("Sending data to ThingSpeak: ");
        Serial.println(url);

        client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                     "Host: " + server + "\r\n" +
                     "Connection: close\r\n\r\n");

        delay(1000); // Delay to allow the server to process the request

        while (client.available()) {
            String line = client.readStringUntil('\r');
            Serial.print(line);
        }
        Serial.println();
        client.stop();
    } else {
        Serial.println("Failed to connect to ThingSpeak server!");
    }
}*/


#include <ESP8266WiFi.h>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
const int S0 = D5;
const int S1 = D6;
const int S2 = D7;
const int analogPin = A0;

const char* ssid = "MCA_IOT";
const char* password = "23471482024";

//WiFiClient client;

void setup() {
    Wire.begin();
    Serial.begin(9600);
    mpu.initialize();
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);

    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
}

void loop() {
    // Read data from 5 sensors
    int sensorValues[5];
    for (int i = 0; i < 5; i++) {
        sensorValues[i] = readSensor(i);
    }

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);

    float accelX = ax / 16384.0;
    float accelY = ay / 16384.0;
    float accelZ = az / 16384.0;

    float gyroX = gx / 131.0;
    float gyroY = gy / 131.0;
    float gyroZ = gz / 131.0;

    // Display sensor values on Serial Monitor
    Serial.println("Sensor Values:");
    for (int i = 0; i < 5; i++) {
        Serial.print("Sensor ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(sensorValues[i]);
    }

    Serial.print("Acceleration: ");
    Serial.print("X="); Serial.print(accelX);
    Serial.print(" Y="); Serial.print(accelY);
    Serial.print(" Z="); Serial.println(accelZ);

    Serial.print("Gyroscope: ");
    Serial.print("X="); Serial.print(gyroX);
    Serial.print(" Y="); Serial.print(gyroY);
    Serial.print(" Z="); Serial.println(gyroZ);

    delay(1000);  // Adjust delay as per your requirement
}

int readSensor(int channel) {
    // Ensure channel is within the valid range
    if (channel < 0 || channel > 4) {
        Serial.println("Invalid sensor channel");
        return 0;
    }

    // Set S0, S1, S2 pins according to the sensor channel
    digitalWrite(S0, bitRead(channel, 0));
    digitalWrite(S1, bitRead(channel, 1));
    digitalWrite(S2, bitRead(channel, 2));
    return analogRead(analogPin);
}

