#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

const char* ssid = "abc";
const char* password = "12345678";
const char* server = "http://192.168.131.7:5000/data";
float calibration = 0.38;  // Adjust this based on actual voltage readings
int analogInPin = A0;  // Battery voltage sensing pin
int sensorValue;
float voltage = 0;
float bat_percentage = 0; 
WiFiClient client;
HTTPClient http;

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected!");
}

void loop() {
    if (WiFi.status() == WL_CONNECTED) {
        http.begin(client, server);
        http.addHeader("Content-Type", "application/x-www-form-urlencoded");
        sensorValue = analogRead(analogInPin);
        voltage = (((sensorValue * 3.3) / 1024) * 2 + calibration); // Voltage calculation using voltage divider
        bat_percentage = mapfloat(voltage, 2.8, 4.2, 0, 100);
        bat_percentage = constrain(bat_percentage, 1, 100);  // Ensure value stays between 1% and 100%
        String postData = "sensor_value=" + String(bat_percentage);
        int httpResponseCode = http.POST(postData);
        Serial.print("Charging: ");

        Serial.println("Voltage: " + String(voltage) + "V");
        Serial.println("Battery Percentage: " + String(bat_percentage) + "%");
        Serial.println("Response Code: " + String(httpResponseCode));
        http.end();
    } 
    delay(4000); // Send data every 4 seconds
}
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}