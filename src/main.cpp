#include <Arduino.h>
#include <Wire.h>
#include <AloraSensorKit.h>
#include <Adafruit_SSD1306.h>
#include <HardwareSerial.h>
#include <NMEAGPS.h>
#include <Streamers.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

Adafruit_SSD1306 display;
AloraSensorKit sensorKit;

#define GPS_RX 14
#define GPS_TX 12
#define GPS_BAUD 9600
HardwareSerial gpsSerial(1);
SensorValues sensorValue;

void displaySensorValue(SensorValues& value);
void displayGPSInformation(SensorValues& value);
void displayValuesTask(void* arg);

void setup() {
    Serial.begin(9600);

    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
    sensorKit.begin();
    sensorKit.initGPS(&gpsSerial);

    /* initialize display */
    display.begin();
    display.display();

    /* create task to display sensor values on OLED */
    xTaskCreate(displayValuesTask, "display_values", 2048, NULL, 5, NULL);
}

void loop() {
    /* read sensor data */
    sensorKit.run();
    sensorValue = sensorKit.getLastSensorData();
    trace_all(Serial, *sensorKit.getGPSObject(), sensorValue.gpsFix);

    displaySensorValue(sensorValue);

    delay(3000);
}

/**
 * @brief Task for displaying result of measurement on the OLED
 * 
 * @param arg unused
 */
void displayValuesTask(void* arg) {
    int displayInterval = 2000;

    while (true) {
        displaySensorValue(sensorValue);
        delay(displayInterval);
        displayGPSInformation(sensorValue);
        delay(displayInterval);
    }
}

/**
 * @brief Display GPS location information on the OLED
 * 
 * @param value measurement data from sensors on Alora
 */
void displayGPSInformation(SensorValues& value) {
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(0, 0);

    display.setTextSize(1);
    display.println("Latitude");
    display.setTextSize(2);
    display.println(value.gpsFix.latitude(), 4);
    display.println("");

    display.setTextSize(1);
    display.println("Longitude");
    display.setTextSize(2);
    display.println(value.gpsFix.longitude(), 4);

    display.display();
}

/**
 * @brief Display sensor values (temperature and light intensity) on the OLED
 * 
 * @param value 
 */
void displaySensorValue(SensorValues& value) {
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(0, 0);

    display.setTextSize(1);
    display.println("Temperature");
    display.setTextSize(2);
    display.println(value.T1, 2);
    display.println("");

    display.setTextSize(1);
    display.println("Lux");
    display.setTextSize(2);
    display.println(value.lux, 2);

    display.display();
}
