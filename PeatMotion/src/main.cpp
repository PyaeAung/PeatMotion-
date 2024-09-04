#include "M5Unified.h"
#include "M5UnitENV.h"
#include <Wire.h>
#include <VL53L1X.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "MODULE_4_20MA.h"
#include "KalmanFilter.h" // Include the Kalman Filter class

// DS18B20 setup
#define ONE_WIRE_BUS 8  // Use G8 for DS18B20
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);

// Other sensor setups
VL53L1X tofSensor;
SHT4X sht4;
BMP280 bmp;
MODULE_4_20MA meter;

// Kalman Filters
KalmanFilter tempKalman(0.01, 1.0, 1.0, 0.0);  // Temperature
KalmanFilter distanceKalman(0.1, 10.0, 1.0, 0.0);  // ToF Distance
KalmanFilter waterLevelKalman(0.1, 5.0, 1.0, 0.0);  // Water Level

// Timing variables
unsigned long previousMillis = 0;
const long interval = 10000; // Interval between sensor readings

void setup() {
    M5.begin(); // Initialize with default settings
    Serial.begin(115200);
    M5.Power.begin();

    // Initialize DS18B20
    ds18b20.begin();

    // Initialize ToF4M sensor
    M5.Ex_I2C.begin();
    tofSensor.setBus(&Wire);
    tofSensor.setTimeout(500);
    if (!tofSensor.init()) {
        Serial.println("Failed to detect and initialize ToF4M sensor!");
    } else {
        tofSensor.setDistanceMode(VL53L1X::Long);
        tofSensor.setMeasurementTimingBudget(50000);
        tofSensor.startContinuous(50);
    }

    // Initialize ENV IV SHT4x sensor
    if (!sht4.begin(&Wire, SHT40_I2C_ADDR_44, 2, 1, 400000U)) {
        Serial.println("Couldn't find SHT4x sensor");
    } else {
        sht4.setPrecision(SHT4X_HIGH_PRECISION);
        sht4.setHeater(SHT4X_NO_HEATER);
    }

    // Initialize ENV IV BMP280 sensor
    if (!bmp.begin(&Wire, BMP280_I2C_ADDR, 2, 1, 400000U)) {
        Serial.println("Couldn't find BMP280 sensor");
    } else {
        bmp.setSampling(BMP280::MODE_NORMAL,     /* Operating Mode. */
                        BMP280::SAMPLING_X2,     /* Temp. oversampling */
                        BMP280::SAMPLING_X16,    /* Pressure oversampling */
                        BMP280::FILTER_X16,      /* Filtering. */
                        BMP280::STANDBY_MS_500); /* Standby time. */
    }
    
    // Initialize 4-20mA module for water level sensor
    if (!meter.begin(&Wire, MODULE_4_20MA_ADDR, 2, 1, 100000UL)) {
        Serial.println("No 4-20mA Module!");
    }
}

void loop() {
    unsigned long currentMillis = millis();

    // Check if it's time for the next reading
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        // Read and print ToF4M data
        if (tofSensor.timeoutOccurred()) {
            Serial.println("ToF sensor timeout occurred!");
        } else {
            float distance = tofSensor.read();
            distance = distanceKalman.update(distance);
            Serial.print("Filtered ToF Distance: ");
            Serial.print(distance);
            Serial.println(" mm");
        }

        // Read and print SHT4x data
        if (sht4.update()) {
            Serial.println("-----SHT4X-----");
            float temp = tempKalman.update(sht4.cTemp);
            Serial.print("Filtered Temperature: ");
            Serial.print(temp);
            Serial.println(" degrees C");
            Serial.print("Humidity: ");
            Serial.print(sht4.humidity);
            Serial.println("% rH");
            Serial.println("-------------\r\n");
        }

        // Read and print BMP280 data
        if (bmp.update()) {
            Serial.println("-----BMP280-----");
            Serial.print(F("Temperature: "));
            Serial.print(bmp.cTemp);
            Serial.println(" degrees C");
            Serial.print(F("Pressure: "));
            Serial.print(bmp.pressure);
            Serial.println(" Pa");
            Serial.print(F("Approx altitude: "));
            Serial.print(bmp.altitude);
            Serial.println(" m");
            Serial.println("-------------\r\n");
        }

        // Read and print DS18B20 data
        ds18b20.requestTemperatures();
        float ds18b20Temp = ds18b20.getTempCByIndex(0);
        if (ds18b20Temp != DEVICE_DISCONNECTED_C) {
            ds18b20Temp = tempKalman.update(ds18b20Temp);
            Serial.print("Filtered DS18B20 Temperature: ");
            Serial.print(ds18b20Temp);
            Serial.println(" degrees C");
        } else {
            Serial.println("DS18B20 sensor disconnected!");
        }

        // Read and print water level sensor data
        float current_mA = (float)(meter.getCurrentValue(0)) / 100.0;
        if (current_mA >= 4.0 && current_mA <= 20.0) {
            float water_level = map(current_mA, 4.0, 20.0, 0.0, 100.0); // Adjust for your sensor
            water_level = waterLevelKalman.update(water_level);
            Serial.print("Filtered Water Level (mA): ");
            Serial.print(current_mA);
            Serial.println(" mA");
            Serial.print("Filtered Water Level: ");
            Serial.print(water_level);
            Serial.println("%");
        } else {
            Serial.println("Water level sensor error or out of range!");
        }
    }
}
