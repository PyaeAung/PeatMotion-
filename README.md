# PeatMotion
This project focuses on monitoring and analyzing the displacement of peatland using distance sensor and Low-cost GPS.
The goal is to gather data related to the movement, which are crucial for environmental studies.

## Features

- Real-time monitoring of peatland displacement.
- Integration of multiple sensors including GNSS, distance, temperature, Soil temperature, humidity, submersible water level sensor and pressure sensors.
- Data filtering and smoothing using a Kalman filter.
- Sensors data storage in InfluxDB for long-term analysis.
- Visualization using Node-RED and Grafana.

  ## Hardware

- Microcontroller: M5Stack
- Distance Sensor: VL53L1X
- Temperature Sensor: DS18B20
- Humidity and Temperature Sensor: SHT4x
- Pressure and Altitude Sensor: BMP280
- 4-20mA Current Sensor for water level sensor: M5 AIN MODULE_4_20MA

## Software

- Arduino IDE for firmware development.
- Node-RED for data processing and visualization.
- InfluxDB for time-series data storage.
- Grafana for data visualization.
  
