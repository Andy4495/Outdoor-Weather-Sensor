Battery Powered Outdoor Weather Sensor
======================================

This project is a battery-powered wireless outdoor weather sensor. It is designed to run on an MSP-EXP430F5529LP LaunchPad, 430BOOST-CC110L BoosterPack, and BOOSTXL-SENSORS BoosterPack, all from Texas Instruments.

Data from the TMP007, BME280, and OPT3001 sensors are periodically sent to a wireless receiver hub using the CC110L BoosterPack. A [receiver hub implementation](https://gitlab.com/Andy4495/Sensor-Receiver) is available on GitLab.

The project is designed for low-power operation. [EnergyTrace](http://www.ti.com/tool/ENERGYTRACE) measurements indicate that it should operate 80+ days on a pair of AA batteries.

## Program details ##
The sketch collects the following data from the SENSORS BoosterPack:

- TMP007 Sensor:
     - Die temperature
     - External Temperature
- BME280 Sensor:
     - Temperature
     - Humidity
     - Pressure
- OPT3001 Sensor:
     - Lux

It also collects the following data from the MSP430:

- Die temperature
- Battery voltage (Vcc)

After collecting the sensor data, the data is packaged and transmitted to a receiver hub which can then further process and store the data over time.

While the internal processing of the data uses floating-point math, all data is converted to an integer format before it is packaged and transmitted to the [receiver hub](https://gitlab.com/Andy4495/Sensor-Receiver) as follows:

- **Temperature** is formatted in tenth degrees Fahrenheit. For example, 733 represents 73.3 degrees Fahrenheit
- **Humidity** is formatted as tenth percent relative humidity. For example, 643 represents 64.3 % RH
- **Pressure** is formatted as hundredth inches of Mercury. For example, 3012 represents 30.12 inHg
- **Lux** is formatted as a long integer lux unit.

The compiled sketch currently takes about 21K of program space. Converting all math operations to integer (including operations in the external libraries) could potentially save about 4K of program space.


## External Libraries ##
[Weather Sensors Library](https://github.com/rei-vilo/SensorsWeather_Library) by Rei Vilo

* Sensor_Units.h
* Sensor_TMP007.h
    - Slightly modified to comment out the TMP007\_RESET command sent in Sensor\_TMP007.begin() method.
        - This operation causes the code to hang in my setup.
* Sensor_OPT3001.h
* Sensor_BME280.h

[Calibrated Temp and Vcc Library](https://gitlab.com/Andy4495/mspTandV)

* Used for the internal MSP430 temperature and battery voltage measurements.

## Assembled Weather Sensor ##
- ![Weather Sensor: CC110L BoosterPack(bottom), MSP-EXPF5529LP LaunchPad (middle), and SENSORS BoosterPack (top), powered by 2xAA batteries.](jpg/WeatherSensor.jpg)
