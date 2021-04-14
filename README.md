Battery Powered Outdoor Weather Sensor
======================================

This project is a battery-powered wireless outdoor weather sensor. It is designed to run on an [MSP-EXP430G2 LaunchPad][9], [430BOOST-CC110L BoosterPack][11], and [BOOSTXL-SENSORS BoosterPack][10].

Data from the TMP007, BME280, and OPT3001 sensors are periodically sent to a wireless [receiver hub][2] using the CC110L BoosterPack.

The project is designed for low-power operation. My current implementation is still running on a pair of AA batteries for over 18 months. The limiting factor in the life of the batteries is the TMP007 sensor, which is specified to operate down to 2.5 V. The MSP430 (running at 8 MHz) will work down to 2.2 V, the BME280 sensor down to 1.8 V, and the OPT3001 sensor down to 1.6 V.

## Updated Design ##

Previous versions of the weather sensor were implemented for an MSP-EXP430F5529LP LaunchPad and used Rei Vilo's [Weather Sensors Library][8]. However, that design required new batteries every two months, so I looked for ways to cut power usage and improve the design.

The new design uses an MSP430G2553 processor running at 8 MHz.

By moving to a G2553 controller, I had to switch to a software I2C implementation, since the G2 processors share hardware I2C with the hardware SPI signals. So I created my own [Software I2C Weather Sensors library][1].

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
     - Lignt Intensity

It also collects the following data from the MSP430:

- Die temperature
- Battery voltage (Vcc)

After collecting the sensor data, the data is packaged and transmitted to a receiver hub which can then further process and store the data over time.

All data is processed using integer math and is transmitted to the [receiver hub][2] as follows:

- **Temperature** is formatted in tenth degrees Fahrenheit. For example, 733 represents 73.3 degrees Fahrenheit
- **Humidity** is formatted as tenth percent relative humidity. For example, 643 represents 64.3 % RH
- **Pressure** is formatted as hundredth inches of Mercury. For example, 3012 represents 30.12 inHg
- **Light Intensity** is formatted as a long integer lux unit.

The compiled sketch currently takes about 15K of program space, and therefore fits on a G2553 controler. Further code reductions could be made by removing the `Serial.print` operations, which are currently only used for debugging.

Note that for low-voltage MSP430G2553 operation, it is necessary to program the processor to run at 8 MHz instead of the standard 16 MHz.

## External Libraries ##
* [Weather Sensors][1] - Weather Sensors interface using software I2C.
* [SWI2C][3] - Software I2C implementation, used by the Wether Sensors library.
* [Calibrated Temp and Vcc Library][4] - Internal MSP430 temperature and battery voltage measurements.

## References ##
* [TMP007][5] Temperature Sensor.
* [OPT3001][6] Amient Light Sensor.
* [BME280][7] Temperature, Humidity, and Pressure Sensor.

## Assembled Weather Sensor ##
- ![Weather Sensor: CC110L BoosterPack(bottom), MSP-EXPF5529LP LaunchPad (middle), and SENSORS BoosterPack (top), powered by 2xAA batteries.](jpg/WeatherSensor.jpg)
Although pictured here with an F5529 LauchPad, the current implementation uses an MSP-EXP430G2 LaunchPad.

## License ##
The software and other files in this repository are released under what is commonly called the [MIT License][100]. See the file [`LICENSE.txt`][101] in this repository.

[1]: https://github.com/Andy4495/Weather_Sensors_SWI2C
[2]: https://github.com/Andy4495/Wireless-Sensor-Receiver-Hub
[3]: https://github.com/Andy4495/SWI2C
[4]: https://github.com/Andy4495/MspTandV
[5]: https://cdn-shop.adafruit.com/datasheets/tmp007.pdf
[6]: http://www.ti.com/lit/ds/symlink/opt3001.pdf
[7]: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
[8]: https://github.com/rei-vilo/SensorsWeather_Library
[9]: https://www.ti.com/tool/MSP-EXP430G2ET
[10]: https://www.ti.com/tool/BOOSTXL-SENSORS
[11]: http://www.ti.com/lit/ml/swru312b/swru312b.pdf
[100]: https://choosealicense.com/licenses/mit/
[101]: ./LICENSE.txt
