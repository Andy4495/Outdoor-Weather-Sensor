/* -----------------------------------------------------------------
  /* Outdoor Weather Sensor
    10/22/17 - A.T. - Original
    11/03/17 - A.T. - Add internal MSP430 temp and voltage sensing
                    - Add SHT21 temp and humidity sensing
                    - Converted float code to int
                    - Code and comment cleanup
    11/12/17 - A.T. - Removed Object Temperature measurement from
                      TMP006 because it requires floating point
                      calculation.
    11/18/17 - A.T. - Updated to use F5529
    12/20/17 - A.T. - Increase sleep time (and fix data type).
    01/02/18 - A.T. - Major update:
                      - Change to SENSORS BoosterPack instead of SENSORHUB
                      - Embrace floats for measurements, in order to be
                        able to use existing libraries
                      - Remove references to FRAM and circular buffer
                        (previously used to store weather data in NVM).
                      - Remove support for built-in LCD, since code is
                        specific to F5529 LaunchPad

*/
/* -----------------------------------------------------------------

   Design is specific to MSP430F5529, SENSORS BoosterPack, and
   CC110L BoosterPack.

   With minor modifications (e.g. ADC voltage references, LED
   name, TLV/calibration memory locations), this could be modified
   to work with other MSP430 variants.

   Special Pin and Button Operations:
     - Ground Pin 11 for Special Function
       - No function is currently defined
     - PUSH1 and PUSH2 not currently used


   Configuration:
   - Update sleepTime variable to set the number of milliseconds to
     sleep between sensor readings.
   - Define BOARD_LED to the LED name appropriate for the board:
     - For FR6989 and F5529, use #DEFINE BOARD_LED RED_LED
     - For FR4133, use #DEFINE BOARD_LED LED2

   setup()
     I/O and sensor setup
     MSP430 temperature calibration

   loop()
     Read and process individual sensor data
     Send data to receiver hub
   Data collected:                               Units
   - BME280 sensor: Temperature                  F * 10
                    Humidity                     %RH
                    Pressure                     mmHg * 100
   - TMP007 sensor: Die temperature              F * 10
                    External temperature         F * 10
   - OPT3001 sensor: Ambient light               lux
   - MSP430: Internal Temperature                F * 10
   - MSP430: Supply voltage (Vcc)                mV
   - Internal Timing:
                  # of times loop() has run
                  Current value of millis()


  /*
    External libraries:
      Weather Sensors Library by Rei Vilo
        https://github.com/rei-vilo/SensorsWeather_Library
          Sensor_Units.h
          Sensor_TMP007.h
            - Slightly modified to comment out the TMP007_RESET command
              sent in Sensor_TMP007.begin() method.
              This operation causes the code to hang in my setup.
          Sensor_OPT3001.h
          Sensor_BME280.h

*/

// ****** Compile-time Configuration Options ****** //
// If using without CC110L BoosterPack,
// then comment out the following line:
#define ENABLE_RADIO
#define BOARD_LED RED_LED
const unsigned long sleepTime = 55000;
// ************************************************ //



#include "Wire.h"
#include "Sensor_Units.h"
#include "Sensor_TMP007.h"
#include "Sensor_OPT3001.h"
#include "Sensor_BME280.h"

#include <SPI.h>
#include <AIR430BoostFCC.h>

// CC110L Declarations
#define ADDRESS_LOCAL   0x02    // This device
#define ADDRESS_REMOTE  0x01    // Receiver hub

struct sPacket
{
  uint8_t from;           // Local node address that message originated from
  uint8_t message[59];    // Local node message [MAX. 59 bytes]
};

struct sPacket txPacket;

// SENSORS I2C Addressing (7-bit)
/* Note that the I2C addresses are all hardcoded in the Weather Sensors Library
  // and are only listed here for reference
  const uint8_t TMP007_addr   = 0x40;
  const uint8_t BME280_addr   = 0x77;
  const uint8_t OPT3001_addr  = 0x47;
*/


int* Cal30;   // MSP chip Temp calibration @ 30C
int* Cal85;   // MSP chip Temp calibration @ 85C
float Tc;     // Tc coeffecient calculated from calibration values

Sensor_TMP007 myTMP007;
float TMP007_internalTemperature, TMP007_externalTemperature;
Sensor_OPT3001 myOPT3001;
float OPT3001_light;
Sensor_BME280 myBME280;
float BME280_pressure, BME280_temperature, BME280_humidity;

struct WeatherData {
  int             BME280_T;  // Tenth degrees F
  unsigned int    BME280_P;  // Pressure in inches of Hg * 100
  int             BME280_H;  // % Relative Humidity
  int             TMP107_Ti; // Tenth degrees F
  int             TMP107_Te; // Tenth degrees F
  unsigned int    LUX;       // Limited in code to 65535
  int             MSP_T;     // Tenth degrees F
  unsigned int    Batt_mV;   // milliVolts
  unsigned int    Loops;
  unsigned long   Millis;
};


WeatherData weatherdata;

volatile unsigned char bootCount;

unsigned int loopCount = 0;

int            ADCraw;
float          msp430T;
float          msp430mV;
float          P;

/* Use these definitions if Pin 11 Special Function is implemented
  boolean        SpecialFunctionEnable;
  const int      SpecialFunctionPin = 11;
*/

void setup() {

  /* Uncomment to enable special function with Pin 11
    pinMode(SpecialFunction, INPUT_PULLUP);  // Ground Pin 11 to enable Special Function
    if (digitalRead(SpecialFunctionPin)) {
      SpecialFunctionEnable = false;
    }
    else {
      SpecialFunctionEnable = true;
    }
  */

  Serial.begin(9600);
  Serial.println(F("Reset"));

  // CC110L Setup
  txPacket.from = ADDRESS_LOCAL;
  memset(txPacket.message, 0, sizeof(txPacket.message));
#ifdef ENABLE_RADIO
  Serial.println(F("Radio Enabled"));
  Radio.begin(ADDRESS_LOCAL, CHANNEL_1, POWER_MAX);
#endif

  Wire.begin();            // initialize I2C that connects to sensor
  Serial.println(F("I2C Initialized"));

  myTMP007.begin();
  Serial.println(F("TMP007 Initialized"));
  myTMP007.get();
  myOPT3001.begin();
  Serial.println(F("OPT3001 Initialized"));
  myOPT3001.get();
  myBME280.begin();
  Serial.println(F("BME280 Initialized"));
  myBME280.get();

  // If enabling a special function for PUSH1 or PUSH2, then configure INPUT_PULLUP
  // Otherwise, leave default setup as INPUT to save a little power.
  //  pinMode(PUSH1, INPUT_PULLUP);
  //  pinMode(PUSH2, INPUT_PULLUP);

  digitalWrite(BOARD_LED, LOW);
  pinMode(BOARD_LED, OUTPUT); 

  // Set all structure values to zero on startup
  weatherdata.BME280_T = 0;
  weatherdata.BME280_P = 0;
  weatherdata.BME280_H = 0;
  weatherdata.TMP107_Ti = 0;
  weatherdata.TMP107_Te = 0;
  weatherdata.LUX = 0;
  weatherdata.MSP_T = 0;
  weatherdata.Batt_mV = 0;
  weatherdata.Loops = 0;
  weatherdata.Millis = 0;

  // Calculate the internal Temp Coeffecient using factory calibration values
  // See MSP430F5529 Family Guide Sections 28.2.8 and Fig 28-2, and
  // MS430FR6989 User Guide Section 6.11
  Cal30 = (int*) 0x1a1a;
  Cal85 = (int*) 0x1a1c;
  Tc = 55.0 / (float)(*Cal85 - *Cal30);

  // Flash the LED to indicate we started
  digitalWrite(BOARD_LED, HIGH);
  delay(500);
  digitalWrite(BOARD_LED, LOW);
}


void loop() {

  loopCount++;

  // TMP007 sensor
  Serial.println("TMP007");
  myTMP007.get();
  TMP007_internalTemperature = conversion(myTMP007.internalTemperature(), KELVIN, FAHRENHEIT);
  TMP007_externalTemperature = conversion(myTMP007.externalTemperature(), KELVIN, FAHRENHEIT);
  Serial.print("  Internal: ");
  Serial.print(TMP007_internalTemperature);
  Serial.println(" F");
  Serial.print("  External: ");
  Serial.print(TMP007_externalTemperature);
  Serial.println(" F");

  // OPT3001 sensor
  Serial.println("OPT3001");
  myOPT3001.get();
  OPT3001_light = myOPT3001.light();
  Serial.print("  Ambient Light: ");
  Serial.print(OPT3001_light);
  Serial.println(" lux");

  // BME280 sensor
  Serial.println("BME280");
  myBME280.get();
  BME280_pressure = myBME280.pressure();
  BME280_temperature = conversion(myBME280.temperature(), KELVIN, FAHRENHEIT);
  BME280_humidity = myBME280.humidity();
  Serial.print("  Pressure: ");
  Serial.print(BME280_pressure);
  Serial.print(" hPa, ");
  /// *** Need to convert to mmHg
  P = BME280_pressure / 33.863886667;    // inches Hg = Pa / 33.863886667
  Serial.print(P);
  Serial.println(" inHg");
  Serial.print("  Temperature: ");
  Serial.print(BME280_temperature);
  Serial.println(" F");
  Serial.print("  Humidity: ");
  Serial.print(BME280_humidity);
  Serial.println(" %");

  // MSP430 internal temp sensor
  Serial.println("MSP430");
  analogReference(INTERNAL1V5);
  ADCraw = analogRead(TEMPSENSOR);
  ADCraw = analogRead(TEMPSENSOR);
  // Adjust temp reading with calibration factor and convert to degrees F
  msp430T = (Tc * (ADCraw - (*Cal30)) + 30.0L) * 1.8 + 32.0;
  Serial.print("  Temperature: ");
  Serial.print(msp430T);
  Serial.println(" F");

  // MSP430 battery voltage (Vcc) calibrated against reference
  // Start with 2.0V reference, which requires Vcc >= 2.3 V
  // Once Vcc is below 2.8V, then switch to 1.5V reference.
  // Vcc = (4096 * 1.2 V) / 1.2-V reference ADC result
  // Internal 1.5V reference is on ADC channel 13
  analogReference(INTERNAL2V0);
  analogRead(A11);
  ADCraw = analogRead(A11);
  // Need calculation to be Long int due to mV scaling
  msp430mV = ADCraw * 4.0 / 4095.0;
  // Use 1.5V reference if Vcc < 2.8V
  if (msp430mV < 2.8) {
    analogReference(INTERNAL1V5);
    ADCraw = analogRead(A11);
    msp430mV = ADCraw * 3.0 / 4095.0;
  }
  Serial.print("  Battery Voltage: ");
  Serial.print(msp430mV, 3);
  Serial.println(" V");

  weatherdata.BME280_T = (BME280_temperature + 0.05) * 10.0;
  weatherdata.BME280_P = (P + 0.005) * 100.0;
  weatherdata.BME280_H = (BME280_humidity + 0.05) * 10.0;
  weatherdata.TMP107_Ti = (TMP007_internalTemperature + 0.05) * 10.0;
  weatherdata.TMP107_Te = (TMP007_externalTemperature + 0.05) * 10.0;
  if (OPT3001_light > 65534.5)
    weatherdata.LUX = 65535;
  else
    weatherdata.LUX = OPT3001_light;
  weatherdata.MSP_T = (msp430T + 0.05) * 10.0;
  weatherdata.Batt_mV = (msp430mV + 0.0005) * 1000.0;
  weatherdata.Loops = loopCount;
  weatherdata.Millis = millis();

  // Send the data over-the-air
  memcpy(&txPacket.message, &weatherdata, sizeof(WeatherData));
#ifdef ENABLE_RADIO
  Radio.transmit(ADDRESS_REMOTE, (unsigned char*)&txPacket, sizeof(WeatherData) + 4);
  Serial.print("Tx 'From' address: ");
  Serial.println(txPacket.from);
#endif

  Serial.println(weatherdata.BME280_T);
  Serial.println(weatherdata.BME280_P);
  Serial.println(weatherdata.BME280_H);
  Serial.println(weatherdata.TMP107_Ti);
  Serial.println(weatherdata.TMP107_Te);
  Serial.println(weatherdata.LUX);
  Serial.println(weatherdata.MSP_T);
  Serial.println(weatherdata.Batt_mV);
  Serial.println(weatherdata.Loops);
  Serial.println(weatherdata.Millis);
  Serial.println(F("--"));

  sleep(sleepTime);
}

