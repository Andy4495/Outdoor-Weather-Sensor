/* -----------------------------------------------------------------
    Outdoor Weather Sensor
    https://github.com/Andy4495/Outdoor-Weather-Sensor

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
    01/17/18 - A.T. - Use MspTandV library
                    - Change lux measurement from int to long.
    02/01/18 - A.T. - Update message structure to align on word boundary.
    03/12/18 - A.T. - Changed TX to Channel 4 to allow use of repeater
                      Default is Chananel 4.
                      Press and hold PUSH1/S1/P2.1 during reset to
                      change to Channel 1.
    10/20/18 - A.T. - Use Software I2C (SWI2C) for comms with SENSORS BoosterPack.
                      Implement custom interface to TMP007, OPT3001, BME280
                      sensors.
                      Cut down code size to fit on MSP430G2553.
                      Go back to integer-only math.
                      Use sleepSeconds(), so sleep time is now in seconds, not ms
    11/10/18 - A.T. - Fix TMP007 temp calculations in "sensor_functions.h" (signed
                      value, so don't clear the sign bit)
    05/19/20 - A.T. - Sensor interfacing moved to new Weather_Sensors_SWI2C library.
    08/16/20 - A.T. - Re-arrange calls to library to optimize code size.
                      Comment out code that prints out packet data; uncomment if needed for debugging. 

*/
/* -----------------------------------------------------------------

   Design is specific to MSP430G2553, SENSORS BoosterPack, and
   CC110L BoosterPack.

   With minor modifications (e.g. ADC voltage references, LED
   name, TLV/calibration memory locations), this could be modified
   to work with other MSP430 variants.

   Configuration:
   - Update sleepTime variable to set the number of milliseconds to
     sleep between sensor readings.
   - Define BOARD_LED to the LED name appropriate for the board:
     - For FR6989 and F5529, use #DEFINE BOARD_LED RED_LED
     - For FR4133, use #DEFINE BOARD_LED LED2
     - For G2553, Do not define BOARD_LED

   setup()
     I/O and sensor setup
     MSP430 temperature calibration

   loop()
     Read and process individual sensor data
     Send data to receiver hub
   Data collected:                               Units
   - BME280 sensor: Temperature                  0.1 degrees F
                    Humidity                     0.1 %RH
                    Pressure                     0.01 mmHg
   - TMP007 sensor: Die temperature              0.1 degrees F
                    External temperature         0.1 degrees F
   - OPT3001 sensor: Ambient light               lux
   - MSP430: Internal Temperature                0.1 degrees F
   - MSP430: Supply voltage (Vcc)                mV
   - Internal Timing:
                  # of times loop() has run
                  Current value of millis()

    External libraries:
      Weather_Sensors_SWI2C
         https://github.com/Andy4495/Weather_Sensors_SWI2C
      Software I2C "SWI2C"
         https://github.com/Andy4495/SWI2C
      Calibrated Temp and Vcc library "MspTandV"
         https://github.com/Andy4495/MspTandV

*/

// ****** Compile-time Configuration Options ****** //
// If using without CC110L BoosterPack,
// then comment out the following line:
#define ENABLE_RADIO
// G2553 LED conflict with CC110L BoosterPack, so don't use
//#define BOARD_LED RED_LED
const unsigned long sleepTime = 55;       // Seconds, not ms
// ************************************************ //

#include "Weather_Sensors_SWI2C.h"
#include "MspTandV.h"

#include <SPI.h>
#include <AIR430BoostFCC.h>

// CC110L Declarations
#define ADDRESS_LOCAL   0x02    // This device
#define ADDRESS_REMOTE  0x01    // Receiver hub
channel_t txChannel = CHANNEL_4;        // Can be changed with PUSH2

enum {WEATHER_STRUCT, TEMP_STRUCT};

struct WeatherData {
  int             BME280_T;  // Tenth degrees F
  unsigned int    BME280_P;  // Pressure in inches of Hg * 100
  int             BME280_H;  // % Relative Humidity
  int             TMP007_Ti; // Tenth degrees F
  int             TMP007_Te; // Tenth degrees F
  unsigned long   LUX;       // Lux units
  int             MSP_T;     // Tenth degrees F
  unsigned int    Batt_mV;   // milliVolts
  unsigned int    Loops;
  unsigned long   Millis;
};

struct sPacket
{
  uint8_t from;           // Local node address that message originated from
  uint8_t struct_type;    // Flag to indicate type of message structure
  union {
    uint8_t message[58];     // Local node message keep even word boundary
    WeatherData weatherdata;
  };
};

struct sPacket txPacket;

#define SDA_PIN                         10
#define SCL_PIN                         9
#define TMP007_ADDRESS                  0x40
#define OPT3001_ADDRESS                 0x47
#define BME280_ADDRESS                  0x77

TMP007_SWI2C  myTMP007(SDA_PIN, SCL_PIN, TMP007_ADDRESS);
OPT3001_SWI2C myOPT3001(SDA_PIN, SCL_PIN, OPT3001_ADDRESS);
BME280_SWI2C  myBME280(SDA_PIN, SCL_PIN, BME280_ADDRESS);

MspTemp msp430Temp;
MspVcc  msp430Vcc;

unsigned int loopCount = 0;

void setup() {

  Serial.begin(9600);

  pinMode(PUSH2, INPUT_PULLUP);                // Used to select TX channel

#ifdef BOARD_LED
  digitalWrite(BOARD_LED, LOW);
  pinMode(BOARD_LED, OUTPUT);
#endif

  // If PUSH2 pressed during reset, then use CH1. Otherwise, use default (CH4).
  if ( digitalRead(PUSH2) == LOW) txChannel = CHANNEL_1;

  if (txChannel == CHANNEL_1) Serial.print("TX Channel: CHANNEL_1 (");
  else Serial.println("TX Channel: CHANNEL_4");

  // CC110L Setup
  txPacket.from = ADDRESS_LOCAL;
  txPacket.struct_type = WEATHER_STRUCT;
  memset(txPacket.message, 0, sizeof(txPacket.message));
#ifdef ENABLE_RADIO
  Serial.println(F("Radio Enabled"));
  Radio.begin(ADDRESS_LOCAL, txChannel, POWER_MAX);
#endif

  myTMP007.begin();
  myOPT3001.begin();
  myBME280.begin();

#ifdef BOARD_LED
  // Flash the LED to indicate we started
  // Number of flashes indicates TX channel number
  int flashes = 4;
  if (txChannel == CHANNEL_1) flashes = 1;
  for (int i = 0; i < flashes; i++) {
    digitalWrite(BOARD_LED, HIGH);
    delay(350);
    digitalWrite(BOARD_LED, LOW);
    delay(350);
  }
#endif
}


void loop() {

  loopCount++;

  myTMP007.readSensor();
  myOPT3001.readSensor();
  myBME280.readSensor();
  msp430Temp.read(CAL_ONLY);   // Only get the calibrated reading
  msp430Vcc.read(CAL_ONLY);    // Only get the calibrated reading

  txPacket.weatherdata.BME280_T  = myBME280.getTempF();
  txPacket.weatherdata.BME280_P  = myBME280.getPressureInHg();
  txPacket.weatherdata.BME280_H  = myBME280.getRH();
  txPacket.weatherdata.TMP007_Ti = myTMP007.getIntTempF();
  txPacket.weatherdata.TMP007_Te = myTMP007.getExtTempF();
  txPacket.weatherdata.LUX       = myOPT3001.getLux();
  txPacket.weatherdata.MSP_T     = msp430Temp.getTempCalibratedF();
  txPacket.weatherdata.Batt_mV   = msp430Vcc.getVccCalibrated();
  txPacket.weatherdata.Loops     = loopCount;
  txPacket.weatherdata.Millis    = millis();

  Serial.print("TMP007 Int (0.1 C): ");
  Serial.println(myTMP007.getIntTempC());
  Serial.print("TMP007 Int (0.1 F): ");
  Serial.println(txPacket.weatherdata.TMP007_Ti);

  Serial.print("TMP007 Ext (0.1 C): ");
  Serial.println(myTMP007.getExtTempC());
  Serial.print("TMP007 Ext (0.1 F): ");
  Serial.println(txPacket.weatherdata.TMP007_Te);

  Serial.print("OTP3001 Lux: ");
  Serial.println(txPacket.weatherdata.LUX);

  Serial.print("BME280 T (0.01 C): ");
  Serial.println(myBME280.getTempC());
  Serial.print("BME280 T (0.1 F): ");
  Serial.println(txPacket.weatherdata.BME280_T);
  Serial.print("BME280 P (Pa): ");
  Serial.println(myBME280.getPressurePa());
  Serial.print("BME280 P (0.01 inHG): ");
  Serial.println(txPacket.weatherdata.BME280_P);
  Serial.print("BME280 H (0.1%RH): ");
  Serial.println(txPacket.weatherdata.BME280_H);

  // MSP430 internal temp sensor
  Serial.println("MSP430");
  Serial.print("  Temp (0.1 F): ");
  Serial.println(txPacket.weatherdata.MSP_T);

  // MSP430 battery voltage (Vcc)
  Serial.print("  Batt (mV): ");
  Serial.println(txPacket.weatherdata.Batt_mV);

#ifdef ENABLE_RADIO
  Radio.transmit(ADDRESS_REMOTE, (unsigned char*)&txPacket, sizeof(WeatherData) + 4);
  Serial.print("Tx 'From' address: ");
  Serial.println(txPacket.from);
#endif

/* Uncomment for more debugging data. Note that this info is already printed above. 
  Serial.println("Pkt Data:");
  Serial.println(txPacket.weatherdata.BME280_T);
  Serial.println(txPacket.weatherdata.BME280_P);
  Serial.println(txPacket.weatherdata.BME280_H);
  Serial.println(txPacket.weatherdata.TMP007_Ti);
  Serial.println(txPacket.weatherdata.TMP007_Te);
  Serial.println(txPacket.weatherdata.LUX);
  Serial.println(txPacket.weatherdata.MSP_T);
  Serial.println(txPacket.weatherdata.Batt_mV);
  Serial.println(txPacket.weatherdata.Loops);
  Serial.println(txPacket.weatherdata.Millis);
  Serial.println(F("--"));
*/

  sleepSeconds(sleepTime);
}
