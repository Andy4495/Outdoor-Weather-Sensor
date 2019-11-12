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
   - BME280 sensor: Temperature                  F * 10
                    Humidity                     0.1%RH
                    Pressure                     mmHg * 100
   - TMP007 sensor: Die temperature              F * 10
                    External temperature         F * 10
   - OPT3001 sensor: Ambient light               lux
   - MSP430: Internal Temperature                F * 10
   - MSP430: Supply voltage (Vcc)                mV
   - Internal Timing:
                  # of times loop() has run
                  Current value of millis()

    External libraries:
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

#include "SWI2C.h"
#include "MspTandV.h"
#include "sensor_definitions.h"

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
  int             TMP107_Ti; // Tenth degrees F
  int             TMP107_Te; // Tenth degrees F
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

SWI2C myTMP007(SDA_PIN, SCL_PIN, TMP007_ADDRESS);
SWI2C myOPT3001(SDA_PIN, SCL_PIN, OPT3001_ADDRESS);
SWI2C myBME280(SDA_PIN, SCL_PIN, BME280_ADDRESS);

int TMP007T_Internal;
int TMP007T_External;

unsigned long OPT3001Lux;

uint8_t  BME280RawData[8];
int32_t  rawBME280T, rawBME280P, rawBME280H;
int32_t  BME280T;
int32_t  BME280TF;
uint32_t BME280P;
int32_t  BME280PinHg;
uint32_t BME280H;
int32_t  t_fine;    // Used to carry over the Temp value to H and P calculations

// BME280 Calibration values
uint16_t dig_T1;
int16_t  dig_T2;
int16_t  dig_T3;

uint16_t dig_P1;
int16_t  dig_P2;
int16_t  dig_P3;
int16_t  dig_P4;
int16_t  dig_P5;
int16_t  dig_P6;
int16_t  dig_P7;
int16_t  dig_P8;
int16_t  dig_P9;

uint8_t  dig_H1;
int16_t  dig_H2;
uint8_t  dig_H3;
int16_t  dig_H4;
int16_t  dig_H5;
int8_t   dig_H6;

MspTemp msp430Temp;
MspVcc  msp430Vcc;

unsigned int loopCount = 0;

int            msp430T;
int            msp430mV;

void setup() {
  uint16_t data16;
  uint8_t  data8;

  Serial.begin(9600);
  ///  Serial.println(F("Reset"));

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

  TMP007_startup();
  OPT3001_startup();
  BME280_startup();

  /*
    // Verify device IDs
    myTMP007.read2bFromRegisterMSBFirst(TMP007_DEVICE_ID, &data16);
    Serial.print("TMP007 DevID: 0x");
    Serial.println(data16, HEX);

    myOPT3001.read2bFromRegisterMSBFirst(OPT3001_MANUFACTURE_ID_REGISTER, &data16);
    Serial.print("OPT3001 ManuID: 0x");
    Serial.println(data16, HEX);

    myOPT3001.read2bFromRegisterMSBFirst(OPT3001_DEVICE_ID_REGISTER, &data16);
    Serial.print("OPT3001 DevID: 0x");
    Serial.println(data16, HEX);

    Serial.print("BME280 Chip ID: 0x");
    myBME280.read1bFromRegister(BME280_ID, &data8);
    Serial.println(data8, HEX);

    Serial.println("--");
  */

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

  TMP007_get();

  Serial.print("TMP007 Int (0.1 C): ");
  Serial.println(TMP007T_Internal);
  Serial.print("TMP007 Int (0.1 F): ");
  TMP007T_Internal = (TMP007T_Internal * 9) / 5 + 320;
  Serial.println(TMP007T_Internal);

  Serial.print("TMP007 Ext (0.1 C): ");
  Serial.println(TMP007T_External);
  Serial.print("TMP007 Ext (0.1 F): ");
  TMP007T_External = (TMP007T_External * 9) / 5 + 320;
  Serial.println(TMP007T_External);

  OPT3001_get();

  Serial.print("OTP3001 Lux: ");
  Serial.println(OPT3001Lux);


  BME280_get();
  BME280T = calculate_BME280_T();
  BME280P = calculate_BME280_P();
  BME280H = calculate_BME280_H();
  Serial.print("BME280 T (0.01 C): ");
  Serial.println(BME280T);
  Serial.print("BME280 T (F): ");
  BME280TF = (BME280T * 9) / 50 + 320;
  Serial.print(BME280TF / 10);
  Serial.print(".");
  Serial.println(BME280TF % 10);
  Serial.print("BME280 P (Pa): ");
  Serial.println(BME280P);
  Serial.print("BME280 P (inHG): ");
  BME280PinHg = BME280P * 100 / 3386;
  Serial.print(BME280PinHg / 100);
  Serial.print(".");
  Serial.println(BME280PinHg % 100);
  Serial.print("BME280 H (0.1%RH): ");
  Serial.println(BME280H);

  // MSP430 internal temp sensor
  Serial.println("MSP430");
  msp430Temp.read(CAL_ONLY);   // Only get the calibrated reading
  msp430T = msp430Temp.getTempCalibratedF();
  Serial.print("  Temp: ");
  Serial.print(msp430T / 10);
  Serial.print(".");
  Serial.print(msp430T % 10);
  Serial.println(" F");

  // MSP430 battery voltage (Vcc)
  msp430Vcc.read(CAL_ONLY);    // Only get the calibrated reading
  msp430mV = msp430Vcc.getVccCalibrated();

  Serial.print("  Batt: ");
  Serial.print(msp430mV);
  Serial.println(" mV");

  txPacket.weatherdata.BME280_T  = BME280TF;
  txPacket.weatherdata.BME280_P  = BME280PinHg;
  txPacket.weatherdata.BME280_H  = BME280H;
  txPacket.weatherdata.TMP107_Ti = TMP007T_Internal;
  txPacket.weatherdata.TMP107_Te = TMP007T_External;
  txPacket.weatherdata.LUX       = OPT3001Lux;
  txPacket.weatherdata.MSP_T     = msp430T;
  txPacket.weatherdata.Batt_mV   = msp430mV;
  txPacket.weatherdata.Loops     = loopCount;
  txPacket.weatherdata.Millis    = millis();

#ifdef ENABLE_RADIO
  Radio.transmit(ADDRESS_REMOTE, (unsigned char*)&txPacket, sizeof(WeatherData) + 4);
  Serial.print("Tx 'From' address: ");
  Serial.println(txPacket.from);
#endif

  Serial.println("Pkt Data:");
  Serial.println(txPacket.weatherdata.BME280_T);
  Serial.println(txPacket.weatherdata.BME280_P);
  Serial.println(txPacket.weatherdata.BME280_H);
  Serial.println(txPacket.weatherdata.TMP107_Ti);
  Serial.println(txPacket.weatherdata.TMP107_Te);
  Serial.println(txPacket.weatherdata.LUX);
  Serial.println(txPacket.weatherdata.MSP_T);
  Serial.println(txPacket.weatherdata.Batt_mV);
  Serial.println(txPacket.weatherdata.Loops);
  Serial.println(txPacket.weatherdata.Millis);
  Serial.println(F("--"));

  sleepSeconds(sleepTime);
}
