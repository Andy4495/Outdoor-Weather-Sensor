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


*/
/* -----------------------------------------------------------------
   General code structure:
   Design is specific to MSP430FR6989 and SensorHub BoosterPack
   Special Operations:
     - Ground Pin 11 for Special Function
       - No function is currently defined
     - Press PUSH1 during reset to initialize Info FRAM
     - Press PUSH2 at reset to flash RED_LED to confirm operation
     - Press PUSH2 at end of measurement cycle to dump contents of
       Info FRAM to Serial.

   setup()
     Initialize Information FRAM if necessary
         User may also press PUSH1 during reset to initialize FRAM
     I/O and sensor setup
     Temperature calibration

   loop()
     Sensor readings stored in circular buffer in Info FRAM
     Connect to server every 15 minutes to upload data
         *** Still need to implement
   Data collected:                               Units
   - BMP180 sensor: Temperature                  F * 10
                    Pressure                     mmHG * 100
   - TMP006 sensor: Die temp and external temp   F * 10
   - ISL29023 sensor: Ambient light              lux
   - SHT21 sensor: Humidity and temperature (*** not implemented)
   - MSP430: Internal Temperature                F * 10
   - MSP430: Supply voltage (Vcc)                mV
   - Internal Timing:
                  # of times loop() has run
                  Current value of millis()
                  # of resets since FRAM initialized

*/

/*
    External and modified libraries:
    BMP180: https://github.com/astuder/BMP085-template-library-Energia
    - Library uses integer-only math
    TMP006: https://github.com/adafruit/Adafruit_TMP006
    - Modified to use integer-only math and renamed Adafruit_TMP006_INT
    ISL29023: https://github.com/perelloc/isl29023
    - library slightly updated to fix incorrect comment blocking
      which was causing compilation to fail.
    - Modified to use integer-only math and renamed isl29023_INT
    SHT21: https://github.com/e-radionicacom/SHT21-Arduino-Library
    - Modified to use integer-only math and renamed SHT21_INT

*/

#include <Wire.h>
#include <BMP085_t.h>         // Used for BMP180
#include <Adafruit_TMP006_INT.h>
#include <isl29023_INT.h>
#include <SHT21_INT.h>
#include <SPI.h>
#include <AIR430BoostFCC.h>

//#define LCD_ENABLED
#ifdef LCD_ENABLED
#include "LCD_Launchpad.h"
LCD_LAUNCHPAD myLCD;
#endif

// CC110L Declarations
#define ADDRESS_LOCAL   0x02
#define ADDRESS_REMOTE  0x01

struct sPacket
{
  uint8_t from;           // Local node address that message originated from
  uint8_t message[59];    // Local node message [MAX. 59 bytes]
};

struct sPacket txPacket;

//   SENSORHUB I2C Addressing (7-bit)
const uint8_t MPU9150_addr  = 0x68; // Sensor not used
const uint8_t BMP180_addr   = 0x77;
const uint8_t SHT21_addr    = 0x40;
const uint8_t ISL29023_addr = 0x44;
const uint8_t TMP006_addr   = 0x41;

/*
  BMP085<oversampling, eocpin, tempunit, i2caddress>
  i2caddress default is 0x77
*/
BMP085<0, 0, BMP085_F> PSensor;         // Return temp in F

Adafruit_TMP006 tmp006(TMP006_addr);

ISL29023_INT isl = ISL29023_INT();    // Library default address is 0x44

// Default I2C address is 0x40
SHT21_INT mysht;

int* Cal30;   // MSP chip Temp calibration @ 30C
int* Cal85;   // MSP chip Temp calibration @ 85C
long Tc;     // Tc coeffecient calculated from calibration values
// Calculated as "times 10,000" to avoid floating math

// This code is specific to the MSP430FR6989 Memory map
// The FR4133 has same Info RAM memory map
// 512 bytes @ 0x1800 - 0x19FF
// Check the datasheets for other processors
#define infoFRAM 0x1800

const char FlashMarker[] = "WS-430";
const int markerLength = sizeof(FlashMarker);

struct FlashData {
  char marker[markerLength];
  unsigned char numBoots;
  unsigned char dataIndex;
  char Padding[15];
};

FlashData flash;

struct WeatherData {
  int             BMP180_T;  // Tenth degrees F
  unsigned int    BMP180_P;  // Pressure in inches of Hg * 100
  int             TMP106_Ti; // Tenth degrees F
  unsigned int    LUX;       // Limited in code to 65535
  int             SHT_T;     // Tenth degrees F
  int             SHT_H;     // Tenth % Relative Humidity * 10
  int             MSP_T;     // Tenth degrees F
  unsigned int    Batt_mV;   // milliVolts
  unsigned int    Loops;
  unsigned long   Millis;
  unsigned int    Resets;
  //  char            Padding[2];
};

const int maxWeatherRecords = 21;    // 21 * 24 = 504 < 512

WeatherData weatherdata[maxWeatherRecords];
unsigned int weatherindex;

volatile unsigned char bootCount;
volatile unsigned char volIndex;

unsigned int loopCount = 0;
const int sleepTime = 30000;

int            TempF;
unsigned int   P;
int            Ti;
unsigned int   lux;
int            ADCraw;
double         msp430T;
unsigned long  msp430mV;
int            shtT;
int            shtH;

boolean        SpecialFunctionEnable;
const int      SpecialFunctionPin = 11;

void setup() {

  boolean firstTime = false;

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

  /// For lowest power operation, set all unused and un-tied pins to OUTPUT

#ifdef LCD_ENABLED
  myLCD.init();
  myLCD.clear();
  myLCD.displayText(F("HELLO"));
#endif

  // CC110L Setup
  txPacket.from = ADDRESS_LOCAL;
  memset(txPacket.message, 0, sizeof(txPacket.message));
  Radio.begin(ADDRESS_LOCAL, CHANNEL_1, POWER_MAX);

  // SENSORHUB Setup
  // When plugging in SENSORHUB directly to BoosterPack pins, use Module 0
  // When also using CC110L BoosterPack, need to manually wire SENSORHUB
  // to I2C bus 1 (pins 9/SCL and 10/SDA).
  Wire.setModule(1);
  Wire.begin();            // initialize I2C that connects to sensor

  PSensor.begin();         // initalize BMP180 pressure sensor. Use default i2c address of 0x77.

  int r2 =  tmp006.begin();
  Serial.print("tmp006 init status: ");
  Serial.println(r2);

  isl.init();             // Initialize the ISL29023 sensor

  mysht.reset();          // reset the SHT21 sensor

  // Press PUSH1 on reset to clear the reset count
  pinMode(PUSH1, INPUT_PULLUP);
  // PUSH2 operation:
  //   Press PUSH2 at reset to flash RED_LED: indicate device booted and operating
  //   If Serial Enabled, press PUSH2 at end of measurement cycle to
  //   dump Info FRAM to serial
  pinMode(PUSH2, INPUT_PULLUP);

  digitalWrite(RED_LED, LOW);
  pinMode(RED_LED, OUTPUT);  // Turn off LED for lower power

  // Initialize memory
  flash.numBoots = 0;
  flash.dataIndex = 0;

  for (int i=0; i<maxWeatherRecords; i++) {
    weatherdata[i].BMP180_T = 0;
    weatherdata[i].BMP180_P = 0;
    weatherdata[i].TMP106_Ti = 0;
    weatherdata[i].LUX = 0;
    weatherdata[i].SHT_T = 0;
    weatherdata[i].SHT_H = 0;
    weatherdata[i].MSP_T = 0;
    weatherdata[i].Batt_mV = 0;
    weatherdata[i].Loops = 0;
    weatherdata[i].Millis = 0;
    weatherdata[i].Resets = 0;
  }
  
    firstTime = false;
  bootCount = 0;

  if (firstTime)
  {
    Serial.println(F("FRAM erase, count reset"));
  }

  /*
    //DEBUG to confirm structure size changes
      WeatherData instance;
      Serial.print(F("Weather Data struct size = "));
      Serial.println(sizeof(instance));

      FlashData in2;
      Serial.print(F("Flash Data struct size = "));
      Serial.println(sizeof(in2));
  */

  // Calculate the internal Temp Coeffecient using factory calibration values
  // See MSP430F5529 Family Guide Sections 28.2.8 and Fig 28-2, and 
  // MS430FR6989 User Guide Section 6.11
  Cal30 = (int*) 0x1a1a;
  Cal85 = (int*) 0x1a1c;
  // Scale the calibration factory by 10,000 to avoid floating point math
  Tc = 550000L / (*Cal85 - *Cal30);

  if (!digitalRead(PUSH2)) {
    digitalWrite(RED_LED, HIGH);
    delay(500);
    digitalWrite(RED_LED, LOW);
  }
}


void loop() {

  uint16_t MeasValue;

  loopCount++;

  // BMP180 sensor.
  Serial.println("BMP180");
  PSensor.refresh();                    // read current sensor data
  PSensor.calculate();                  // calculate temp and pressure
  TempF = PSensor.temperature;          // value returned in 1/10 degrees
  Serial.print("BMP180 T(raw): ");
  Serial.println(TempF); /// DEBUG
  P = PSensor.pressure * 100 / 3386;    // inches Hg = Pa / 33.863886667

  // TMP006 sensor
  long diet = tmp006.readDieTempC();   // Read die temp in C * 10
  Serial.print("TMP006 T(raw): ");
  Serial.println(diet); /// DEBUG
  Ti = diet * 9;                   // Convert to F * 10
  Ti = Ti / 5 + 320;

  // ISL29023 sensor
  Serial.println("ISL");
  long luxLong = isl.read();
  Serial.print("LUX (raw): ");
  Serial.println(luxLong); /// DEBUG
  lux = luxLong;
  if (luxLong > 64000) lux = 65535;

  // MSP430 internal temp sensor
  Serial.println("MSP430");
  analogReference(INTERNAL1V5);
  ADCraw = analogRead(TEMPSENSOR);
  ADCraw = analogRead(TEMPSENSOR);
  // Calibration factor Tc is scaled by 10,000 to avoid floating point math,
  // so adjust temp calculation and conversion to degrees F accordingly
  msp430T = (Tc * (ADCraw - (*Cal30)) + 300000L) * 18L / 10000L + 320L;

  // MSP430 battery voltage (Vcc) calibrated against reference
  // Start with 2.0V reference, which requires Vcc >= 2.3 V
  // Once Vcc is below 2.8V, then switch to 1.5V reference. 
  // Vcc = (4096 * 1.2 V) / 1.2-V reference ADC result
  // Internal 1.2V reference is on ADC channel 13
  analogReference(INTERNAL2V0);
  analogRead(A11);
  ADCraw = analogRead(A11);
  // Need calculation to be Long int due to mV scaling
  msp430mV = ADCraw * 4000L;
  msp430mV = msp430mV / 4095L;
  // Use 1.5V reference if Vcc < 2.8V
  if (msp430mV < 2800) {
    analogReference(INTERNAL1V5);
    ADCraw = analogRead(A11);
    msp430mV = ADCraw * 3000L;
    msp430mV = msp430mV / 4095L;
  }

  // SHT21 Temp and Humidity sensor
  long shtTtemp = mysht.getTemperature();
  Serial.print("SHT Temp(raw): ");
  Serial.println(shtTtemp); /// DEBUG
  if (shtTtemp == 1) { // Error reading temperature
    shtT = -999;
  } else {
    shtTtemp = (shtTtemp * 9L + 1600L);
    shtTtemp = shtTtemp / 5L;
    shtT = shtTtemp;
  }
  if ((shtT < -400) || (shtT > 1400)) shtT = -998;
  shtH = mysht.getHumidity();
  Serial.print("SHT RH% (raw): ");
  Serial.println(shtH); /// DEBUG
  if (shtH == 1) { // Error reading temperature
    shtH = -999;
  }
  if ((shtH < 0) || (shtH > 1000)) shtH = -998;

  // Make sure I2C bus is clear so we can share with SPI
  Wire.endTransmission(true);

  // Write the data to FRAM
  weatherindex = flash.dataIndex;
  weatherindex++;
  if (weatherindex >= maxWeatherRecords) weatherindex = 1; // Circular buffer

  weatherdata[weatherindex].BMP180_T = TempF;
  weatherdata[weatherindex].BMP180_P = P;
  weatherdata[weatherindex].TMP106_Ti = Ti;
  weatherdata[weatherindex].LUX = lux;
  weatherdata[weatherindex].SHT_T = shtT;
  weatherdata[weatherindex].SHT_H = shtH;
  weatherdata[weatherindex].MSP_T = msp430T;
  weatherdata[weatherindex].Batt_mV = msp430mV;
  weatherdata[weatherindex].Loops = loopCount;
  weatherdata[weatherindex].Millis = millis();
  weatherdata[weatherindex].Resets = bootCount;
  flash.dataIndex = weatherindex;

  // Send the data over-the-air
  memcpy(&txPacket.message, &weatherdata[weatherindex], sizeof(WeatherData));
  Radio.transmit(ADDRESS_REMOTE, (unsigned char*)&txPacket, sizeof(WeatherData)+4);
  Serial.print("Tx 'From' address: ");
  Serial.println(txPacket.from);

  Serial.println(weatherindex);
  Serial.println(weatherdata[weatherindex].BMP180_T);
  Serial.println(weatherdata[weatherindex].BMP180_P);
  Serial.println(weatherdata[weatherindex].TMP106_Ti);
  Serial.println(weatherdata[weatherindex].LUX);
  Serial.println(weatherdata[weatherindex].SHT_T);
  Serial.println(weatherdata[weatherindex].SHT_H);
  Serial.println(weatherdata[weatherindex].MSP_T);
  Serial.println(weatherdata[weatherindex].Batt_mV);
  Serial.println(weatherdata[weatherindex].Loops);
  Serial.println(weatherdata[weatherindex].Millis);
  Serial.println(weatherdata[weatherindex].Resets);
  Serial.println(F("--"));

#ifdef LCD_ENABLED
  displayOnLCD();
#endif

  if (digitalRead(PUSH2) == 0) {
    Serial.println(F("Dump Info FRAM"));
    for (int i = 0; i < maxWeatherRecords; i++) {
      if (i == 0) { // First block contains the marker and global info
        Serial.println(F("Block: 0"));
        Serial.println((unsigned int)flash.numBoots);
        Serial.println((unsigned int)flash.dataIndex);
      }
      else { // Remaining blocks contain the weather data
        Serial.print(F("Block: "));
        Serial.println(i);
        Serial.println(weatherdata[i].BMP180_T);
        Serial.println(weatherdata[i].BMP180_P);
        Serial.println(weatherdata[i].TMP106_Ti);
        Serial.println(weatherdata[i].LUX);
        Serial.println(weatherdata[i].MSP_T);
        Serial.println(weatherdata[i].Batt_mV);
        Serial.println(weatherdata[i].Loops);
        Serial.println(weatherdata[i].Millis);
        Serial.println(weatherdata[i].Resets);
      }
    }
  }

  sleep(sleepTime);
}

#ifdef LCD_ENABLED
void displayOnLCD() {
  myLCD.clear();
  myLCD.displayText(F("TEMP F"));
  delay(500);
  myLCD.clear();
  myLCD.println(PSensor.temperature / 10);
  delay(1500);
  myLCD.clear();
  myLCD.displayText(F("P INHG"));
  delay(500);
  myLCD.clear();
  myLCD.println(P);
  myLCD.showSymbol(LCD_SEG_DOT2, 1);
  delay(1500);
  myLCD.clear();
  myLCD.displayText(F("OBJ T"));
  delay(500);
  myLCD.clear();
  myLCD.println(Ti);
  delay(1500);
  myLCD.clear();
  myLCD.displayText(F("DIE T"));
  delay(500);
  myLCD.clear();
  myLCD.println(Te);
  delay(1500);
  myLCD.clear();
  myLCD.displayText(F("LUX"));
  delay(500);
  myLCD.clear();
  myLCD.println(lux);
  delay(1500);
  myLCD.clear();
}
#endif

