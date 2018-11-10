void get_BME280_Calibration() {
  uint8_t  data8;
  uint16_t data16;

  myBME280.read2bFromRegister(BME280_CALIBRATION_T1, &dig_T1); // uint16_t
  myBME280.read2bFromRegister(BME280_CALIBRATION_T2, &data16); // int16_t
  dig_T2 = (int16_t) data16;
  myBME280.read2bFromRegister(BME280_CALIBRATION_T3, &data16); // int16_t
  dig_T3 = (int16_t) data16;

  myBME280.read2bFromRegister(BME280_CALIBRATION_P1, &dig_P1); // uint16_t
  myBME280.read2bFromRegister(BME280_CALIBRATION_P2, &data16); // int16_t
  dig_P2 = (int16_t) data16;
  myBME280.read2bFromRegister(BME280_CALIBRATION_P3, &data16); // int16_t
  dig_P3 = (int16_t) data16;
  myBME280.read2bFromRegister(BME280_CALIBRATION_P4, &data16); // int16_t
  dig_P4 = (int16_t) data16;
  myBME280.read2bFromRegister(BME280_CALIBRATION_P5, &data16); // int16_t
  dig_P5 = (int16_t) data16;
  myBME280.read2bFromRegister(BME280_CALIBRATION_P6, &data16); // int16_t
  dig_P6 = (int16_t) data16;
  myBME280.read2bFromRegister(BME280_CALIBRATION_P7, &data16); // int16_t
  dig_P7 = (int16_t) data16;
  myBME280.read2bFromRegister(BME280_CALIBRATION_P8, &data16); // int16_t
  dig_P8 = (int16_t) data16;
  myBME280.read2bFromRegister(BME280_CALIBRATION_P9, &data16); // int16_t
  dig_P9 = (int16_t) data16;

  myBME280.read1bFromRegister(BME280_CALIBRATION_H1, &dig_H1); // uint8_t
  myBME280.read2bFromRegister(BME280_CALIBRATION_H2, &data16); // int16_t
  dig_H2 = (int16_t) data16;
  myBME280.read1bFromRegister(BME280_CALIBRATION_H3, &dig_H3); // uint8_t

  //    0xE4 / 0xE5[3:0]  dig_H4 [11:4] / [3:0] signed short = int16_t
  myBME280.read1bFromRegister(BME280_CALIBRATION_H4, &data8);
  //Serial.print("Register Address E4: 0x"); ///
  //Serial.println(data8, HEX); ///
  dig_H4 = data8 << 4;
  myBME280.read1bFromRegister(BME280_CALIBRATION_H5, &data8);
  //Serial.print("Register Address E5: 0x"); ///
  //Serial.println(data8, HEX); ///
  dig_H4 = dig_H4 + (data8 & 0x0F);
  //Serial.print("dig_H4: 0x"); ///
  //Serial.println(dig_H4, HEX); ///
  //    0xE5[7:4] / 0xE6 dig_H5 [3:0] / [11:4] signed short = int16_t
  // dig_H5  = (myBME280.read1bFromRegister(BME280_SLAVE_ADDRESS, BME280_CALIBRATION_H5) >> 4) + (myBME280.read1bFromRegister(BME280_SLAVE_ADDRESS, BME280_CALIBRATION_H5B2) << 4);
  myBME280.read1bFromRegister(BME280_CALIBRATION_H5, &data8);
  //Serial.print("Register Address E5: 0x"); ///
  //Serial.println(data8, HEX); ///
  dig_H5 = (data8 >> 4) & 0x0F;
  myBME280.read1bFromRegister(BME280_CALIBRATION_H5B2, &data8);
  //Serial.print("Register Address E6: 0x"); ///
  //Serial.println(data8, HEX); ///
  data16 = data8;
  dig_H5 = dig_H5 + (data16 << 4);
  //Serial.print("dig_H5: 0x"); ///
  //Serial.println(dig_H5, HEX); ///

  myBME280.read1bFromRegister(BME280_CALIBRATION_H5, &data8); // int8_t
  dig_H6 = (int8_t) data8;
}

void BME280_get() {
  // Write register BME280_DATA_F7_FE with no data, plus a stop
  // 8 consecutive byte reads
  myBME280.startBit();
  myBME280.writeAddress(0); // 0 == Write bit
  myBME280.checkAckBit();
  myBME280.writeRegister(BME280_DATA_F7_FE);
  myBME280.checkAckBit();
  //  stopBit();
  myBME280.startBit();
  myBME280.writeAddress(1); // 1 == Read bit
  myBME280.checkAckBit();
  // Loop 8 bytes
  for (int i = 0; i < 8; i++) {
    BME280RawData[i] = myBME280.read1Byte();
    if (i < 7) {
      myBME280.writeAck();
    }
    else { // Last byte needs a NACK
      myBME280.checkAckBit(); // Master needs to send NACK when done reading data
    }
  }
  myBME280.stopBit();

  /*
    _rawPressure = ((uint32_t)Wire.read() << 12) + ((uint32_t)Wire.read() << 4) + ((uint32_t)Wire.read() >> 4); // f7.f8.f9
    _rawTemperature = ((uint32_t)Wire.read() << 12) + ((uint32_t)Wire.read() << 4) + ((uint32_t)Wire.read() >> 4); // fa.fb.fc
    _rawHumidity = ((uint32_t)Wire.read() << 8) + ((uint32_t)Wire.read()); // fd.fe
  */

  /*
    Serial.print("BME280 Raw data: 0x");
    for (int i = 0; i < 8; i++) {
      Serial.print(BME280RawData[i]);
      Serial.print(", 0x");
    }
    Serial.println();
  */

  rawBME280P = ((uint32_t)BME280RawData[0] << 12) + ((uint32_t)BME280RawData[1] << 4) + ((uint32_t)BME280RawData[2] >> 4);
  rawBME280T = ((uint32_t)BME280RawData[3] << 12) + ((uint32_t)BME280RawData[4] << 4) + ((uint32_t)BME280RawData[5] >> 4);
  rawBME280H = ((uint32_t)BME280RawData[6] <<  8) + (uint32_t)BME280RawData[7];
}

int32_t calculate_BME280_T() {  // Returns 0.01 degrees C (e.g. 2101 = 21.01 C)
  int32_t var1, var2, T;
  var1  = ((((rawBME280T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  var2  = (((((rawBME280T >> 4) - ((int32_t)dig_T1)) * ((rawBME280T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;

  return ((t_fine * 5 + 128) >> 8);  // 0.01 C
}

uint32_t calculate_BME280_P() { // Returns pressure in Pascals (e.g 98729 = 98729 Pa = 987.29 hPa)
  int64_t var1, var2, var3;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

  if (var1 != 0)
  {
    var3 = 1048576 - rawBME280P;
    var3 = (((var3 << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (var3 >> 13) * (var3 >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * var3) >> 19;

    var3 = ((var3 + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (var3 / 256); // in Pa
  }
  else {   // Avoid divide by 0 and return 0
    return 0;
  }
}

uint32_t calculate_BME280_H() { // Returns % RH
  int32_t v_x1_u32r;

  v_x1_u32r = (t_fine - ((int32_t)76800));
  v_x1_u32r = (((((rawBME280H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
  if (v_x1_u32r < 0)
  {
    v_x1_u32r = 0;
  }
  if (v_x1_u32r > 419430400)
  {
    v_x1_u32r = 419430400;
  }
  return ((v_x1_u32r >> 12) / 102); // in 0.1% (Note that a more accurate result is to divide by 1024 and get 0.01%
}

void TMP007_startup() {
  myTMP007.begin();
  myTMP007.write2bToRegisterMSBFirst(TMP007_CONFIGURATION, TMP007_VALUE_CONFIG);
}

void OPT3001_startup() {
  // Since OPT3001 is on same I2C bus, don't need to call begin() since pins are already initialized
  // myOPT3001.begin();
  myOPT3001.write2bToRegisterMSBFirst(OPT3001_CONFIGURATION_REGISTER, OPT3001_STARTUP_CONFIG);
}

void BME280_startup() {
  // Initialize BME280
  // myBME280.begin()  // Since on the same I2C bus, don't need to call begin() since pins are already initialized
  myBME280.writeToRegister(BME280_RESET, BME280_VALUE_RESET_COMMAND);   // Force a soft reset
  myBME280.writeToRegister(BME280_CONFIGURATION, BME280_VALUE_CONFIG);
  myBME280.writeToRegister(BME280_CONTROL_HUMIDITY, BME280_VALUE_HUM_CONFIG); // Becomes effective after writing TP Config
  myBME280.writeToRegister(BME280_CONTROL_TEMPERATURE_PRESSURE, BME280_VALUE_TP_CONFIG);  // Temp and Pressure configuration, needs to be set after Humidity
  get_BME280_Calibration();
}

void TMP007_get() {
  uint16_t data16;

  myTMP007.read2bFromRegisterMSBFirst(TMP007_INTERNAL_TEMPERATURE, &data16);
  // Temp is returned in 14 msb, so shift over to lsb 
  TMP007T_Internal = (int)data16 >> 2;

  // Temperature is returned in 1/32 degree Celsius in the most significant 14 bits
  // Next, multiply by 10 and divide by 32 (right shift 5) to get units in 1/10 degree Celsius (using integer math)
  TMP007T_Internal = (TMP007T_Internal * 10) >> 5;

  myTMP007.read2bFromRegisterMSBFirst(TMP007_EXTERNAL_TEMPERATURE, &data16);
  // Temp is returned in 14 msb, so shift over to lsb
  TMP007T_External = (int)data16 >> 2;

  // Temperature is returned in units of 1/32 degree Celsius
  // Next, multiply by 10 and divide by 32 (right shift 5) to get units in 1/10 degree Celsius (using integer math)
  TMP007T_External = (TMP007T_External * 10) >> 5;
}

void OPT3001_get() {
  uint16_t data16;
  unsigned int exponent;
  unsigned int mantissa;

  myOPT3001.read2bFromRegisterMSBFirst(OPT3001_RESULT_REGISTER, &data16);
  exponent = ((data16 >> 12) & 0x000f);   // Exponent is 4 msb
  exponent = 1 << exponent;               // Convert to "lsb size", in units of 1/100 lux
  mantissa = data16 & 0x0fff;             // Fractional part is 12 lsb
  OPT3001Lux = (mantissa * exponent) / 100;      // Only need precision down to single lux
}
