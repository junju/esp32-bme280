#include <Wire.h>

#define ADDRESS_I2C 0x76
#define REGISTER_id 0xD0
#define REGISTER_ctrl_meas 0xF4
#define REGISTER_ctrl_hum 0xF2

uint8_t read_data[8];
uint32_t temprature_raw, pressure_raw, humidity_raw;
double temprature, humidity, pressure;

struct calibration_t {
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;
  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;
  uint8_t dig_H1;
  int16_t dig_H2;
  uint8_t dig_H3;
  int16_t dig_H4;
  int16_t dig_H5;
  int8_t  dig_H6;
  int32_t t_fine;
};

struct calibration_t calibrations;

void setup() {
  Serial.begin(115200);

  Wire.begin(21,22);

  // デバイスIDの表示
  Wire.beginTransmission(ADDRESS_I2C);
  Wire.write(REGISTER_id);
  Wire.endTransmission(false);
  Wire.requestFrom(ADDRESS_I2C, 1, false);

  Serial.print("Device id is 0x");
  Serial.println(Wire.read(),HEX);

  Wire.endTransmission(true);

  // 校正データの読み込み
  calibrations = getCalibrationData();  

  // configレジスタをセット
  // リセット時Ox00
  
  // ctrl_measレジスタをセット
  Wire.beginTransmission(ADDRESS_I2C);
  Wire.write(REGISTER_ctrl_meas);
  Wire.write(B00100111);
  Wire.endTransmission(true);

  // ctrl_humレジスタをセット
  Wire.beginTransmission(ADDRESS_I2C);
  Wire.write(REGISTER_ctrl_hum);
  Wire.write(B00000001);
  Wire.endTransmission(true);  

}

void loop() {
  
  // センサデータ読み込み 
  Wire.beginTransmission(ADDRESS_I2C);
  Wire.write(0xF7);
  Wire.endTransmission(false);
  Wire.requestFrom(ADDRESS_I2C, 8, false);
  
  for (int i =0; i < sizeof(read_data); i++){
    read_data[i] = Wire.read();
  }
  
  Wire.endTransmission(true);

  temprature_raw = read_data[3] << 12 | read_data[4] << 4 | read_data[5] >> 4;
  humidity_raw  = read_data[6] << 8 | read_data[7];
  pressure_raw = read_data[0] << 12 | read_data[1] << 4 | read_data[2] >> 4;

  temprature = (double)compensateTemperature(temprature_raw, &calibrations)/100.0;
  humidity = (double)compensateHumidity(humidity_raw, &calibrations)/1024.0;
  pressure = (double)compensatePressure(pressure_raw, &calibrations)/100.0;
  
  Serial.print(" T: ");
  Serial.print(temprature);
  Serial.print(" H: ");
  Serial.print(humidity);
  Serial.print(" P: ");
  Serial.println(pressure);

  delay(1000);
  
  }

  calibration_t getCalibrationData(void){

    calibration_t results;
    uint8_t read_data[33];
    uint8_t i;

    i = 0;
    
    // 校正データ読み込み 
    Wire.beginTransmission(ADDRESS_I2C);
    Wire.write(0x88);
    Wire.endTransmission(false);
    Wire.requestFrom(ADDRESS_I2C, 24, false);
    while (Wire.available()){
      read_data[i] = Wire.read();
      i++;
    }
    //Serial.print(i);
    Wire.endTransmission(true);

    Wire.beginTransmission(ADDRESS_I2C);
    Wire.write(0xA1);
    Wire.endTransmission(false);
    Wire.requestFrom(ADDRESS_I2C, 1, false);
    read_data[i] = Wire.read();
    i++;
    Wire.endTransmission(true);
    
    Wire.beginTransmission(ADDRESS_I2C);
    Wire.write(0xE1);
    Wire.endTransmission(false);
    Wire.requestFrom(ADDRESS_I2C, 7, false);
    while (Wire.available()){
      read_data[i] = Wire.read();
      i++;
    }
    Wire.endTransmission(true);

    results.dig_T1 = read_data[1] << 8 | read_data[0];
    results.dig_T2 = read_data[3] << 8 | read_data[2];
    results.dig_T3 = read_data[5] << 8 | read_data[4];

    results.dig_P1 = (read_data[7] << 8) | read_data[6];
    results.dig_P2 = (read_data[9] << 8) | read_data[8];
    results.dig_P3 = (read_data[11]<< 8) | read_data[10];
    results.dig_P4 = (read_data[13]<< 8) | read_data[12];
    results.dig_P5 = (read_data[15]<< 8) | read_data[14];
    results.dig_P6 = (read_data[17]<< 8) | read_data[16];
    results.dig_P7 = (read_data[19]<< 8) | read_data[18];
    results.dig_P8 = (read_data[21]<< 8) | read_data[20];
    results.dig_P9 = (read_data[23]<< 8) | read_data[22];

    results.dig_H1 = read_data[24];
    results.dig_H2 = (read_data[26]<< 8) | read_data[25];
    results.dig_H3 = read_data[27];
    //results.dig_H4 = (read_data[28]<< 4) | (0x0F & read_data[29]);
    //results.dig_H5 = (read_data[30] << 4) | ((read_data[29] >> 4) & 0x0F);
    results.dig_H6 = read_data[32];

    int16_t dig_H4_lsb;
    int16_t dig_H4_msb;
    int16_t dig_H5_lsb;
    int16_t dig_H5_msb;

    dig_H4_msb = (int16_t)(int8_t)read_data[28] * 16;
    dig_H4_lsb = (int16_t)(read_data[29] & 0x0F);
    results.dig_H4 = dig_H4_msb | dig_H4_lsb;

    dig_H5_msb = (int16_t)(int8_t)read_data[31] * 16;
    dig_H5_lsb = (int16_t)(read_data[30] >> 4);
    results.dig_H5 = dig_H5_msb | dig_H5_lsb;
    
    return results;

  }
  
int32_t compensateTemperature(uint32_t rawData, struct calibration_t *calibrations){
  int32_t var1;
  int32_t var2;
  int32_t temperature;
  int32_t temperature_min = -4000;
  int32_t temperature_max = 8500;

  var1 = (int32_t)((rawData / 8) - ((int32_t)calibrations->dig_T1 * 2));
  var1 = (var1 * ((int32_t)calibrations->dig_T2)) / 2048;
  var2 = (int32_t)((rawData / 16) - ((int32_t)calibrations->dig_T1));
  var2 = (((var2 * var2) / 4096) * ((int32_t)calibrations->dig_T3)) / 16384;
  calibrations->t_fine = var1 + var2;
  temperature = (calibrations->t_fine * 5 + 128) / 256;

  if (temperature < temperature_min)
    temperature = temperature_min;
  else if (temperature > temperature_max)
    temperature = temperature_max;

  return temperature;
}

uint32_t compensateHumidity(uint32_t rawData, struct  calibration_t *calibrations){
  int32_t var1;
  int32_t var2;
  int32_t var3;
  int32_t var4;
  int32_t var5;
  uint32_t humidity;
  uint32_t humidity_max = 100000;

  var1 = calibrations->t_fine - ((int32_t)76800);
  var2 = (int32_t)(rawData * 16384);
  var3 = (int32_t)(((int32_t)calibrations->dig_H4) * 1048576);
  var4 = ((int32_t)calibrations->dig_H5) * var1;
  var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
  var2 = (var1 * ((int32_t)calibrations->dig_H6)) / 1024;
  var3 = (var1 * ((int32_t)calibrations->dig_H3)) / 2048;
  var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
  var2 = ((var4 * ((int32_t)calibrations->dig_H2)) + 8192) / 16384;
  var3 = var5 * var2;
  var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
  var5 = var3 - ((var4 * ((int32_t)calibrations->dig_H1)) / 16);
  var5 = (var5 < 0 ? 0 : var5);
  var5 = (var5 > 419430400 ? 419430400 : var5);
  humidity = (uint32_t)(var5 / 4096);

  if (humidity > humidity_max)
    humidity = humidity_max;

  return humidity;
}

uint32_t compensatePressure(uint32_t rawData, struct calibration_t *calibrations){
  int32_t var1;
  int32_t var2;
  int32_t var3;
  int32_t var4;
  uint32_t var5;
  uint32_t pressure;
  uint32_t pressure_min = 30000;
  uint32_t pressure_max = 110000;

  var1 = (((int32_t)calibrations->t_fine) / 2) - (int32_t)64000;
  var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t)calibrations->dig_P6);
  var2 = var2 + ((var1 * ((int32_t)calibrations->dig_P5)) * 2);
  var2 = (var2 / 4) + (((int32_t)calibrations->dig_P4) * 65536);
  var3 = (calibrations->dig_P3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
  var4 = (((int32_t)calibrations->dig_P2) * var1) / 2;
  var1 = (var3 + var4) / 262144;
  var1 = (((32768 + var1)) * ((int32_t)calibrations->dig_P1)) / 32768;
   /* avoid exception caused by division by zero */
  if (var1) {
    var5 = (uint32_t)((uint32_t)1048576) - rawData;
    pressure = ((uint32_t)(var5 - (uint32_t)(var2 / 4096))) * 3125;
    if (pressure < 0x80000000)
      pressure = (pressure << 1) / ((uint32_t)var1);
    else
      pressure = (pressure / (uint32_t)var1) * 2;

    var1 = (((int32_t)calibrations->dig_P9) * ((int32_t)(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
    var2 = (((int32_t)(pressure / 4)) * ((int32_t)calibrations->dig_P8)) / 8192;
    pressure = (uint32_t)((int32_t)pressure + ((var1 + var2 + calibrations->dig_P7) / 16));

    if (pressure < pressure_min)
      pressure = pressure_min;
    else if (pressure > pressure_max)
      pressure = pressure_max;
  } else {
    pressure = pressure_min;
  }

  return pressure;
}
