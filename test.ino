#include <Wire.h>
#include <WiFi.h>

#define BME280_ADDRESS 0x76

static const char* ssid     = "";
static const char* password = "";
 
static const char* server   = "api.thingspeak.com"; 
String apiKey = "";

WiFiClient client;

struct calibration_t{
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
 int8_t  dig_H1;
 int16_t dig_H2;
 int8_t  dig_H3;
 int16_t dig_H4;
 int16_t dig_H5;
 int8_t  dig_H6;
 signed long int t_fine;
};

struct rawData_t{
  unsigned long int humminity_raw,temperature_raw,pressure_raw;
};

struct calibration_t inits = {
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
  };

struct rawData_t inits_rawData = {
  0,0,0
};

struct calibration_t* calibrations;
struct rawData_t* rawData;

void setup()
{
    uint8_t osrs_t = B010;             //Temperature oversampling x 2
    uint8_t osrs_p = B101;             //Pressure oversampling x 16
    uint8_t osrs_h = B001;             //Humidity oversampling x 1
    uint8_t mode = B11;               //Normal mode
    uint8_t t_sb = B000;               //Tstandby 0.5ms
    uint8_t filter = B100;             //Filter on 
    uint8_t spi3w_en = 0;           //3-wire SPI Disable
    
    uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
    uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
    uint8_t ctrl_hum_reg  = osrs_h;
    
    calibrations = &inits;
    rawData = &inits_rawData;
    
    Serial.begin(115200);
    Wire.begin(21,22);
    
    writeReg(0xF2,ctrl_hum_reg);
    writeReg(0xF4,ctrl_meas_reg);
    writeReg(0xF5,config_reg);
    
    readTrim(calibrations);                    //

  WiFi.begin(ssid, password) ;
  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(WiFi.status());
  }
  Serial.println("\n\nWiFi connected.") ;

}


void loop()
{
    double temperature = 0.0, pressure = 0.0,humidity = 0.0;
    signed long int temp_cal;
    unsigned long int press_cal,hum_cal;
    
    readData(rawData);
    
    temp_cal = calibration_T(rawData->temperature_raw, calibrations);
    press_cal = calibration_P(rawData->pressure_raw, calibrations);
    hum_cal = calibration_H(rawData->humminity_raw, calibrations);
    temperature = (double)temp_cal / 100.0;
    pressure = (double)press_cal / 100.0;
    humidity = (double)hum_cal / 1024.0;
    Serial.print("TEMP : ");
    Serial.print(temperature);
    Serial.print(" DegC  PRESS : ");
    Serial.print(pressure);
    Serial.print(" hPa  HUM : ");
    Serial.print(humidity);
    Serial.println(" %");    

    if (client.connect(server, 80)) { 
    String postStr = apiKey ;
    postStr += "&field1=" + String(temperature);
    postStr += "&field2=" + String(humidity);
    postStr += "&field3=" + String(pressure);
    postStr += "\r\n\r\n" ;
 
    client.println("POST /update HTTP/1.1") ;
    client.println("Host: api.thingspeak.com") ;
    client.println("Connection: close") ;
    client.println("X-THINGSPEAKAPIKEY: " + apiKey) ;
    client.println("Content-Type: application/x-www-form-urlencoded") ;
    client.print("Content-Length: ") ;
    client.println(postStr.length()) ;
    client.println("") ;
    client.print(postStr) ;
  }
  client.stop();
 
  delay(30 * 1000);
  
    
    //delay(1000);
}
void readTrim(struct calibration_t* calibrations)
{
    uint8_t data[32],i=0;                      // Fix 2014/04/06
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0x88);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,24);       // Fix 2014/04/06
    while(Wire.available()){
        data[i] = Wire.read();
        i++;
    }
    
    Wire.beginTransmission(BME280_ADDRESS);    // Add 2014/04/06
    Wire.write(0xA1);                          // Add 2014/04/06
    Wire.endTransmission();                    // Add 2014/04/06
    Wire.requestFrom(BME280_ADDRESS,1);        // Add 2014/04/06
    data[i] = Wire.read();                     // Add 2014/04/06
    i++;                                       // Add 2014/04/06
    
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0xE1);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,7);        // Fix 2014/04/06
    while(Wire.available()){
        data[i] = Wire.read();
        i++;    
    }
    calibrations->dig_T1 = (data[1] << 8) | data[0];
    calibrations->dig_T2 = (data[3] << 8) | data[2];
    calibrations->dig_T3 = (data[5] << 8) | data[4];
    calibrations->dig_P1 = (data[7] << 8) | data[6];
    calibrations->dig_P2 = (data[9] << 8) | data[8];
    calibrations->dig_P3 = (data[11]<< 8) | data[10];
    calibrations->dig_P4 = (data[13]<< 8) | data[12];
    calibrations->dig_P5 = (data[15]<< 8) | data[14];
    calibrations->dig_P6 = (data[17]<< 8) | data[16];
    calibrations->dig_P7 = (data[19]<< 8) | data[18];
    calibrations->dig_P8 = (data[21]<< 8) | data[20];
    calibrations->dig_P9 = (data[23]<< 8) | data[22];
    calibrations->dig_H1 = data[24];
    calibrations->dig_H2 = (data[26]<< 8) | data[25];
    calibrations->dig_H3 = data[27];
    calibrations->dig_H4 = (data[28]<< 4) | (0x0F & data[29]);
    calibrations->dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F); // Fix 2014/04/06
    calibrations->dig_H6 = data[31];                                   // Fix 2014/04/06
}
void writeReg(uint8_t reg_address, uint8_t data)
{
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(reg_address);
    Wire.write(data);
    Wire.endTransmission();    
}


void readData(struct rawData_t* rawData)
{
    int i = 0;
    uint32_t data[8];
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0xF7);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,8);
    while(Wire.available()){
        data[i] = Wire.read();
        i++;
    }
    rawData->pressure_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    rawData->temperature_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    rawData->humminity_raw  = (data[6] << 8) | data[7];
}


signed long int calibration_T(signed long int adc_T, struct calibration_t* trims)
{
    
    signed long int var1, var2, T;
    var1 = ((((adc_T >> 3) - ((signed long int)trims->dig_T1<<1))) * ((signed long int)trims->dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((signed long int)trims->dig_T1)) * ((adc_T>>4) - ((signed long int)trims->dig_T1))) >> 12) * ((signed long int)trims->dig_T3)) >> 14;
    
    trims->t_fine = var1 + var2;
    T = (trims->t_fine * 5 + 128) >> 8;
    return T; 
}

unsigned long int calibration_P(signed long int adc_P, struct calibration_t* trims)
{
    signed long int var1, var2;
    unsigned long int P;
    var1 = (((signed long int)trims->t_fine)>>1) - (signed long int)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int)trims->dig_P6);
    var2 = var2 + ((var1*((signed long int)trims->dig_P5))<<1);
    var2 = (var2>>2)+(((signed long int)trims->dig_P4)<<16);
    var1 = (((trims->dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((signed long int)trims->dig_P2) * var1)>>1))>>18;
    var1 = ((((32768+var1))*((signed long int)trims->dig_P1))>>15);
    if (var1 == 0)
    {
        return 0;
    }    
    P = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
    if(P<0x80000000)
    {
       P = (P << 1) / ((unsigned long int) var1);   
    }
    else
    {
        P = (P / (unsigned long int)var1) * 2;    
    }
    var1 = (((signed long int)trims->dig_P9) * ((signed long int)(((P>>3) * (P>>3))>>13)))>>12;
    var2 = (((signed long int)(P>>2)) * ((signed long int)trims->dig_P8))>>13;
    P = (unsigned long int)((signed long int)P + ((var1 + var2 + trims->dig_P7) >> 4));
    return P;
}

unsigned long int calibration_H(signed long int adc_H, struct calibration_t* trims)
{
    signed long int v_x1;
    
    v_x1 = (trims->t_fine - ((signed long int)76800));
    v_x1 = (((((adc_H << 14) -(((signed long int)trims->dig_H4) << 20) - (((signed long int)trims->dig_H5) * v_x1)) + 
              ((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)trims->dig_H6)) >> 10) * 
              (((v_x1 * ((signed long int)trims->dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) * 
              ((signed long int) trims->dig_H2) + 8192) >> 14));
   v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)trims->dig_H1)) >> 4));
   v_x1 = (v_x1 < 0 ? 0 : v_x1);
   v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
   return (unsigned long int)(v_x1 >> 12);   
}
