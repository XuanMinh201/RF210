/**

   RRRR  FFFF  22   11   000  
   R   R F    2  2 111  0  00 
   RRRR  FFF    2   11  0 0 0 
   R R   F     2    11  00  0 
   R  RR F    2222 11l1  000


   @file ATC_Command_RF210.ino
   @author XuanMinh201, FabienFerrero

   @brief This sketch add custom ATC command to RFThings RF210 board. These commands help controlling on-board sensors,
   GNSS & measuring battery level. For detail description, please visit: https://github.com/XuanMinh201/RF210

   @version 0.1.1
   @date 2023-10-26

   @copyright Copyright (c) 2023

*/

#define DATA_INTERVAL 5000 //ms

#include "Adafruit_SHTC3.h"         // http://librarymanager/All#Adafruit_SHTC3
#include <Kionix_KX023.h>           // TO-DO: Add this original
//#include "Adafruit_LTR329_LTR303.h" // http://librarymanager/All#Adafruit_LTR329_LTR303
#include <LTR303.h> // https://github.com/automote/LTR303     // update lib with : value = (high << 8) + low;
#include <MicroNMEA.h>              // http://librarymanager/All#SparkFun_Ublox

#define GPS_EN PA1
#define GPS_detected PA9
#define boot_button PH3
#define LED PA0
#define DATA_INTERVAL 5000 // ms
#define LS_ADC_AREF 3.3f
#define LS_BATVOLT_R1 1.0f
#define LS_BATVOLT_R2 2.0f
#define LS_BATVOLT_PIN PB2

Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();
KX023 myIMU;
LTR303 light;

int8_t gain=0;     // Gain setting, values = 0-7 
int8_t integrationTime=1;  // Integration ("shutter") time in milliseconds
int8_t measRate=0;  // Integration ("shutter") time in milliseconds
uint16_t voltage_adc;
uint16_t voltage;
float kx_x, kx_y, kx_z;
int humi, temper;
sensors_event_t humidity, temp;
bool valid;
bool ltr_status;
unsigned int visible, infrared;
double lux;
int timezone = 7;
int year;
int mon;
int day;
int hr;
int minute;
double sec;

char nmeaBuffer[1000];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
String revString;
char revChar[1000];
int len = 0;
uint8_t is_fixing = 0;
int t2f=0;
uint8_t on_off_gps=0;
uint8_t nmea_gps=0;
uint8_t log_gps=0;
uint8_t dc_gps=0;
uint8_t it=0;

uint32_t quectelDelayTime = 500;
unsigned long currentMillis = 0, getSensorDataPrevMillis = 0, getGPSPrevMillis = 0;
bool sht_status;


bool getSHTstatus()
{
  if (!shtc3.begin())
  {
    return 0;
  }
  return 1;
}

int ATC_Ver(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
  if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");
  }

    Serial.println("0.1.2");
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

int SHTC3_init(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
     if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");
  }
    sht_status = getSHTstatus();
    Serial.println(sht_status ? "1" : "0");
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

int SHTC3_temp(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {

    if (sht_status)
    {
       if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");
  }
      shtc3.getEvent(&humidity, &temp);
      Serial.println(temp.temperature);

    }
    else
    {
      Serial.println("0");
    }
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

int SHTC3_humi(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    if (sht_status)
    {

    if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
      shtc3.getEvent(&humidity, &temp);
      Serial.println(humidity.relative_humidity);

    }
    else
    {
      Serial.println("0");
    }
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

bool kx023_status;

bool getKX023status()
{
  if (myIMU.begin())
  {
    return 0;
  }
  else
  {
    myIMU.configContinuousReading(LOWPOWER, RANGE_8G, DATARATE_100HZ);
    return 1;
  }
}

int KX023_init(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
    Serial.println(kx023_status ? "1" : "0");
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

int KX023_AX(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    if (kx023_status)
    {
      myIMU.readAcceleration(&kx_x, &kx_y, &kx_z);
      if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
      Serial.println(kx_x);
    }
    else
    {
      if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
      Serial.println("0");
    }
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

int KX023_AY(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    if (kx023_status)
    {
      myIMU.readAcceleration(&kx_x, &kx_y, &kx_z);
      if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
      Serial.println(kx_y);
    }
    else
    {
      if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
      Serial.println("0");
    }
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

int KX023_AZ(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    if (kx023_status)
    {
      myIMU.readAcceleration(&kx_x, &kx_y, &kx_z);
      if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
      Serial.println(kx_z);
    }
    else
    {
      if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
      Serial.println("0");
    }
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}


bool getLTRstatus()
{
    light.begin();
    light.setControl(gain, false, false);
    light.setMeasurementRate(integrationTime,measRate);
    light.setPowerUp();
    
    
    return 1;
  //}
  // Most library commands will return true if communications was successful,
  // and false if there was a problem. You can ignore this returned value,
  // or check whether a command worked correctly and retrieve an error code:
  //else {
  //  return 0;
  //}  
}

unsigned char getLTRID()
{
  unsigned char ID; 
   
  if (light.getPartID(ID)) {  
    return ID;
  }
  // Most library commands will return true if communications was successful,
  // and false if there was a problem. You can ignore this returned value,
  // or check whether a command worked correctly and retrieve an error code:
  else {
    return 0;
  }  
}

int LTR_init(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    //light.setPowerUp();
    ltr_status = getLTRstatus();
    if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
    Serial.println(ltr_status ? "1" : "0");
    Serial.print("LTR Sensor Part ID: 0X");
    Serial.println(getLTRID(),HEX);
    //light.setPowerDown();
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

int LTR_ch0(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    //ltr_status = getLTRstatus();
    
    //delay(300);
    if (light.getData(visible, infrared)){
          if (param->argc == 0){
          Serial.print(cmd);
          Serial.print("=");}
          Serial.println(visible);
          //light.setPowerDown(); 
        }
    else
        {
      if (param->argc == 0){
      Serial.print(cmd);
      Serial.print("=");}
      Serial.println("0");
        }
  }

  
    else
    {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

int LTR_ch1(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    
    //ltr_status = getLTRstatus();
    //light.setPowerUp();
    //delay(300);
      if (light.getData(visible, infrared))
      {        
          if (param->argc == 0){
      Serial.print(cmd);
      Serial.print("=");}
      Serial.println(infrared);
      //light.setPowerDown();      
      }  
    else
    {
      if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
      Serial.println("0");
    }
  }
  else
  {
    return AT_PARAM_ERROR;
  }
if (param->argc == 0){
  return AT_OK;}
}

int LTR_lux(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    //ltr_status = getLTRstatus();
    //light.setPowerUp();
    //delay(300);    
      if (light.getData(visible, infrared))
      {       
          if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
     
    valid = light.getLux(gain,integrationTime,visible,infrared,lux);     
     Serial.println(lux);
    // light.setPowerDown();        
      }
    
    else
    {
      if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
      Serial.println("0");
    }
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

bool GPS_status = 0;

int GPS_STT(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    
    if (Serial1.available()){
      GPS_status = 1;
    }
    else {
      GPS_status = 0;
    }
    if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
    Serial.println(GPS_status ? "1" : "0");
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}


int GPS_on_off(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
   if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
   Serial.println(on_off_gps ? "HIGH" : "LOW");
  }
  else if (param->argc == 1)
  {
    for (int i = 0; i < strlen(param->argv[0]); i++)
    {
      if (!isdigit(*(param->argv[0] + i)))
      {
        return AT_PARAM_ERROR;
      }
    }
    on_off_gps = strtoul(param->argv[0], NULL, 10);
    getGPSPrevMillis=millis();
    is_fixing=1; it=0; t2f=0; nmea.clear();
    Serial1.begin(9600, RAK_CUSTOM_MODE);   
    
    if (on_off_gps != 0 && on_off_gps != 1)
    {
      return AT_PARAM_ERROR;
    }
    digitalWrite(GPS_EN, (on_off_gps == 1) ? HIGH : LOW);
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

int GPS_sat(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
    Serial.println(nmea.getNumSatellites());
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

int GPS_time(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    uint32_t unixt = unixTimestamp(nmea.getYear(), nmea.getMonth(), nmea.getDay(), nmea.getHour(), nmea.getMinute(), nmea.getSecond());
    if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
    Serial.println(unixt);
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

int GPS_fix(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    
    if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
    Serial.println(nmea.isValid());
    }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

int GPS_lat(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    double latitude = nmea.getLatitude();
    if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
    Serial.println(latitude / 1.0e6, 4);
    //    } else {
    //      Serial.println("0");
    //    }
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

int GPS_lon(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    double longitude = nmea.getLongitude();
    if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
    Serial.println(longitude / 1.0e6, 4);
    //    } else {
    //      Serial.println("0");
    //    }
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

int GPS_alt(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    long alt;
    if (nmea.getAltitude(alt))
    {
      if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
      Serial.println(alt / 1000., 3);
    }
    else
    {
      if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
      Serial.println("0");
    }
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

int GPS_nmea(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
   if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
   Serial.println(nmea_gps ? "ON" : "OFF");
  }
  else if (param->argc == 1)
  {
    for (int i = 0; i < strlen(param->argv[0]); i++)
    {
      if (!isdigit(*(param->argv[0] + i)))
      {
        return AT_PARAM_ERROR;
      }
    }
    nmea_gps = strtoul(param->argv[0], NULL, 10);
    if (nmea_gps != 0 && nmea_gps != 1)
    {
      return AT_PARAM_ERROR;
    }
    //digitalWrite(GPS_EN, (nmea_gps == 1) ? ON : OFF);
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

int GPS_log(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
   if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
   Serial.println(log_gps ? "ON" : "OFF");
  }
  else if (param->argc == 1)
  {
    for (int i = 0; i < strlen(param->argv[0]); i++)
    {
      if (!isdigit(*(param->argv[0] + i)))
      {
        return AT_PARAM_ERROR;
      }
    }
    log_gps = strtoul(param->argv[0], NULL, 10);
    if (log_gps != 0 && log_gps != 1)
    {
      return AT_PARAM_ERROR;
    }
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

int GPS_dc(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    
   if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
   Serial.println(dc_gps ? "ON" : "OFF");
  }
  else if (param->argc == 1)
  {
    for (int i = 0; i < strlen(param->argv[0]); i++)
    {
      if (!isdigit(*(param->argv[0] + i)))
      {
        return AT_PARAM_ERROR;
      }
    }
    dc_gps = strtoul(param->argv[0], NULL, 10);
    if(dc_gps==1){
    Serial1.println("$PGKC105,1,2000,30000*07");}
    else if(dc_gps==0){
      Serial1.println("$PGKC105,0*37");}
    if (log_gps != 0 && log_gps != 1)
    {
      return AT_PARAM_ERROR;
    }
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

int GPS_const(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    
   if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
   Serial.println("activate all Constallations");
   Serial1.println("$PGKC121,1,1,0,1*2C");
  }
    
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}


int battery(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    
    voltage_adc = (uint16_t)analogRead(LS_BATVOLT_PIN);
    voltage = (uint16_t)((LS_ADC_AREF / 1.024) * (LS_BATVOLT_R1 + LS_BATVOLT_R2) / LS_BATVOLT_R2 * (float)voltage_adc);
    if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
    Serial.println(voltage);
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

int ldo_read(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
    Serial.println(api.system.bat.get()*1000);
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}


void setup()
{
  Serial.begin(115200, RAK_AT_MODE);
  Serial1.begin(9600, RAK_CUSTOM_MODE);
  pinMode(GPS_EN, OUTPUT);
  pinMode(GPS_detected, INPUT);
  pinMode(boot_button, INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  sht_status = getSHTstatus();
  kx023_status = getKX023status();
  ltr_status = getLTRstatus();
  
  while (!Serial)
  {
    delay(10); // wait for serial port to open
  }

  api.system.atMode.add("VER", "Return firmware version", "VER", ATC_Ver,RAK_ATCMD_PERM_READ);

  api.system.atMode.add("SHT", "Return the status of the SHTC3 sensor. 1 if available.", "SHT", SHTC3_init,RAK_ATCMD_PERM_READ);
  api.system.atMode.add("TEMP", "Return the temperature value with 0.01° resolution", "TEMP", SHTC3_temp,RAK_ATCMD_PERM_READ);
  api.system.atMode.add("HUM", "Return the humidity value with 1% resolution", "HUM", SHTC3_humi,RAK_ATCMD_PERM_READ);

  api.system.atMode.add("KX023", "Return the status of the KX023 sensor. 1 if available.", "KX023", KX023_init,RAK_ATCMD_PERM_READ);
  api.system.atMode.add("AX", "Return the value of X acceleration with 0.01G resolution", "AX", KX023_AX,RAK_ATCMD_PERM_READ);
  api.system.atMode.add("AY", "Return the value of Y acceleration with 0.01G resolution", "AY", KX023_AY,RAK_ATCMD_PERM_READ);
  api.system.atMode.add("AZ", "Return the value of Z acceleration with 0.01G resolution", "AZ", KX023_AZ,RAK_ATCMD_PERM_READ);

  api.system.atMode.add("LTR", "Return the status of the LTR-303 sensor. 1 if available.", "LTR", LTR_init,RAK_ATCMD_PERM_READ);
  api.system.atMode.add("LUMCH0", "Return the CHANNEL0 value of the LTR-303 sensor", "LUMCH0", LTR_ch0,RAK_ATCMD_PERM_READ);
  api.system.atMode.add("LUMCH1", "Return the CHANNEL1 value of the LTR-303 sensor", "LUMCH1", LTR_ch1,RAK_ATCMD_PERM_READ);
  api.system.atMode.add("LUM", "Return the CHANNEL1 value of the LTR-303 sensor", "LUM", LTR_lux,RAK_ATCMD_PERM_READ);

  api.system.atMode.add("GPS", "Return the status of the GNSS module. 1 if available.", "GPS", GPS_STT,RAK_ATCMD_PERM_READ);
  api.system.atMode.add("GPSON", "Return the GNSS module power | =1 : GNSS ON | =0 : GNSS OFF", "GPSON", GPS_on_off,RAK_ATCMD_PERM_WRITE);
  api.system.atMode.add("GPSPWR", "Return the GNSS module power | =1 : GNSS ON | =0 : GNSS OFF", "GPSPWR", GPS_on_off,RAK_ATCMD_PERM_WRITE);
  api.system.atMode.add("GPSSAT", "Return the number of available satellite(s)", "GPSSAT", GPS_sat,RAK_ATCMD_PERM_READ);
  api.system.atMode.add("GPSTIME", "Return GPS time in EPOCH format", "GPSTIME", GPS_time,RAK_ATCMD_PERM_READ);
  api.system.atMode.add("GPSLAT", "Return latitude | Return 0 if coordinate is not available", "GPSLAT", GPS_lat,RAK_ATCMD_PERM_READ);
  api.system.atMode.add("GPSLON", "Return longtitude | Return 0 if coordinate is not available", "GPSLON", GPS_lon,RAK_ATCMD_PERM_READ);
  api.system.atMode.add("GPSALT", "Return altitude | Return 0 if coordinate is not available", "GPSALT", GPS_alt,RAK_ATCMD_PERM_READ);
  api.system.atMode.add("GPSNMEA", "Activate the NMEA log from GNSS module | =1 : NMEA ON | =0 : NMEA OFF", "GPSON", GPS_nmea,RAK_ATCMD_PERM_WRITE);
  api.system.atMode.add("GPSLOG", "Activate the results from GNSS module every 5s | =1 : LOG ON | =0 : LOG OFF", "GPSLOG", GPS_log,RAK_ATCMD_PERM_WRITE);
  api.system.atMode.add("GPSDC", "Set GNSS module in duty cycle mode with 30sec sleep | =1 : DC ON | =0 : DC OFF", "GPSDC", GPS_dc,RAK_ATCMD_PERM_WRITE);
  api.system.atMode.add("GPSCONST", "Activate Galileo, Beidu, GPS and Glonass constellations", "GPSCONST", GPS_const,RAK_ATCMD_PERM_READ);
  api.system.atMode.add("GPSFIX", "Return the fix status of the GNSS module. 1 if the module get a fix, 0 else.", "GPSFIX", GPS_fix,RAK_ATCMD_PERM_READ);

  api.system.atMode.add("BAT", "Return battery voltage in mV | Return 0 if not available", "BAT", battery,RAK_ATCMD_PERM_READ);
  api.system.atMode.add("LDO", "Return LDO voltage in mV | Return 0 if not available", "LDO", ldo_read,RAK_ATCMD_PERM_READ);
}

void loop()
{
    
  if (GPS_EN == 1 && Serial1.available())
  {

    if(nmea.isValid() && is_fixing){
      t2f = millis()- getGPSPrevMillis; 
      is_fixing=0;
    }

    if (log_gps==1){
      currentMillis = millis();  
      // Print sensor & gps data
      if (currentMillis - getSensorDataPrevMillis > DATA_INTERVAL){
        getSensorDataPrevMillis = currentMillis;
        // GPS
        GPS_showData(); // print GPS info
        print_line(); // print line
      }
    }

    revString = Serial1.readStringUntil(0x0D);
    //Serial.println(_revString);
    len = revString.length() + 1;
    revString.toCharArray(revChar, len);
    for (int i = 0; i < len; i++) {
      if(nmea_gps){
      Serial.print(*(revChar + i));
      Serial1.write(Serial.read());   // read Serial and send it out Serial1      
      }
      nmea.process(*(revChar + i));
    }

  } 
  delay(20); // wait 20ms  
}


unsigned long unixTimestamp(int year, int month, int day, int hour, int min, int sec)
{
  const short days_since_beginning_of_year[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
  int leap_years = ((year - 1) - 1968) / 4 - ((year - 1) - 1900) / 100 + ((year - 1) - 1600) / 400;
  long days_since_1970 = (year - 1970) * 365 + leap_years + days_since_beginning_of_year[month - 1] + day - 1;
  if ((month > 2) && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)))
    days_since_1970 += 1; /* +leap day, if year is a leap year */
  return sec + 60 * (min + 60 * (hour + 24 * days_since_1970));
}



void GPS_showData(void)
{ /*
  Serial.print("Valid fix: ");
  Serial.println(nmea.isValid() ? "yes" : "no");
  if(nmea.isValid()){ */
   Serial.print("Nav. system: ");
   if (nmea.getNavSystem())
      Serial.print(nmea.getNavSystem());
    else
      Serial.print("none");
    Serial.print(", Sat in view: ");
    Serial.print(nmea.getNumSatellites());

    Serial.print(", Time to Fix: ");
    Serial.println(t2f/1000);
    
    
    double latitude = nmea.getLatitude();
    double longitude = nmea.getLongitude();
    long alt;
    
    Serial.print("GPS position: ");
    Serial.print(latitude / 1.0e6, 4);
    Serial.print(", ");
    Serial.print(longitude / 1.0e6, 4);
     Serial.print(", ");
    if (nmea.getAltitude(alt)){
      Serial.print(alt / 1000., 3);}
      Serial.println();
      
    Serial.print("Speed: ");
    Serial.print(nmea.getSpeed() / 1000., 3);
    Serial.print(" Course: ");
    Serial.print(nmea.getCourse() / 1000., 3); 
    uint32_t unixt = unixTimestamp(nmea.getYear(), nmea.getMonth(), nmea.getDay(), nmea.getHour(), nmea.getMinute(), nmea.getSecond());
    Serial.print("  Unix time: ");
    Serial.println(unixt);      
}



void print_line(void)
{ 
Serial.print(it);
Serial.print("***********************");
Serial.print(it);
Serial.print("*************************");
Serial.println(it);
it++;

     }
