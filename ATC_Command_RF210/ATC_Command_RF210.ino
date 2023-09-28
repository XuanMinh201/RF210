/**
 * @file ATC_Command_RF210.ino
 * @author XuanMinh201
 *
 * @brief This sketch add custom ATC command to RFThings RF210 board. These commands help controlling on-board sensors,
 * GNSS & measuring battery level. For detail description, please visit: https://github.com/XuanMinh201/RF210
 *
 * @version 0.0.1
 * @date 2023-09-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "Adafruit_SHTC3.h"         // http://librarymanager/All#Adafruit_SHTC3
#include <Kionix_KX023.h>           // TO-DO: Add this original
#include "Adafruit_LTR329_LTR303.h" // http://librarymanager/All#Adafruit_LTR329_LTR303
#include <MicroNMEA.h>              // http://librarymanager/All#SparkFun_Ublox

#define GPS_EN PA1
#define GPS_detected PA9
#define DATA_INTERVAL 5000 // ms
#define LS_ADC_AREF 3.0f
#define LS_BATVOLT_R1 1.0f
#define LS_BATVOLT_R2 2.0f
#define LS_BATVOLT_PIN PA15

Adafruit_LTR303 ltr = Adafruit_LTR303();
Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();
KX023 myIMU;

uint16_t voltage_adc;
uint16_t voltage;
float kx_x, kx_y, kx_z;
int humi, temper;
sensors_event_t humidity, temp;
bool valid;
uint16_t visible_plus_ir, infrared;
int timezone = 7;
int year;
int mon;
int day;
int hr;
int minute;
double sec;

char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
String revString;
char revChar[100];
int len = 0;

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

int SHTC3_init(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    Serial.print(cmd);
    Serial.print("=");
    sht_status = getSHTstatus();
    Serial.println(sht_status ? "1" : "0");
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}

int SHTC3_temp(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    Serial.print(cmd);
    Serial.print("=");
    if (sht_status)
    {
      shtc3.getEvent(&humidity, &temp);
      Serial.print(temp.temperature);
      Serial.println(" degree C");
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
  return AT_OK;
}

int SHTC3_humi(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    Serial.print(cmd);
    Serial.print("=");
    if (sht_status)
    {
      shtc3.getEvent(&humidity, &temp);
      Serial.print(int(humidity.relative_humidity));
      Serial.println("%");
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
  return AT_OK;
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
    Serial.print(cmd);
    Serial.print("=");
    kx023_status = getKX023status();
    Serial.println(kx023_status ? "1" : "0");
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}

int KX023_AX(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    Serial.print(cmd);
    Serial.print("=");
    if (kx023_status)
    {
      myIMU.readAcceleration(&kx_x, &kx_y, &kx_z);
      Serial.println(kx_x);
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
  return AT_OK;
}

int KX023_AY(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    Serial.print(cmd);
    Serial.print("=");
    if (kx023_status)
    {
      myIMU.readAcceleration(&kx_x, &kx_y, &kx_z);
      Serial.println(kx_y);
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
  return AT_OK;
}

int KX023_AZ(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    Serial.print(cmd);
    Serial.print("=");
    if (kx023_status)
    {
      myIMU.readAcceleration(&kx_x, &kx_y, &kx_z);
      Serial.println(kx_z);
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
  return AT_OK;
}

bool ltr_status;

bool getLTRstatus()
{
  if (!ltr.begin())
  {
    return 0;
  }
  else
  {
    ltr.setGain(LTR3XX_GAIN_1);
    ltr.setIntegrationTime(LTR3XX_INTEGTIME_50);
    ltr.setMeasurementRate(LTR3XX_MEASRATE_50);
    return 1;
  }
}

int LTR_init(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    Serial.print(cmd);
    Serial.print("=");
    ltr_status = getLTRstatus();
    Serial.println(ltr_status ? "1" : "0");
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}

int LTR_ch0(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    Serial.print(cmd);
    Serial.print("=");
    if (ltr_status)
    {
      if (ltr.newDataAvailable())
      {
        valid = ltr.readBothChannels(visible_plus_ir, infrared);
        if (valid)
        {
          Serial.println(visible_plus_ir);
        }
      }
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
  return AT_OK;
}

int LTR_ch1(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    Serial.print(cmd);
    Serial.print("=");
    if (ltr_status)
    {
      if (ltr.newDataAvailable())
      {
        valid = ltr.readBothChannels(visible_plus_ir, infrared);
        if (valid)
        {
          Serial.println(infrared);
        }
      }
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
  return AT_OK;
}
bool GPS_status = 0;

int GPS_init(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    Serial.print(cmd);
    Serial.print("=");
    quectel_getData(revString, revChar, len);
    Serial.println(GPS_status ? "1" : "0");
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}
// int GPS_on(SERIAL_PORT port, char *cmd, stParam *param) {
//   if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0))) {
//     nmea.getNavSystem();
//     digitalWrite(GPS_EN, HIGH);
//   } else {
//     return AT_PARAM_ERROR;
//   }
//   return AT_OK;
// }
//
// int GPS_off(SERIAL_PORT port, char *cmd, stParam *param) {
//   if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0))) {
//     digitalWrite(GPS_EN, LOW);
//   } else {
//     return AT_PARAM_ERROR;
//   }
//   return AT_OK;
// }
uint32_t on_off_gps;
int GPS_on_off(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    Serial.print(cmd);
    Serial.print("=");
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
  return AT_OK;
}

int GPS_sat(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    Serial.print(cmd);
    Serial.print("=");
    Serial.println(nmea.getNumSatellites());
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}

int GPS_time(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    Serial.print(cmd);
    Serial.print("=");
    uint32_t unixt = unixTimestamp(nmea.getYear(), nmea.getMonth(), nmea.getDay(), nmea.getHour(), nmea.getMinute(), nmea.getSecond());
    Serial.println(unixt);
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}

int GPS_lat(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    Serial.print(cmd);
    Serial.print("=");
    double latitude = nmea.getLatitude();
    Serial.println(latitude / 1.0e6, 4);
    //    } else {
    //      Serial.println("0");
    //    }
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}

int GPS_lon(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    Serial.print(cmd);
    Serial.print("=");
    double longitude = nmea.getLongitude();
    Serial.println(longitude / 1.0e6, 4);
    //    } else {
    //      Serial.println("0");
    //    }
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}

int GPS_alt(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    Serial.print(cmd);
    Serial.print("=");
    long alt;
    if (nmea.getAltitude(alt))
    {
      Serial.println(alt / 1000., 3);
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
  return AT_OK;
}

int battery(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    Serial.print(cmd);
    Serial.print("=");
    voltage_adc = (uint16_t)analogRead(LS_BATVOLT_PIN);
    voltage = (uint16_t)((LS_ADC_AREF / 1.024) * (LS_BATVOLT_R1 + LS_BATVOLT_R2) / LS_BATVOLT_R2 * (float)voltage_adc);
    Serial.println(voltage);
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}

void setup()
{
  Serial.begin(115200,RAK_AT_MODE);
  Serial1.begin(9600,RAK_CUSTOM_MODE);
  pinMode(GPS_EN, OUTPUT);
  pinMode(GPS_detected, INPUT);
  
  while (!Serial)
  {
    delay(10); // wait for serial port to open
  }

  api.system.atMode.add("SHT", "This command gets the status of the SHTC3 sensor. 1 if available.", "SHT", SHTC3_init);
  api.system.atMode.add("TEMP", "This command gets the temperature value with 0.01° resolution", "TEMP", SHTC3_temp);
  api.system.atMode.add("HUM", "This command gets the humidity value with 1% resolution", "HUM", SHTC3_humi);

  api.system.atMode.add("KX023", "This command gets the status of the KX023 sensor. 1 if available.", "KX023", KX023_init);
  api.system.atMode.add("AX", "This command gets the value of X acceleration with 0.01G resolution", "AX", KX023_AX);
  api.system.atMode.add("AY", "This command gets the value of Y acceleration with 0.01G resolution", "AY", KX023_AY);
  api.system.atMode.add("AZ", "This command gets the value of Z acceleration with 0.01G resolution", "AZ", KX023_AZ);

  api.system.atMode.add("LTR", "This command gets the status of the LTR-303 sensor. 1 if available.", "LTR", LTR_init);
  api.system.atMode.add("LUMCH0", "This command gets the CHANNEL0 value of the LTR-303 sensor", "LUMCH0", LTR_ch0);
  api.system.atMode.add("LUMCH1", "This command gets the CHANNEL1 value of the LTR-303 sensor", "LUMCH1", LTR_ch1);
  api.system.atMode.add("LUM", "This command gets the CHANNEL1 value of the LTR-303 sensor", "LUM", LTR_ch1);

  api.system.atMode.add("GPS", "This command gets the status of the GNSS module. 1 if available.", "GPS", GPS_init);
  api.system.atMode.add("GPSON", "This command sets the GNSS module power | =1 : GNSS ON | =0 : GNSS OFF", "GPSON", GPS_on_off);
  api.system.atMode.add("GPSSAT", "This command gets the number of available satellite(s)", "GPSSAT", GPS_sat);
  api.system.atMode.add("GPSTIME", "This command gets the GPS time in EPOCH format", "GPSTIME", GPS_time);
  api.system.atMode.add("GPSLAT", "This command gets the current latitude | Return 0 if coordinate is not available", "GPSLAT", GPS_lat);
  api.system.atMode.add("GPSLON", "This command gets the current longtitude | Return 0 if coordinate is not available", "GPSLON", GPS_lon);
  api.system.atMode.add("GPSALT", "This command gets the current altitude | Return 0 if coordinate is not available", "GPSALT", GPS_alt);

  api.system.atMode.add("BAT", "This command gets the battery voltage in mV | Return 0 if not available", "BAT", battery);
}

void loop()
{
}

void quectel_getData(String _revString, char *_revChar, int _len)
{
  int count_gps = 0;
  while (Serial1.available() && count_gps <50 )
  {
    count_gps++;
    _revString = Serial1.readStringUntil(0x0D);
    //Serial.println(_revString);
    _len = _revString.length() + 1;
    _revString.toCharArray(_revChar, _len);
    for (int i = 0; i < _len; i++) {
      // Serial2.print(*(_revChar + i));
      nmea.process(*(_revChar + i));
    }
  }
  quectelDelayTime = 5;
  GPS_status = 1;
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
