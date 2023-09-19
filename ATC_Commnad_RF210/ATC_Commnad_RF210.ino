#include "Adafruit_SHTC3.h"
#include <Kionix_KX023.h>
#include "Adafruit_LTR329_LTR303.h"
#include <MicroNMEA.h>

#define GPS_EN PA1
#define GPS_detected PA9
#define DATA_INTERVAL 5000 //ms
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
int timezone = 7 ;
int  year; int mon; int day; int hr; int minute; double sec;

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
  if (! shtc3.begin()) {
    return 0;
  }
  return 1;
}

int SHTC3_init(SERIAL_PORT port, char *cmd, stParam *param) {
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    sht_status = getSHTstatus();
    Serial.println(sht_status ? "1" : "0");
  } else {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}

int SHTC3_temp(SERIAL_PORT port, char *cmd, stParam *param) {
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    if (sht_status)
    {
      shtc3.getEvent(&humidity, &temp);
      Serial.print(temp.temperature); Serial.println(" degree C");
    } else {
      Serial.println("0");
    }
  } else {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}

int SHTC3_humi(SERIAL_PORT port, char *cmd, stParam *param) {
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    if (sht_status)
    {
      shtc3.getEvent(&humidity, &temp);
      Serial.print(int(humidity.relative_humidity));Serial.println("%");
    } else {
      Serial.println("0");
    }
  } else {
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
  }  else  {
    myIMU.configContinuousReading(LOWPOWER, RANGE_8G, DATARATE_100HZ);
    return 1;
  }
}

int KX023_init(SERIAL_PORT port, char *cmd, stParam *param) {
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    kx023_status = getKX023status();
    Serial.println(kx023_status ? "1" : "0");
  } else {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}

int KX023_AX(SERIAL_PORT port, char *cmd, stParam *param) {
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    if (kx023_status)
    {
      myIMU.readAcceleration(&kx_x, &kx_y, &kx_z);
      Serial.println(kx_x);
    } else {
      Serial.println("0");
    }

  } else {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}

int KX023_AY(SERIAL_PORT port, char *cmd, stParam *param) {
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    if (kx023_status)
    {
      myIMU.readAcceleration(&kx_x, &kx_y, &kx_z);
      Serial.println(kx_y);
    } else {
      Serial.println("0");
    }

  } else {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}

int KX023_AZ(SERIAL_PORT port, char *cmd, stParam *param) {
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    if (kx023_status)
    {
      myIMU.readAcceleration(&kx_x, &kx_y, &kx_z);
      Serial.println(kx_z);
    } else {
      Serial.println("0");
    }

  } else {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}

bool ltr_status;

bool getLTRstatus()
{
  if ( ! ltr.begin() ) {
    return 0;
  } else {
    ltr.setGain(LTR3XX_GAIN_1);
    ltr.setIntegrationTime(LTR3XX_INTEGTIME_50);
    ltr.setMeasurementRate(LTR3XX_MEASRATE_50);
    return 1;
  }
}

int LTR_init(SERIAL_PORT port, char *cmd, stParam *param) {
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    ltr_status = getLTRstatus();
    Serial.println(ltr_status ? "1" : "0");
  } else {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}

int LTR_ch0(SERIAL_PORT port, char *cmd, stParam *param) {
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    if (ltr_status)
    {
      if (ltr.newDataAvailable()) {
        valid = ltr.readBothChannels(visible_plus_ir, infrared);
        if (valid) {
          Serial.println(visible_plus_ir);
        }
      }
    } else {
      Serial.println("0");
    }

  } else {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}

int LTR_ch1(SERIAL_PORT port, char *cmd, stParam *param) {
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    if (ltr_status)
    {
      if (ltr.newDataAvailable()) {
        valid = ltr.readBothChannels(visible_plus_ir, infrared);
        if (valid) {
          Serial.println(infrared);
        }
      }
    } else {
      Serial.println("0");
    }

  } else {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}
bool GPS_status = 0;

int GPS_init(SERIAL_PORT port, char *cmd, stParam *param) {
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    quectel_getData(revString, revChar, len);
    Serial.println(GPS_status ? "1" : "0");
  } else {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}
//int GPS_on(SERIAL_PORT port, char *cmd, stParam *param) {
//  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
//    nmea.getNavSystem();
//    digitalWrite(GPS_EN, HIGH);
//  } else {
//    return AT_PARAM_ERROR;
//  }
//  return AT_OK;
//}
//
//int GPS_off(SERIAL_PORT port, char *cmd, stParam *param) {
//  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
//    digitalWrite(GPS_EN, LOW);
//  } else {
//    return AT_PARAM_ERROR;
//  }
//  return AT_OK;
//}
uint32_t on_off_gps;
int GPS_on_off(SERIAL_PORT port, char *cmd, stParam *param) {
    if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
        Serial.print(cmd);
        Serial.print("=");
        Serial.println(on_off_gps?"HIGH":"LOW");
    } else if (param->argc == 1) {
        for (int i = 0 ; i < strlen(param->argv[0]) ; i++) {
            if (!isdigit(*(param->argv[0]+i))) {
                return AT_PARAM_ERROR;
            }
        }
        on_off_gps = strtoul(param->argv[0], NULL, 10);
        if (on_off_gps != 0 && on_off_gps != 1) {
            return AT_PARAM_ERROR;
        }
        digitalWrite(GPS_EN, (on_off_gps == 1)?HIGH:LOW);
    } else {
        return AT_PARAM_ERROR;
    }
    return AT_OK;
}

int GPS_sat(SERIAL_PORT port, char *cmd, stParam *param) {
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    Serial.println(nmea.getNumSatellites());
  } else {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}

int GPS_time(SERIAL_PORT port, char *cmd, stParam *param) {
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    uint32_t unixt = unixTimestamp(nmea.getYear(), nmea.getMonth(), nmea.getDay(), nmea.getHour(), nmea.getMinute(), nmea.getSecond());
    Serial.println(unixt);
  } else {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}


int GPS_lat(SERIAL_PORT port, char *cmd, stParam *param) {
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    double latitude = nmea.getLatitude();
    Serial.println(latitude / 1.0e6, 4);
    //    } else {
    //      Serial.println("0");
    //    }

  } else {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}

int GPS_lon(SERIAL_PORT port, char *cmd, stParam *param) {
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    double longitude = nmea.getLongitude();
    Serial.println(longitude / 1.0e6, 4);
    //    } else {
    //      Serial.println("0");
    //    }

  } else {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}

int GPS_alt(SERIAL_PORT port, char *cmd, stParam *param) {
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    long alt;
    if (nmea.getAltitude(alt)) {
      Serial.println(alt / 1000., 3);
    } else {
      Serial.println("0");
    }

  } else {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}





int battery(SERIAL_PORT port, char *cmd, stParam *param) {
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    voltage_adc = (uint16_t)analogRead(LS_BATVOLT_PIN);
    voltage = (uint16_t)((LS_ADC_AREF / 1.024) * (LS_BATVOLT_R1 + LS_BATVOLT_R2) / LS_BATVOLT_R2 * (float)voltage_adc);
    Serial.println(voltage);
  } else {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}



void setup()
{
  Serial1.begin(9600);
  pinMode(GPS_EN, OUTPUT);
  pinMode(GPS_detected, OUTPUT);
  digitalWrite(GPS_detected, LOW);
  while (!Serial) {
    delay(10); // wait for serial port to open
  }

  api.system.atMode.add("SHT", "", "SHT", SHTC3_init, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("TEMP", "", "TEMP", SHTC3_temp,  RAK_ATCMD_PERM_READ);
  api.system.atMode.add("HUM", "", "HUM", SHTC3_humi, RAK_ATCMD_PERM_READ);

  api.system.atMode.add("KX023", "", "KX023", KX023_init, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("AX", "", "AX", KX023_AX, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("AY", "", "AY", KX023_AY, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("AZ", "", "AZ", KX023_AZ, RAK_ATCMD_PERM_READ);

  api.system.atMode.add("LTR", "", "LTR", LTR_init, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("LUMCH0", "", "LUMCH0", LTR_ch0, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("LUMCH1", "", "LUMCH1", LTR_ch1, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("LUM", "", "LUM", LTR_ch1, RAK_ATCMD_PERM_READ);

  api.system.atMode.add("GPS", "", "GPS", GPS_init, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("GPSON", "", "GPSON", GPS_on_off, RAK_ATCMD_PERM_WRITE | RAK_ATCMD_PERM_READ);
  api.system.atMode.add("GPSSAT", "", "GPSSAT", GPS_sat, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("GPSTIME", "", "GPSTIME", GPS_time, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("GPSLAT", "", "GPSLAT", GPS_lat, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("GPSLON", "", "GPSLON", GPS_lon, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("GPSALT", "", "GPSALT", GPS_alt, RAK_ATCMD_PERM_READ);

  api.system.atMode.add("BAT", "", "BAT", battery, RAK_ATCMD_PERM_READ);

}

void loop()
{

}

void quectel_getData(String _revString, char* _revChar, int _len)
{
  while (Serial1.available())
  {
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

unsigned long unixTimestamp(int year, int month, int day, int hour, int min, int sec) {
  const short days_since_beginning_of_year[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
  int leap_years = ((year - 1) - 1968) / 4
                   - ((year - 1) - 1900) / 100
                   + ((year - 1) - 1600) / 400;
  long days_since_1970 = (year - 1970) * 365 + leap_years
                         + days_since_beginning_of_year[month - 1] + day - 1;
  if ( (month > 2) && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) )
    days_since_1970 += 1; /* +leap day, if year is a leap year */
  return sec + 60 * ( min + 60 * (hour + 24 * days_since_1970) );
}
