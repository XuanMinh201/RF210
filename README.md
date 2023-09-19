# RF210

RF210 firmware ATC 

Support SenSor: SHTC3, KX023, LTR303, LC76F
Format: ATC+<cmd>=?

- ATC+SHT? : return 1 if sensor is available, return 0 of not
- ATC+TEMP? : return the value of temperature with 0.01° resolution, return 0 if not available
- ATC+HUM? return the value of humidity with 1% resolution, return 0 if not available

- ATC+KX023? : return 1 if sensor is available, return 0 of not
- ATC+AX? : return the value of X acceleration with 0.01G resolution, return 0 if not available
- ATC+AY? : return the value of Y acceleration with 0.01G resolution, return 0 if not available
- ATC+AZ? : return the value of Z acceleration with 0.01G resolution, return 0 if not available

- ATC+LTR? : return 1 if sensor is available, return 0 of not
- ATC+LUMCH0? : return the value of CH0, return 0 if not available
- ATC+LUMCH1? : return the value of CH1, return 0 if not available
- ATC+LUM? : return the value of CH1, return 0 if not available

- ATC+GPS? : return 1 if sensor is available, return 0 of not
- ATC+GPSON=<1/0> : Turn ON/OFF GPS LDO, return HIGH/LOW 
- ATC+GPSSAT? : return number of satellite available
- ATC+GPSTIME? : return GPS time in EPOCH format, 0 if not available
- ATC+GPSLAT? : return GPS Latitude, 0 if not available
- ATC+GPSLON? : return GPS Longitude, 0 if not available
- ATC+GPSALT? : return GPS Altitude, 0 if not available

- ATC+BAT? : return battery voltage in mv, 0 if not available
  
## Prepare
### Hardware
- USB UART
- RF210 Board
### Sortware
- Arduino IDE (my version 1.8.19)
- RUI 3 RAK3172-E_latest_final.hex
- STM32CubeProgammer (my version 2.9.0)
### Additional Libraries
- Adafruit_SHTC3.h
- Adafruit_LTR329_LTR303.h
- MicroNMEA.h
- Kionix_KX023.h
## Installation
- Hardware connection:
  - ![image](https://github.com/XuanMinh201/RF210/assets/75436464/ea7faa22-7082-44b6-a6ea-442fadfd687f)
  - On Borad RF210
  - ![image](https://github.com/XuanMinh201/RF210/assets/75436464/9c0f2eca-0ba7-4fdb-ba18-9b7516c0776f)

- STM32CubeProgammer:
  -  Hold down the B_RAK (boot) button and press R_RAK(reset) and release the B_RAK(boot) button to enter boot mode.
  -  Select UART, Baudrate 115200 and press Connect.
  -  Open RAK3172-E_latest_final.hex
  -  ![image](https://github.com/XuanMinh201/RF210/assets/75436464/55f5c5ab-d69a-4a25-94da-563d1e52a172)
  -  Press Download to Upload firmwave. After, press button Reset to logout bootmode
  -  
- Arduino:
  -  install in Preferences https://raw.githubusercontent.com/RAKWireless/RAKwireless-Arduino-BSP-Index/main/package_rakwireless.com_rui_index.json
  -  On Boards Manager install RAKwireless RUI STM32 Boards
  -  ![image](https://github.com/XuanMinh201/RF210/assets/75436464/141710ed-1294-46ea-9951-63bea73622ed)
  -  Open ATC_Command_RF210 and upload code

