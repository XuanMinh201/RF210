# RF210

RF210 firmware ATC
version 0.1.2

Supported sensors:
- [Digital humidity and temperature sensor - SHTC3](https://www.sensirion.com/products/catalog/SHTC3/)
- [Tri-Axis Accelerometer - KX023-1025](https://www.rohm.com/products/sensors-mems/accelerometer-ics/kx023-1025-product)
- [Ambient Light Sensor - LTR-303ALS-01](https://optoelectronics.liteon.com/en-global/Led/led-component/Detail/926)
- [GNSS LC76F](https://www.quectel.com/product/gnss-lc76f)

Command general format: ```ATC+<cmd>=?```

- ```ATC+VER=?``` : return version of the firmware

- ```ATC+SHT=?``` : return 1 if sensor is available, return 0 of not
- ```ATC+TEMP=?``` : return the value of temperature with 0.01° resolution, return 0 if not available
- ```ATC+HUM=?``` : return the value of humidity with 1% resolution, return 0 if not available

- ```ATC+KX023=?``` : return 1 if sensor is available, return 0 of not
- ```ATC+AX=?``` : return the value of X acceleration with 0.01G resolution, return 0 if not available
- ```ATC+AY=?``` : return the value of Y acceleration with 0.01G resolution, return 0 if not available
- ```ATC+AZ=?``` : return the value of Z acceleration with 0.01G resolution, return 0 if not available

- ```ATC+LTR=?``` : return 1 if sensor is available, return 0 of not
- ```ATC+LUMCH0=?``` : return the value of CH0, return 0 if not available
- ```ATC+LUMCH1=?``` : return the value of CH1, return 0 if not available
- ```ATC+LUM=?``` : return the value of CH1, return 0 if not available

- ```ATC+GPS=?``` : return 1 if sensor is available, return 0 of not
- ```ATC+GPSFIX=?``` : return 1 if GNSS get a fix, return 0 of not
- ```ATC+GPSON=<1/0>``` : Turn ON/OFF GPS LDO, return HIGH/LOW 
- ```ATC+GPSSAT=?``` : return number of satellite available
- ```ATC+GPSTIME=?``` : return GPS time in EPOCH format, 0 if not available
- ```ATC+GPSLAT=?``` : return GPS Latitude, 0 if not available
- ```ATC+GPSLON=?``` : return GPS Longitude, 0 if not available
- ```ATC+GPSALT=?``` : return GPS Altitude, 0 if not available
- ```ATC+GPSNMEA=<1/0>``` : Activate NMEA log from GNSS module
- ```ATC+GPSLOG=<1/0>``` : Activate GNSS Log from GNSS module
- ```ATC+GPSDC=<1/0>``` : Set GNSS module in duty cycle mode with 30sec sleep

- ```ATC+BAT=?``` : return battery voltage in mv, 0 if not available
- ```ATC+LDO=?``` : return LDO voltage on Rak3172 in mv, 0 if not available
  
## Getting Started

### Hardware

- USB to UART Converter
- RFThings RF210 Board

### Sortware

- Arduino IDE (version v1.8.13 or above is recommended)
- RUI3 lastest firmware for RAK3172: [RAK3172-E_latest_final.hex](https://downloads.rakwireless.com/RUI/RUI3/Image/RAK3172-E_latest_final.hex)
- (STM32CubeProgammer)[https://www.st.com/en/development-tools/stm32cubeprog.html]
  
### Additional Libraries

- Adafruit_SHTC3.h
- Adafruit_LTR329_LTR303.h
- MicroNMEA.h
- Kionix_KX023.h
  
## Installation

### Hardware connection:

<!-- ![image](https://github.com/XuanMinh201/RF210/assets/75436464/ea7faa22-7082-44b6-a6ea-442fadfd687f) -->
<img src="https://github.com/XuanMinh201/RF210/assets/75436464/ea7faa22-7082-44b6-a6ea-442fadfd687f" height="450">

### On the RF210 board:

<!-- ![image](https://github.com/XuanMinh201/RF210/assets/75436464/9c0f2eca-0ba7-4fdb-ba18-9b7516c0776f) -->
<img src="https://github.com/XuanMinh201/RF210/assets/75436464/9c0f2eca-0ba7-4fdb-ba18-9b7516c0776f" height="450">

### In STM32CubeProgammer:
  -  Hold the **B_RAK (boot)** button and press **R_RAK (reset)** button and release the **B_RAK (boot)** button to enter bootmode.
  -  Select UART, Baudrate 115200 and press Connect.
  -  Open RAK3172-E_latest_final.hex
  -  Select the address as in following image if needed
  -  Press Download to upload firmwave
  -  After download success, press **R_RAK (reset)** button to exit the bootmode
  
<!-- ![image](https://github.com/XuanMinh201/RF210/assets/75436464/55f5c5ab-d69a-4a25-94da-563d1e52a172) -->
<img src="https://github.com/XuanMinh201/RF210/assets/75436464/55f5c5ab-d69a-4a25-94da-563d1e52a172" height="450">
 

### In Arduino IDE:
  -  Add this JSON in Additional Boards Manager URLs [\(Show me how?\)](https://support.arduino.cc/hc/en-us/articles/360016466340-Add-third-party-platforms-to-the-Boards-Manager-in-Arduino-IDE):

```  
https://raw.githubusercontent.com/RAKWireless/RAKwireless-Arduino-BSP-Index/main/package_rakwireless.com_rui_index.json
```

  -  Go to **Tool -> Board -> Boards Manager**, search & install **RAKwireless RUI STM32 Boards**
  -  Open ```ATC_Command_RF210.ino``` sketch, seletc **WisDou RAK3172-T Board** from **Tool** menu
  -  Plug in your board and upload


<!-- ![image](https://github.com/XuanMinh201/RF210/assets/75436464/141710ed-1294-46ea-9951-63bea73622ed) -->
<img src="https://github.com/XuanMinh201/RF210/assets/75436464/141710ed-1294-46ea-9951-63bea73622ed" height="450">

  -  Select **Tool -> Board -> RAKwireless RUI STM32 Modules -> WisDuo RAK3172 Evaluation Board**
    
<!-- ![image](https://github.com/XuanMinh201/RF210/assets/75436464/146c570a-ec82-45bc-ada0-89544624b861) -->
<img src="https://github.com/XuanMinh201/RF210/assets/75436464/146c570a-ec82-45bc-ada0-89544624b861" height="450">

