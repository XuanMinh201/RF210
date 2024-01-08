// ESP32 C3 SERIAL1 (second UART)
HardwareSerial mySerial1(1);

//#define SLEEP    1      // Define if Sleep mode is activated

#define USB    1      // Define if Sleep mode is activated

int rxPin = 20;
int txPin = 21;

int period = 90; // period to send LoRaWAN packet
int rx_delay = 0;
int8_t DC=0;

String devAddr = "260B8F3A";
String nwkkey = "784E918B1A403743889FB445E8261895";
String appskey = "BF2E0A2E28B5E5378408C3864C138A57";

// String devAddr = "260B09E2";
// String nwkkey = "305F6DCC666FB67850D45B0344A6ABB2";
// String appskey = "7D1F681C95D309091CF236A9DFF6D6ED";


void setup()
{
  
  #if USB
  Serial.begin(115200);
  delay(1000);
   
   Serial.println("ABP tracker");
  #endif
  
  pinMode(txPin, OUTPUT);
  pinMode(rxPin, INPUT);

  pinMode(10, OUTPUT); //Rak enable
  pinMode(4, OUTPUT); // LED


  digitalWrite(4, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(4, LOW);    // turn the LED off by making the voltage LOW
   
mySerial1.begin(115200, SERIAL_8N1, rxPin, txPin);
  digitalWrite(10, HIGH); // Switch on RAK
    delay(1000);
  


 #if USB
  Serial.println("Setup at command");
    #endif
  //mySerial1.println("ATR"); // Set ABP
  mySerial1.println("AT+NJM=0"); // Set ABP
 // delay(300);
  mySerial1.println("AT+BAND=4");// Set EU868 frequency band
//  delay(300);
  mySerial1.println("AT+DR=5");// Set EU868 frequency band
 // delay(300);
 
  mySerial1.print("AT+DEVADDR=");
  mySerial1.println(devAddr);
  //delay(300);
    mySerial1.print("AT+NWKSKEY=");
  mySerial1.println(nwkkey);
 // delay(300);  
  mySerial1.print("AT+APPSKEY=");
  mySerial1.println(appskey);
 // delay(300);
  while (mySerial1.available() == 0)
{
   #if USB
Serial.println("Waiting");
#endif
delay(200);
  }

//flush_serial_AT(false); 
mySerial1.println("ATC+GPSON=1"); // Activate GNSS
delay(3000);
//flush_serial_AT(false);
mySerial1.println("ATC+GPSDC=1"); // Activate GNSS
//flush_serial_AT(false);
 delay(200); 
 blink();
}

void loop()
{

flush_serial_AT(false);
mySerial1.println("ATC+SENDSENSORS"); // Activate GNSS

blink();
delay(1000);

mySerial1.readStringUntil('\n');

while (mySerial1.available() == 0)
{
rx_delay=rx_delay+100;
delay(100);
}

 #if USB
Serial.print("Rx delay : ");
Serial.println(rx_delay);
#endif

   if (mySerial1.available())
  { // If anything comes in Serial1 (pins 4 & 5)
    while (mySerial1.available()){
     #if USB
      Serial.write(mySerial1.read()); // read it and send it out Serial (USB)
      #endif
    }
  }
   #if USB
  Serial.print("AT set complete with downlink : ");
  
  #endif
  int sleep_time=period*1000 ; //- rx_delay; // Sleep duration in ms
  rx_delay=0;
  sleep (sleep_time, 0); 
}



void array_to_string(byte array[], unsigned int len, char buffer[])
{
    for (unsigned int i = 0; i < len; i++)
    {
        byte nib1 = (array[i] >> 4) & 0x0F;
        byte nib2 = (array[i] >> 0) & 0x0F;
        buffer[i*2+0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
        buffer[i*2+1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
    }
    buffer[len*2] = '\0';
}

 // Return temperature level in degree
 float measure_temp(){

//Serial.flush();
flush_serial_AT(false);// flush AT Serial reading buffer
  
mySerial1.println("ATC+TEMP=?"); // Request bat value
 String temperature;
  while (mySerial1.available() == 0)
{
delay(300);
  }


 if(mySerial1.available()){
        temperature = mySerial1.readStringUntil('\n');
         #if USB
        Serial.print("Temperature:");
        Serial.println(temperature);
        #endif
 }
 
return temperature.toFloat();
 }

 // Return humidity level in percent
 float measure_hum(){

//Serial.flush();
flush_serial_AT(false);// flush AT Serial reading buffer
  
mySerial1.println("ATC+HUM=?"); // Request bat value
 String hum;
  while (mySerial1.available() == 0)
{
delay(300);
  }
 

 if(mySerial1.available()){
        hum = mySerial1.readStringUntil('\n');
         #if USB
        Serial.print("Humidity:");
        Serial.println(hum);
        #endif
 }
 
return hum.toFloat();
 }

// Return humidity level in percent
 float measure_lum(){

//Serial.flush();
flush_serial_AT(false);// flush AT Serial reading buffer
  
mySerial1.println("ATC+LUM=?"); // Request bat value
 String lum;
  while (mySerial1.available() == 0)
{
delay(300);
  }
 delay(100);

 if(mySerial1.available()){
        lum = mySerial1.readStringUntil('\n');
         #if USB
        Serial.print("Luminosity:");
        Serial.println(lum);
         #endif
 }
 
return lum.toFloat();
 } 

// Return bat level in mv
 float measure_bat(){

//Serial.flush();
flush_serial_AT(false);// flush AT Serial reading buffer
  
mySerial1.println("ATC+BAT=?"); // Request bat value
 String bat;
  while (mySerial1.available() == 0)
{
delay(300);
  }
 delay(100);

 if(mySerial1.available()){
        bat = mySerial1.readStringUntil('\n');
         #if USB
        Serial.print("Bat:");
        Serial.println(bat);
        #endif
 }
 
return bat.toFloat();
 } 

  // Return Acceleration level in G
 float measure_acc(int axis){

//Serial.flush();
flush_serial_AT(false);// flush AT Serial reading buffer

if(axis==1){  
mySerial1.println("ATC+AX=?"); // Request bat value
}
else if(axis==2){  
mySerial1.println("ATC+AY=?"); // Request bat value
}
else if(axis==3){  
mySerial1.println("ATC+AZ=?"); // Request bat value
}
 String a;
  while (mySerial1.available() == 0)
{
delay(300);
  }
 delay(100);
 if(mySerial1.available()){
        a = mySerial1.readStringUntil('\n');
         #if USB
        Serial.print("Acc:");
        Serial.println(a);
        #endif
 }
 
return a.toFloat();
 }

void flush_serial_AT(bool print){

   if (mySerial1.available())
  { // If anything comes in Serial1 (pins 4 & 5)
    while (mySerial1.available())
      if(print) {
        #if USB
        Serial.write(mySerial1.read()); // read it and send it out Serial (USB)
         #endif
      }
      else {
      mySerial1.read();}
  }
  delay(100);
}


void blink(){
digitalWrite(4, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);                       // wait for a second
  digitalWrite(4, LOW);    // turn the LED off by making the voltage LOW
} 

// Sleep function over a period in ms
void sleep(int period, int mode) {

if(mode==0){

#if USB
  Serial.println("Move to sleep");
#endif

    blink();
   mySerial1.println("ATC+SLEEP"); // Set Sleep for RAK3172
   //mySerial1.println(period); // Set Sleep
   //delay(1000);
//blink();
  //digitalWrite(10, LOW); // Switch off RAK
  delay(100);
blink(); 

 #if USB
  Serial.end(); 
  #endif

  mySerial1.flush();
  mySerial1.end();

   //gpio_hold_en((gpio_num_t)10);
   //esp_sleep_enable_ext0_wakeup(WAKEPIN,0); //1 = Low to High, 0 = High to Low. Pin pulled HIGH
 
  //gpio_light_sleep_hold_en();
  gpio_deep_sleep_hold_en();
  // wake up 1 second later and then go into deep sleep
  esp_sleep_enable_timer_wakeup(period*1000); // in us
  delay(100);
  esp_light_sleep_start();
  delay(500);
 

  
 #if USB
  Serial.begin(115200);
  #endif
 mySerial1.begin(115200, SERIAL_8N1, rxPin, txPin);
//mySerial1.println("ATC+GPSON=1"); // Activate GNSS

 mySerial1.println("AT"); // Set Sleep for RAK3172
  flush_serial_AT(false); 

 //digitalWrite(10, HIGH); // Switch on RAK
   blink();
   //delay(2000);

}

else {

#if USB
  Serial.print("Stop during : ");
  Serial.println(period/1000);
  #endif
  // Sleep during period
  delay(period);

}

}

  // Return 1:lat 2:long 3:alt 4:sat from GNSS
 float measure_gnss(int axis){

//Serial.flush();
flush_serial_AT(false);// flush AT Serial reading buffer

if(axis==1){  
mySerial1.println("ATC+GPSLAT=?"); // Request lat value
 #if USB
Serial.print("Lat:");
#endif
}
else if(axis==2){  
mySerial1.println("ATC+GPSLON=?"); // Request lon value
 #if USB
Serial.print("Long:");
#endif
}
else if(axis==3){  
mySerial1.println("ATC+GPSALT=?"); // Request alt value
 #if USB
Serial.print("Altitude:");
#endif
}
else if(axis==4){  
mySerial1.println("ATC+GPSSAT=?"); // Request sat value
 #if USB
Serial.print("Sat:");
#endif
}
else if(axis==5){  
mySerial1.println("ATC+GPSPWR=?"); // Request sat value
 #if USB
Serial.print("On:");
#endif
}

 String a;
 while (mySerial1.available() == 0)
{
delay(300);
  }



if (mySerial1.available())
  { // If anything comes in Serial1 (pins 4 & 5)
  a = mySerial1.readStringUntil('\n');  
   #if USB  
  Serial.println(a);
  #endif
  }

  if(a.toFloat() > 5  && DC==0)
  {
 mySerial1.println("ATC+GPSDC=1"); // Activate GNSS
DC=1;
  }
  if(a.toFloat() < 5  && DC==1)
  {
 mySerial1.println("ATC+GPSDC=0"); // Activate GNSS
DC=0;
  }

 
return a.toFloat();
 }
