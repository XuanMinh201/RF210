#include <Wire.h>

#define ESP_SDA 5
#define ESP_SCL 6

HardwareSerial mySerial1(1);

int rxPin = 20;
int txPin = 21;

void setup()
{
  Serial.begin(115200);
  Wire.begin(ESP_SDA, ESP_SCL);
  pinMode(ESP_SDA, INPUT_PULLUP);
  pinMode(ESP_SCL, INPUT_PULLUP);

  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  pinMode(rxPin, INPUT);
  pinMode(7, OUTPUT);
  pinMode(10, OUTPUT);
  mySerial1.begin(115200, SERIAL_8N1, rxPin, txPin);
  digitalWrite(10, HIGH);
//  i2c_set_pin
  delay(2000);
  mySerial1.println("ATE");
  delay(200);

  mySerial1.println("ATR");
  delay(2000);
  
  mySerial1.println("ATC+SHT=?");
  delay(200);
  mySerial1.println("ATC+TEMP=?");
  delay(200);
  mySerial1.println("ATC+HUM=?");
  delay(200);
  mySerial1.println("ATC+KX023=?");
  delay(200);
  mySerial1.println("ATC+AX=?");
  delay(200);
  mySerial1.println("ATC+AY=?");
  delay(200);
  mySerial1.println("ATC+AZ=?");
  delay(200);
  mySerial1.println("ATC+LTR=?");
  delay(200);
  
  

  if (mySerial1.available())
  { 
    while (mySerial1.available())
      Serial.write(mySerial1.read()); 
  }
  
  Serial.println("setup at command");
  mySerial1.println("AT+NWM=1");
  delay(500);
  mySerial1.println("AT+NJM=0");
  delay(500);
  mySerial1.println("AT+CLASS=A");
  delay(500);
  mySerial1.println("AT+BAND=9");
  delay(500);
  mySerial1.println("AT+DEVADDR=260B7A00");
  delay(500);
  mySerial1.println("AT+APPSKEY=00B8A1F69127D4DD9E5D9886AD812C2D");
  delay(500);
  mySerial1.println("AT+NWKSKEY=28BA96253515E5AC94C533297C2D2070");
  delay(500);
  mySerial1.println("AT+JOIN=1:1:10:8");
  delay(10000);


}

void loop()
{
  mySerial1.println("AT+SEND=3:68656c6c6f"); //hello
  delay(3300);
  if (mySerial1.available())
  { 
    while (mySerial1.available())
      Serial.write(mySerial1.read()); 
  }
  mySerial1.println("AT+SLEEP=10000"); //hello
  
  esp_sleep_enable_timer_wakeup(10000000); // 10 sec

  delay(1000);
  gpio_hold_en((gpio_num_t)10);
  gpio_hold_en((gpio_num_t)1);
  gpio_deep_sleep_hold_en();
  esp_sleep_enable_timer_wakeup(10000000); // 10 sec

  esp_deep_sleep_start();
  delay(15000);
  

}
