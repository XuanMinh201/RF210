/*
       __          ____        _____                       __    _ __
      / /   ____  / __ \____ _/ ___/____  ____ _________  / /   (_) /_
     / /   / __ \/ /_/ / __ `/\__ \/ __ \/ __ `/ ___/ _ \/ /   / / __ \
    / /___/ /_/ / _, _/ /_/ /___/ / /_/ / /_/ / /__/  __/ /___/ / /_/ /
   /_____/\____/_/ |_|\__,_//____/ .___/\__,_/\___/\___/_____/_/_.___/
                                /_/
   Author: m1nhle, mtnguyen
   Code : RF210_RAK3172_bridge : Provide on ESP32 Serial port a direct access in write and read to Rak3172 module for AT Command mode


*/

// ESP32 C3 SERIAL1 (second UART)
HardwareSerial mySerial1(1);

int rxPin = 20;
int txPin = 21;
uint8_t button = 0;

void setup()
{
  Serial.begin(115200);
  delay(1000);
 Serial.println("Bridge Test");
  
  pinMode(txPin, OUTPUT);
  pinMode(rxPin, INPUT);
  pinMode(10, OUTPUT); //Rak enable
  pinMode(4, OUTPUT); // LED
   pinMode(9, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(9), i_button_isr, RISING);
 // pinMode(1, OUTPUT); // GNSS enable
  digitalWrite(4, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(4, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);  

  //  digitalWrite(1, LOW);   // switch off GPS
    digitalWrite(10, HIGH); // Switch on RAK
    delay(1000);
  mySerial1.begin(115200, SERIAL_8N1, rxPin, txPin);
  delay(1000);  
  mySerial1.println("ATE");
  delay(100);
  
    while (mySerial1.available()){
      Serial.write(mySerial1.read()); // read it and send it out Serial (USB)
    }
 
}

void loop()
{

 
  if (button > 0){
  
 mySerial1.println("ATC+GPSON=1");
  delay(300);
   mySerial1.println("ATC+GPSCONST");
  delay(300);
  mySerial1.println("ATC+GPSNMEA=1");
  delay(300);
  digitalWrite(4, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(200);                       // wait for a second
  digitalWrite(4, LOW);    // turn the LED off by making the voltage LOW
  delay(200);  
  button=0;    
  }
  
  if (Serial.available()) {      // If anything comes in Serial (USB),

    mySerial1.write(Serial.read());   // read it and send it out mySerial1 (pins 0 & 1)      

  }

  if (mySerial1.available()) {     // If anything comes in Serial1 (pins 0 & 1)

    Serial.write(mySerial1.read());   // read it and send it out Serial (USB)
  }
}

void i_button_isr(void) {
button++; 
}
