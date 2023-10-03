#include <Kionix_KX023.h>

#define KX023_INT1_PIN 6 // PB2

KX023 myIMU;

void gpio_init(void)
{
  pinMode(LS_LED_BLUE, OUTPUT);
  digitalWrite(LS_LED_BLUE, LOW);

  pinMode(KX023_INT1_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(KX023_INT1_PIN), kx023_int1_handler, RISING);
}

void inline led_blink(uint8_t num_of_blink = 1)
{
  for (; num_of_blink > 0; num_of_blink--)
  {
    digitalWrite(LS_LED_BLUE, HIGH);
    delay(50);
    digitalWrite(LS_LED_BLUE, LOW);
    delay(150);
  }
}

void setup(void)
{
  Serial.begin(9600);

  gpio_init();
  Wire.begin();

  while (!Serial && millis() < 5000)
    ;
  Serial.println("****** KX023-1025 Read Example for Gemini ******");

  if (myIMU.begin())
  {
    Serial.println("Could not find KX023-1025? Check wiring");
  }
  else
  {
    Serial.println("KX023-1025: OK");
  }
  
  myIMU.configContinuousReading();
  myIMU.configSingleTapDetection();
  delay(2000);
}

void loop(void)
{
  delay(1000);
  // led_blink(2);

  Serial.println(myIMU.getInterruptStatus(), HEX);
}

void kx023_int1_handler(void)
{
  myIMU.clearInterrupt();
  Serial.println("Tap detected!");
}
