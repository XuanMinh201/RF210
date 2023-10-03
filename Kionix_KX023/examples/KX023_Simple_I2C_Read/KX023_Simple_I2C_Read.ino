#include <Kionix_KX023.h>

KX023 myIMU;
float kx_x, kx_y, kx_z;

void gpio_init(void)
{
  pinMode(LS_LED_BLUE, OUTPUT);
  digitalWrite(LS_LED_BLUE, LOW);
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
  myIMU.configContinuousReading(LOWPOWER, RANGE_8G, DATARATE_100HZ);

  delay(2000);
}

void loop(void)
{
  myIMU.readAcceleration(&kx_x, &kx_y, &kx_z);
  Serial.print("KX023-1025: ");
  Serial.print(kx_x);
  Serial.print(", ");
  Serial.print(kx_y);
  Serial.print(", ");
  Serial.print(kx_z);
  Serial.println("");

  led_blink();

  // Stop reading
  // if (millis() > 30000ul) {
  //   myIMU.end();
  //   while (1);
  // }
}
