void setup() {
  Serial.begin(115200);
  
  pinMode (10, OUTPUT);
  digitalWrite(10, LOW);
  
}

void loop() {

  // wake up 1 second later and then go into deep sleep
  esp_sleep_enable_timer_wakeup(10000000); // 10 sec
  esp_deep_sleep_start(); 
  // we never reach here
}
