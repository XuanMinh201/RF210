void setup() {

  // disable LDO Rak
  pinMode (10, OUTPUT);
  digitalWrite(10, LOW);
  //Disnable GPS
  pinMode (1, OUTPUT);
  digitalWrite(1, LOW);
  
}

void loop() {
  pinMode (10, OUTPUT);
  digitalWrite(10, LOW);
  //Disnable GPS
  pinMode (1, OUTPUT);
  digitalWrite(1, LOW);

  esp_sleep_enable_timer_wakeup(10000000); // 10 sec
  
  delay(1000);
  gpio_hold_en((gpio_num_t)10);
  gpio_hold_en((gpio_num_t)1);
  gpio_deep_sleep_hold_en();
  // wake up 1 second later and then go into deep sleep
  esp_sleep_enable_timer_wakeup(100000000); // 10 sec

  esp_deep_sleep_start();
  
}
