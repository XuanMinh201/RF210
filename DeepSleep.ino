void setup() {
  
  // disnable LDO Rak
  pinMode (10, OUTPUT);
  digitalWrite(10, LOW);
  //Disnable GPS 
  pinMode (1, OUTPUT);
  digitalWrite(1, LOW);
  pinMode (0, OUTPUT);
  digitalWrite(0, LOW);  
}

void loop() {

    digitalWrite(10, HIGH);  
    delay(5000);
    esp_sleep_enable_timer_wakeup(10000000); // 10 sec

    
    digitalWrite(10, LOW);
    delay(1000);
    gpio_hold_en((gpio_num_t)10);
    gpio_deep_sleep_hold_en();
  // wake up 1 second later and then go into deep sleep
  
  esp_deep_sleep_start(); 
  // we never reach here
}
