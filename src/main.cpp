#include <Arduino.h>
#include <M5Core2.h>

int counter = 0;

void setup() {
  M5.begin(true, true, true, true, kMBusModeInput);
  // put your setup code here, to run once:

  M5.Lcd.setCursor(0, 0, 4);
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Lcd.print("Servo TEST");
  M5.Lcd.clear();
}

void loop() {
  // put your main code here, to run repeatedly:
  
  M5.Lcd.setCursor(0, 0, 4);
  M5.Lcd.println(counter);
  delay(5000);
  counter++;
}