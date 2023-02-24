#include <Arduino.h>
#include <M5Core2.h>

int counter = 0;

void setup() {
  M5.begin(true, true, true, true, kMBusModeInput);
  M5.Lcd.setCursor(0, 0, 4);
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Lcd.print("Servo Counter");
  M5.Lcd.clear();
}

void loop() {
  M5.Lcd.setCursor(0, 0, 4);
  char buf[100];
  sprintf(buf, "counter: %04d", counter);
  M5.Lcd.println(buf);
  delay(5000);
  counter++;
}