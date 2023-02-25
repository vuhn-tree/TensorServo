// #include <Arduino.h>
#include <M5Core2.h>
#include <Wire.h>
#include "Adafruit_PWMServoDriver.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

#define SERVOMIN 102  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 512  // This is the 'maximum' pulse length count (out of 4096)
#define USMIN \
  500  // This is the rounded 'minimum' microsecond length based on the minimum
       // pulse of 102
#define USMAX \
  2500  // This is the rounded 'maximum' microsecond length based on the maximum
        // pulse of 512
#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates

const int INFO_HEIGHT_POS = 25;

/**
 * Port A: ??
 * Port B: 36
 * Port C: ??
 */
int sensorPin = 36;          // set the input pin for the potentiometer.
int last_sensorValue = 100;  // Stores the value last read by the sensor.
int cur_sensorValue = 0;     // Stores the value currently read by the sensor.

void setup() {
  M5.begin(true, true, true, true, kMBusModeInput);
  /* kMBusModeOutput, powered by USB or Battery
   kMBusModeInput, powered by outside input need to fill in this Otherwise
   M5Core2 will not work properly
   */
  Wire1.begin(21, 22);
  pwm.begin();
  pwm.setPWMFreq(50);
  Serial.begin(9600);

  pinMode(sensorPin, INPUT);  // Sets the specified pin to input mode.

  // M5.Lcd.setBrightness(0);
  M5.Axp.SetLcdVoltage(2600);
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Lcd.drawString("Servo Monitor", 0, 0, 4);

  // dacWrite(25, 0);
}

void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  pulselength = 1000000;  // 1,000,000 us per second
  pulselength /= 50;      // 50 Hz
  Serial.print(pulselength);
  Serial.println(" us per period");
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength);
  Serial.println(" us per bit");
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void servo_angle_write(uint8_t n, int Angle) {
  double pulse = Angle;
  pulse = pulse / 90 + 0.5;
  setServoPulse(n, pulse);
}

void loop() {
   M5.update();

  // mapped to pot
  cur_sensorValue = analogRead(sensorPin);
  const int normalVal = map(cur_sensorValue, 0, 4096, 0, 180);

  char buf[50];
  sprintf(buf, "Raw Pot:  %04d%", cur_sensorValue);
  M5.Lcd.drawString(buf, 0, INFO_HEIGHT_POS, 4);

  sprintf(buf, "Norm Pot: %03d%", normalVal);
  M5.Lcd.drawString(buf, 0, INFO_HEIGHT_POS + 25, 4);

  const float powerTemp = M5.Axp.GetTempInAXP192();
  sprintf(buf, "Powr Temp: %2.1fC", powerTemp);
  M5.Lcd.drawString(buf, 0, INFO_HEIGHT_POS + 50, 4);

  // M5.Lcd.setCursor(0, INFO_HEIGHT_POS + 75, 4);
  if(M5.BtnA.read()) {
    M5.shutdown();  
  } 

  if(M5.BtnC.read()) {
    // M5.Lcd.println("Sleeping...");
    // M5.setWakeupButton();  
    // M5.Lcd.sleep();
    M5.Axp.SetLcdVoltage(3300);
    // M5.Lcd.setBrightness(10);
  }
  
  // first sensor value
  servo_angle_write(0, normalVal);
  servo_angle_write(15, normalVal);

  delay(1000);
  // M5.Lcd.clearDisplay();
}