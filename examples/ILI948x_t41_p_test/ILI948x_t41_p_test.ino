#include "ILI948x_t41_p.h"
#include "teensy41.c"
ILI948x_t41_p lcd = ILI948x_t41_p(10,8,9); //(dc, cs, rst)


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.print(CrashReport);
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  
  lcd.begin(24);
  lcd.setRotation(3);
}

void loop() {
  lcd.pushPixels16bit(teensy41,0,0,479,319); // 480x320
  delay(1000);
  lcd.pushPixels16bitAsync(teensy41,0,0,479,319); // 480x320
  delay(1000);
}
