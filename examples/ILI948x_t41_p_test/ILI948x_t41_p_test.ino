#include "ILI948x_t41_p.h"
#include "flexio_teensy_mm.c"
ILI948x_t41_p lcd = ILI948x_t41_p(10,8,9); //(dc, cs, rst)

const uint16_t screenWidth = 480;
const uint16_t screenHeight = 320;

uint16_t dispBuffer[screenWidth * screenHeight];


void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.print(CrashReport);
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  //memset(dispBuffer, 0x07e0,sizeof(dispBuffer));
  
  lcd.begin(24);
  lcd.setRotation(3);
}

void loop() {

  lcd.pushPixels16bit(flexio_teensy_mm,0,0,479,319); // 480x320
  delay(1000);
  lcd.pushPixels16bitAsync(flexio_teensy_mm,0,0,479,319); // 480x320
  delay(1000);
}
