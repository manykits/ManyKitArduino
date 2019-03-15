// MK_ArduinoDisplayss1306.cpp
#include "MK_Arduino.h"

#if defined MK_SSD1306

#define OLED_RESET 4
//----------------------------------------------------------------------------
void MK_Arduino::_ScreenInit()
{
  mDisplay = new Adafruit_SSD1306(OLED_RESET);
  mDisplay->begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS);
  mDisplay->display();
}
//----------------------------------------------------------------------------

#endif

