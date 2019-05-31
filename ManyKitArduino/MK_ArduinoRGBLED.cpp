// MK_ArduinoRGBLED.cpp

#include "MK_Arduino.h"

#if defined MK_RGBLED

//----------------------------------------------------------------------------
void MK_Arduino::RGBLEDInit(int pin, int num)
{
    mWS2812 = WS2812(num);
    mWS2812.setOutput(pin);
}
//----------------------------------------------------------------------------
void MK_Arduino::RGBLEDSetColor(int index, int r, int g, int b)
{
    cRGB c;
    c.r = r;
    c.g = g;
    c.b = b;
    mWS2812.set_crgb_at(index, c);
    mWS2812.sync();
}
//----------------------------------------------------------------------------

#endif
