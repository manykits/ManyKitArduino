// MK_ArduinoRGBLED.cpp

#include "MK_Arduino.h"

#if defined MK_LEDSTRIP

//----------------------------------------------------------------------------
void MK_Arduino::_RGBLEDInit(MK_Pin pin, int num)
{
    int pinArduino = MK_Pin2Pin(pin);

    mWS2812 = WS2812(num);
    mWS2812.setOutput(pinArduino);
}
//----------------------------------------------------------------------------
void MK_Arduino::_RGBLEDSetColor(int index, int r, int g, int b)
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
