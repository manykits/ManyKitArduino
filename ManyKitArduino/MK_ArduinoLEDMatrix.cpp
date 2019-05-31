// MK_ArduinoLEDMatrix.cpp

#include "MK_Arduino.h"

#if defined MK_LEDMATRIX

void MK_Arduino::LEDMatrixInit(int sckPin, int dinPin)
{
    mLEDMatrix = LEDMatrix(sckPin, dinPin);
    mLEDMatrix.setColorIndex(1);
}
//----------------------------------------------------------------------------
void MK_Arduino::LEDMatrixSetBrightness(int brightness)
{
  mLEDMatrix.setBrightness(brightness);
}
//----------------------------------------------------------------------------
void MK_Arduino::LEDMatrixClearScreen()
{
    mLEDMatrix.clearScreen();
}
//----------------------------------------------------------------------------
void MK_Arduino::LEDMatrixSetColorIndex(int iColor_Number)
{
    mLEDMatrix.setColorIndex(iColor_Number);
}
//----------------------------------------------------------------------------
void MK_Arduino::LEDMatrixDrawBitmap(int8_t x, int8_t y, uint8_t bitmap_Width, 
    uint8_t *bitmap)
{
   mLEDMatrix.drawBitmap(x, y, bitmap_Width, bitmap);
}
//----------------------------------------------------------------------------
void MK_Arduino::LEDMatrixLightPos(int8_t x, int8_t y, int width, bool onOff)
{
    if (width > 16)
        width = 16;
        
    if (onOff)
    {
        unsigned char buf[16]={128,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};
        for (int i=0; i<width; i++)
        {
            buf[i] = 128;
        }

        mLEDMatrix.drawBitmap(x,y,16,buf);
    }
    else
    {
        unsigned char buf[16]={0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};
        for (int i=0; i<width; i++)
        {
            buf[i] = 0;
        }
        mLEDMatrix.drawBitmap(x,y,16,buf);
    }
}
//----------------------------------------------------------------------------
void MK_Arduino::LEDMatrixDrawStr(int16_t x_position, int8_t y_position,
     const char *str)
{
    mLEDMatrix.drawStr(x_position, y_position, str);
}
//----------------------------------------------------------------------------
void MK_Arduino::LEDMatrixShowClock(uint8_t hour, uint8_t minute,
 bool isPointOn)
{
    mLEDMatrix.showClock(hour, minute, isPointOn);
}
//----------------------------------------------------------------------------
void MK_Arduino::LEDMatrixShowNum(float value,uint8_t val)
{
    mLEDMatrix.showNum(value, val);
}
//----------------------------------------------------------------------------

#endif
