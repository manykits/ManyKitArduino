// MK_ArduinoSegment.cpp

#include "MK_Arduino.h"

#if defined MK_SEGMENT7
//----------------------------------------------------------------------------
void MK_Arduino::SegmentInit(int clkPin, int dataPin)
{
    mSegmentDisplay = SegmentDisplay(clkPin, dataPin);
}
//----------------------------------------------------------------------------
void MK_Arduino::SegmentSetBrightness(int brightness)
{
    mSegmentDisplay.setBrightness((uint8_t)brightness);
}
//----------------------------------------------------------------------------
void MK_Arduino::SegmentClear()
{
    mSegmentDisplay.clearDisplay();
}
//----------------------------------------------------------------------------
void MK_Arduino::SegmentDisplayInt(int val)
{
    mSegmentDisplay.display((int16_t)val);
}
//----------------------------------------------------------------------------
void MK_Arduino::SegmentDisplayFloat(float val)
{
    mSegmentDisplay.display(val);
}
//----------------------------------------------------------------------------
#endif
