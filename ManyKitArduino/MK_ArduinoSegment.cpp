// MK_ArduinoSegment.cpp

#include "MK_Arduino.h"

#if defined MK_SEGMENT7
//----------------------------------------------------------------------------
void MK_Arduino::_SegmentInit(int clkPin, int dataPin)
{
    mSegmentDisplay = SegmentDisplay(clkPin, dataPin);
}
//----------------------------------------------------------------------------
void MK_Arduino::_SegmentSetBrightness(int brightness)
{
    mSegmentDisplay.setBrightness((uint8_t)brightness);
}
//----------------------------------------------------------------------------
void MK_Arduino::_SegmentClear()
{
    mSegmentDisplay.clearDisplay();
}
//----------------------------------------------------------------------------
void MK_Arduino::_SegmentDisplayInt(int val)
{
    mSegmentDisplay.display((int16_t)val);
}
//----------------------------------------------------------------------------
void MK_Arduino::_SegmentDisplayFloat(float val)
{
    mSegmentDisplay.display(val);
}
//----------------------------------------------------------------------------
#endif
