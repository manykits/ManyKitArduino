// MK_ArduinoRCSwitch.cpp

#include "MK_Arduino.h"

#if defined MK_RCSWITCH

//----------------------------------------------------------------------------
void MK_Arduino::_InitRCSwitchReceive(int pinTimerIndex)
{
    mRCSwitch.enableReceive(pinTimerIndex); 
}
//----------------------------------------------------------------------------
void MK_Arduino::_RCInit(int pin)
{
  mRCSwitch.enableTransmit(pin);
}
//----------------------------------------------------------------------------
void MK_Arduino::_RCSend(long val)
{
    mRCSwitch.send(val, 24);
    delay(100);
    mRCSwitch.send(val, 24);
    delay(100);
    mRCSwitch.send(val, 24);
}
//----------------------------------------------------------------------------

#endif
