// MK_ArduinoIR.cpp

#include "MK_Arduino.h"

#if defined MK_IR
//----------------------------------------------------------------------------
void MK_Arduino::IRInit(int pin)
{
    if (mIRrecv)
    {
        delete mIRrecv;
        mIRrecv = 0;
    }

    pinMode(pin, INPUT);
    mIRrecv = new IRrecv(pin);
    mIRrecv->enableIRIn();
}
//----------------------------------------------------------------------------
void MK_Arduino::IRSend(int val)
{ 
    mIRsend.sendSony(val, 32);
    if (mIRrecv)
      mIRrecv->enableIRIn();
}
//----------------------------------------------------------------------------
int MK_Arduino::GetIRReceivedValue()
{
    return mIRRecvValue;
}
//----------------------------------------------------------------------------
void MK_Arduino::_SendIRRecv(int val)
{
    unsigned char cmdCh = sOptTypeVal[OT_RETURN_IR];
    char strCMDCh[32];
    memset(strCMDCh, 0, 32);
    itoa(cmdCh, strCMDCh, 10);

    Serial.print("0000");
    Serial.print(String(strCMDCh));
    Serial.print(" ");
    Serial.println(val);

    // if (7611 == val)
    // {
    //     LeftRun(1,255);
    //     RightRun(1, 255);
    // }
    // else if (-4645 == val)
    // {
    //     LeftRun(2, 255);
    //     RightRun(2, 255);
    // }
    // else if (-11233 == val)
    // {
    //     LeftRun(2, 255);
    //     RightRun(1, 255);
    // }
    // else if (19899 == val)
    // {
    //     LeftRun(1, 255);
    //     RightRun(2, 255);
    // }
    // else if (19227 == val)
    // {
    //     LeftRun(0, 0);
    //     RightRun(0, 0);
    // }
}
#endif
//----------------------------------------------------------------------------
