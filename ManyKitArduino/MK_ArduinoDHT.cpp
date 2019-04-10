// MK_ArduinoDHT.cpp

#include "MK_Arduino.h"

#if defined MK_DHT

//----------------------------------------------------------------------------
void MK_Arduino::_DHTInit(MK_Pin pin)
{
  int pinArduino = MK_Pin2Pin(pin);
  mDHT = ManyKit_DHT(pinArduino, MK_DHTTYPE);
  mDHT.begin();
  mIsInitedDHT = true;
}
//----------------------------------------------------------------------------
void MK_Arduino::_DHTSendTemperatureHumidity()
{
  float temp = mDHT.readTemperature();
  float humi = mDHT.readHumidity();
  
  if (!isnan(humi) && !isnan(temp)) 
  {
    unsigned char cmdCh = sOptTypeVal[OT_RETURN_DHTTEMP];
    char strCMDCh[32];
    memset(strCMDCh, 0, 32);
    itoa(cmdCh, strCMDCh, 10);
  
    Serial.print("0000");
    Serial.print(String(strCMDCh)); 
    Serial.print(" ");
    Serial.print(temp);
    Serial.print(" ");
    Serial.println(humi);
  }
}
//----------------------------------------------------------------------------

#endif
