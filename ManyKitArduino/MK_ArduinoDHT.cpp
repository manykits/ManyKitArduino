// MK_ArduinoDHT.cpp

#include "MK_Arduino.h"

#if defined MK_DHT

//----------------------------------------------------------------------------
void MK_Arduino::DHTInit(int pin)
{
  pinMode(pin, OUTPUT);
  mDHT = ManyKit_DHT(pin, MK_DHTTYPE);
  mDHT.begin();
  mIsInitedDHT = true;
  mTemperature = 0.0f;
  mHumidity = 0.0f;
}
//----------------------------------------------------------------------------
int MK_Arduino::GetTemperature()
{
  return mTemperature;
}
//----------------------------------------------------------------------------
int MK_Arduino::GetHumidity()
{
  return mHumidity;
}
//----------------------------------------------------------------------------
void MK_Arduino::_DHTSendTemperatureHumidity()
{
  if (mIsInitedDHT)
  {
    float temp = mDHT.readTemperature();
    float humi = mDHT.readHumidity();

    if (!isnan(humi) && !isnan(temp))
    {
      mTemperature = temp;
      mHumidity = humi;

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

#if defined MK_ESP_NETWORK
      if (0 != mRemotePort)
      {
        String dst = String("0000") + String(strCMDCh) +
                     String(" ") + F2Str(temp) +
                     String(" ") + F2Str(humi) + String("\n");

        mWiFiUDP.beginPacket(mRemoteIP, mRemotePort);
        mWiFiUDP.write((const char *)&dst[0], dst.length());
        mWiFiUDP.endPacket();
      }
#endif
    }
  }
}
//----------------------------------------------------------------------------

#endif
