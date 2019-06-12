// MK_ESP.cpp

#include "MK_Arduino.h"

#if defined MK_ESP_NETWORK
//----------------------------------------------------------------------------
bool MK_Arduino::AutoConfig()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin();
    for (int i = 0; i < 20; i++)
    {     
        pinMode(0, INPUT);
        int val = digitalRead(0);
        if (0 == val)
        {   
          Serial.println("eraseConfig begin."); 
          WiFi.disconnect();
          delay(2000);
          ESP.restart();
          Serial.println("eraseConfig done.");
        }             
        int wstatus = WiFi.status();
        if (wstatus == WL_CONNECTED)
        {
            Serial.println("AutoConfig Success");
            Serial.printf("SSID:%s\r\n", WiFi.SSID().c_str());
            Serial.printf("PSW:%s\r\n", WiFi.psk().c_str());
            WiFi.printDiag(Serial);
            return true;
        }
        else
        {
            Serial.print("AutoConfig Waiting......");
            Serial.println(wstatus);
            delay(500);
        }
    }
    Serial.println("AutoConfig Faild!");
    return false;
}
//----------------------------------------------------------------------------
void MK_Arduino::SmartConfig()
{
    Serial.println("Wait for Smartconfig");
    WiFi.beginSmartConfig();
    bool isSmartDone = WiFi.smartConfigDone(); 
    while (!isSmartDone)
    {
        Serial.print(".");      
       isSmartDone = WiFi.smartConfigDone();
        
        if (isSmartDone)
        {
            Serial.println("SmartConfig Success!");
            Serial.printf("SSID:%s\r\n", WiFi.SSID().c_str());
            Serial.printf("PSW:%s\r\n", WiFi.psk().c_str());
            WiFi.setAutoConnect(true); // 设置自动连接
        }
        delay(1500); // must do delay     
    }
}
//----------------------------------------------------------------------------
#endif
