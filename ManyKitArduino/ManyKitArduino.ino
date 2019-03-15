// ManyKitArduino.ino

#include "MK_Arduino.h"

MK_Arduino manykit;

void setup() 
{ 
  // Serial
  Serial.begin(9600);

  manykit.Init(true);             

#if defined MA_AXIS
  manykit._InitAxis();
#endif

#if defined MA_SSD1306
  manykit._ScreenInit();
#endif
}

String recvStr;
void loop()
{   
  while (Serial.available())
  {
    char c = Serial.read();
    
    if ('\n' == c)
    {
      if (recvStr.length() > 0)
      {
        // to compatile with PHOENIXEngine
        // 2 for length,2 for id
        String cmdStr = recvStr.substring(4);
        manykit.OnCMD(cmdStr);
      }
      recvStr = "";
    }
    else
    {
      recvStr += c;
    }
  }

  manykit.Tick(); 
}
