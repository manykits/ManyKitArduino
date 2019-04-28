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

void loop()
{   
  while (Serial.available())
  {
    char c = Serial.read();
    
    if ('\n' == c)
    {
      if (manykit.RecvStr.length() > 0)
      {
        // to compatile with PHOENIXEngine
        // 2 for length,2 for id
        String cmdStr = manykit.RecvStr.substring(4);
        manykit.OnCMD(cmdStr);
      }
      manykit.RecvStr = "";
    }
    else
    {
      manykit.RecvStr += c;
    }
  }

  manykit.Tick(); 
}
