// ManyKitArduino.ino

#include "MK_Arduino.h"
 
void setup() 
{ 
  // Serial
  Serial.begin(9600);

  mk.Init(true);             

#if defined MA_AXIS
  mk._InitAxis();
#endif

#if defined MA_SSD1306
  mk._ScreenInit();
#endif
}

void loop()
{   
  while (Serial.available())
  {
    char c = Serial.read();
    
    if ('\n' == c)
    {
      if (mk.RecvStr.length() > 0)
      {
        // to compatile with PHOENIXEngine
        // 2 for length,2 for id
        String cmdStr = mk.RecvStr.substring(4);
        mk.OnCMD(cmdStr);
      }
      mk.RecvStr = "";
    }
    else
    {
      mk.RecvStr += c;
    }
  }

  mk.Tick(); 
}
