// ManyKitArduino.ino

#include "MK_Arduino.h"
 
void setup() 
{ 
  mk.Init(true);
}

void loop()
{
  mk.Tick(); 
}
