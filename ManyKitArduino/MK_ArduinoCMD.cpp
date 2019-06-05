// MK_ArduinoCMD.cpp
#include "MK_Arduino.h"

//----------------------------------------------------------------------------
void MK_Arduino::OnCMD(String *cmdParams, String &cmdStr)
{
  int i=0;
  for (; i<NumCMDParams; i++)
  {
    cmdParams[i] = "";
  }

  String strs = cmdStr;
  int cmdIndexTemp = 0;
  char *pCMDParam = strtok((char *)strs.c_str(), " ");
  while (pCMDParam)
  {
    cmdParams[cmdIndexTemp] = String(pCMDParam);
    cmdIndexTemp++;
    pCMDParam = strtok(NULL, " ");

    if (cmdIndexTemp > NumCMDParams)
      break;
  }

  if (cmdIndexTemp > 0)
  {
    unsigned char cmdCH = atoi(cmdParams[0].c_str());
    if (sOptTypeVal[OT_TOGET_NETID] == cmdCH)
    {
      _SendNetID();
    }
    else if (sOptTypeVal[OT_SET_TIME] == cmdCH)
    {
      _SetTime();
    }
    else if (sOptTypeVal[OT_PM] == cmdCH)
    {
      int pin = _Str2Pin(cmdParams[1]);
      int val = _Str2IO(cmdParams[2]);
      int i = _Str2MK_Pin(cmdParams[1]);
      pinMode(pin, val);
    }
    else if(sOptTypeVal[OT_DW]==cmdCH)
    {
      int pin = _Str2Pin(cmdParams[1]);
      bool isHigh = _Str2Bool(cmdParams[2]);
      digitalWrite(pin, isHigh?HIGH:LOW);
    }
    else if (sOptTypeVal[OT_AW]==cmdCH)
    {
      int pin = _Str2Pin(cmdParams[1]);
      int val = _Str2Int(cmdParams[2]);
      analogWrite(pin, val);
    }
    else if (sOptTypeVal[OT_RETURN_DR]==cmdCH)
    {
      int pin = _Str2Pin(cmdParams[1]);
      int val = digitalRead(pin);

      int index = pin - P_0;
      
      unsigned char cmdCh = sOptTypeVal[OT_RETURN_DR];
      char strCMDCh[32];
      memset(strCMDCh, 0, 32);
      itoa(cmdCh, strCMDCh, 10);
        
      Serial.print("0000");
      Serial.print(String(strCMDCh)); 
      Serial.print(" ");
      Serial.print(index);
      Serial.print(" ");
      Serial.println(val);
    }
    else if (sOptTypeVal[OT_RETURN_AR]==cmdCH)
    {
      int pin = _Str2Pin(cmdParams[1]);
      int val = analogRead(pin);
      int index = pin - P_0;
      
      unsigned char cmdCh = sOptTypeVal[OT_RETURN_AR];
      char strCMDCh[32];
      memset(strCMDCh, 0, 32);
      itoa(cmdCh, strCMDCh, 10);
        
      Serial.print("0000");
      Serial.print(String(strCMDCh)); 
      Serial.print(" ");
      Serial.print(index);
      Serial.print(" ");
      Serial.println(val);
    }
    else if (sOptTypeVal[OT_SVR_I]==cmdCH)
    {
#if defined MK_SERVO
      int index = _Str2Int(cmdParams[1]);
      int pin = _Str2Pin(cmdParams[2]);
      if (0 <= index && index < NumMaxServer)
        mServo[index].attach(pin);
#endif
    }
    else if (sOptTypeVal[OT_SVR_W]==cmdCH)
    {
#if defined MK_SERVO
      int index = _Str2Int(cmdParams[1]);
      int val = _Str2Int(cmdParams[2]);
      if (0 <= index && index < NumMaxServer)
        mServo[index].write(val);
#endif
    }
    else if (sOptTypeVal[OT_DST_I]==cmdCH)
    {
      int pinTrigger = _Str2Pin(cmdParams[1]);
      int pinEcho = _Str2Pin(cmdParams[2]);
      _DistInit_(pinTrigger, pinEcho);
    }
    else if (sOptTypeVal[OT_DST_T]==cmdCH)
    {
      long mDurationLTemp;
      if(millis() - mDistCheckLastTime > 25)
      {
         mDistCheckLastTime = millis();
         _DistTest(); 
      }

      unsigned char cmdCh = sOptTypeVal[OT_RETURN_DIST];
      char strCMDCh[32];
      memset(strCMDCh, 0, 32);
      itoa(cmdCh, strCMDCh, 10);
      
      Serial.print("0000");
      Serial.print(String(strCMDCh)); 
      Serial.print(" ");
      Serial.println(mDist);
    }
    else if (sOptTypeVal[OT_MOTO_I]==cmdCH)
    {
#if defined MK_MOTO
      MotoInit10111213();
#endif
    }
    else if (sOptTypeVal[OT_MOTO_I_DRIVER4567] == cmdCH)
    {
#if defined MK_MOTO
      MotoInit4567();
#endif
    }
    else if (sOptTypeVal[OT_MOTO_I_DRIVER298N] == cmdCH)
    {
#if defined MK_MOTO
      int pinL = _Str2Pin(cmdParams[1]);
      int pinL1 = _Str2Pin(cmdParams[2]);
      int pinLS = _Str2Pin(cmdParams[3]);
      int pinR = _Str2Pin(cmdParams[4]);
      int pinR1 = _Str2Pin(cmdParams[5]);
      int pinRS = _Str2Pin(cmdParams[6]);      
      MotoInit298N(pinL, pinL1, pinLS, pinR, pinR1, pinRS);
#endif
    }   
    else if (sOptTypeVal[OT_MOTO_RUN]==cmdCH)
    {
#if defined MK_MOTO
      int motoIndex = _Str2Int(cmdParams[1]);
      int dir = _Str2DirType(cmdParams[2]);
      int spd = _Str2Int(cmdParams[3]);

      if (0 == dir)
        spd = 0;

      if (0 == motoIndex)
        LeftRun(dir, spd);
      else if (1 == motoIndex)
        RightRun(dir, spd);
#endif
    }
    else if (sOptTypeVal[OT_MOTO_RUNSIMPLE]==cmdCH)
    {
#if defined MK_MOTO
      int dir = _Str2SimpleDirType(cmdParams[1]);
      int spd = _Str2Int(cmdParams[2]);
      if (0 == dir)
      {
        spd = 0;
        LeftRun(0, spd);
        RightRun(0, spd);
      }
      else if (1 == dir)
      {
        LeftRun(1, spd);
        RightRun(1, spd);
      }
      else if (2 == dir)
      {
        LeftRun(2, spd);
        RightRun(2, spd);
      }
      else if (3 == dir)
      {
        LeftRun(2, spd);
        RightRun(1, spd);
      }
      else if (4 == dir)
      {
        LeftRun(1, spd);
        RightRun(2, spd);
      }
#endif
    }
    else if (sOptTypeVal[OT_MOTO_STOP]==cmdCH)
    {
#if defined MK_MOTO
      LeftRun(0, 0);
      RightRun(0, 0);
#endif
    }
    else if (sOptTypeVal[OT_MP3_INIT]==cmdCH)
    {
      int pinR = _Str2Pin(cmdParams[1]);
      int pinT = _Str2Pin(cmdParams[2]);
      MP3Init(pinR, pinT);
    }
    else if (sOptTypeVal[OT_MP3_DO]==cmdCH)
    {
   #if defined MK_MP3
      Mp3PlayType type = (Mp3PlayType)_Str2Int(cmdParams[1]);
      MP3Do(type);
   #endif
    }
    else if (sOptTypeVal[OT_MP3_PLAYFOLDER]==cmdCH)
    {
      int param0 = _Str2Int(cmdParams[1]);
      int param1 = _Str2Int(cmdParams[2]);
      MP3FolderPlay(param0, param1);
    }
    else if (sOptTypeVal[OT_MP3_SETVOLUME]==cmdCH)
    {
      int val = _Str2Int(cmdParams[1]);
      MP3SetVolime(val);
    }
    else if (sOptTypeVal[OT_IR_INIT]==cmdCH)
    {
#if defined MK_IR
      int pinR = _Str2Pin(cmdParams[1]);
      IRInit(pinR);      
#endif
    }
    else if (sOptTypeVal[OT_IR_SEND]==cmdCH)
    {
#if defined MK_IR
      int val = _Str2Int(cmdParams[1]);
      IRSend(val);
#endif
    }
    else if (sOptTypeVal[OT_MOTO_I_SPD]==cmdCH)
    {
#if defined MK_MOTO
      int pinLA = _Str2Pin(cmdParams[1]);
      int pinLB = _Str2Pin(cmdParams[2]);
      int pinRA = _Str2Pin(cmdParams[3]);
      int pinRB = _Str2Pin(cmdParams[4]);
      MotoSpeedInit(pinLA, pinLB, pinRA, pinRB);
#endif
    }
    else if (sOptTypeVal[OT_RETURN_MOTOSPD] == cmdCH)
    {
      /*_*/
    }
    else if (sOptTypeVal[OT_HX711_I]==cmdCH)
    {
#if defined MK_XH711
      int index = _Str2Int(cmdParams[1]);
      int pinOut = _Str2Pin(cmdParams[2]);
      int pinClk = _Str2Pin(cmdParams[3]);
      HX711Init(index, pinOut, pinClk);
#endif
    }
    else if (sOptTypeVal[OT_HX711_TEST]==cmdCH)
    {
#if defined MK_XH711
      int index = _Str2Int(cmdParams[1]);
      float val = _ReadHX711(index);
      _HXSend(index, val);
#endif
    }
    else if (sOptTypeVal[OT_RC_INIT]==cmdCH)
    {      
#if defined MK_RCSWITCH
     int pin = _Str2Pin(cmdParams[1]);
    _RCInit(pin);
#endif
    }
   else if (sOptTypeVal[OT_RC_SEND]==cmdCH)
    {
  #if defined MK_RCSWITCH
      long val = _Str2Long(cmdParams[1]);
      _RCSend(val);
  #endif
    }
    else if (sOptTypeVal[OT_DHT_I]==cmdCH)
    {
#if defined MK_DHT
      int pin = _Str2Pin(cmdParams[1]);
      DHTInit(pin);
#endif
    }
    else if (sOptTypeVal[OT_LEDSTRIP_I]==cmdCH)
    {
#if defined MK_RGBLED  
      int pin = _Str2Pin(cmdParams[1]);
      int num = _Str2Int(cmdParams[2]);
      RGBLEDInit(pin, num);
#endif
    }
    else if (sOptTypeVal[OT_LEDSTRIP_SET]==cmdCH)
    {
#if defined MK_RGBLED  
      int index = _Str2Int(cmdParams[1]);
      int r = _Str2Int(cmdParams[2]);
      int g = _Str2Int(cmdParams[3]);
      int b = _Str2Int(cmdParams[4]);
      RGBLEDSetColor(index, r, g, b);
#endif    
    }
    else if (sOptTypeVal[OT_SEGMENT_I]==cmdCH)
    {
#if defined MK_SEGMENT
      int pinClk = _Str2Pin(cmdParams[1]);
      int pinData = _Str2Pin(cmdParams[2]);
      SegmentInit(pinClk, pinData);
#endif
    }
    else if (sOptTypeVal[OT_SEGMENT_BRIGHTNESS]==cmdCH)
    {
#if defined MK_SEGMENT
      int val = _Str2Int(cmdParams[1]);
      SegmentSetBrightness(val);
#endif
    }
    else if (sOptTypeVal[OT_SEGMENT_CLEAR]==cmdCH)
    {
#if defined MK_SEGMENT
      SegmentClear();
#endif
    }
    else if (sOptTypeVal[OT_SEGMENT_DISPLAY]==cmdCH)
    {
#if defined MK_SEGMENT
      int type = _Str2Int(cmdParams[1]);
      float val = _Str2Float(cmdParams[2]);
      if (1 == type)
      {
       SegmentDisplayInt((int)val);
      }
      else
      {
        SegmentDisplayFloat(val);
      }
#endif
    }
    else if (sOptTypeVal[OT_LEDMATRIX_I]==cmdCH)
    {
#if defined MK_LEDMATRIX
      int pinClk = _Str2Pin(cmdParams[1]);
      int pinData = _Str2Pin(cmdParams[2]);
      LEDMatrixInit(pinClk, pinData);
#endif
    }
    else if (sOptTypeVal[OT_LEDMATRIX_BRIGHTNESS]==cmdCH)
    {
#if defined MK_LEDMATRIX
      int val = _Str2Int(cmdParams[1]);
      LEDMatrixSetBrightness(val);
#endif
    }
    else if (sOptTypeVal[OT_LEDMATRIX_CLEARSCREEN]==cmdCH)
    {
#if defined MK_LEDMATRIX
      LEDMatrixClearScreen();
#endif
    }
    else if (sOptTypeVal[OT_LEDMATRIX_LIGHTAT]==cmdCH)
    {
#if defined MK_LEDMATRIX
      int x = _Str2Int(cmdParams[1]);
      int y = _Str2Int(cmdParams[2]);
      int width = _Str2Int(cmdParams[3]);
      bool onOff = _Str2Bool(cmdParams[4]);

      LEDMatrixLightPos(x,y,width,onOff);
#endif
    }
    else if (sOptTypeVal[OT_STEPMOTO_I]==cmdCH)
    {
#if defined MK_STEPMOTO
      int index = _Str2Int(cmdParams[1]);
      int pinVCC = _Str2Pin(cmdParams[2]);
      int pinPLS = _Str2Pin(cmdParams[3]);
      int pinDIR = _Str2Pin(cmdParams[4]);
      int pinEnable = _Str2Pin(cmdParams[5]);

      StepMotoInit(index, pinVCC, pinPLS, pinDIR, pinEnable);

#endif
    }
    else if (sOptTypeVal[OT_STEPMOTO_ENABLE]==cmdCH)
    {
#if defined MK_STEPMOTO
      int index = _Str2Int(cmdParams[1]);
      bool enable = _Str2Bool(cmdParams[2]);

      StepMotoEnable(index, enable);
#endif
    }
    else if (sOptTypeVal[OT_STEPMOTO_DIR]==cmdCH)
    {
#if defined MK_STEPMOTO
      int index = _Str2Int(cmdParams[1]);
      bool bDir = _Str2Bool(cmdParams[2]);

      StepMotoDir(index, bDir);
#endif
    }
    else if (sOptTypeVal[OT_STEPMOTO_STEP]==cmdCH)
    {
#if defined MK_STEPMOTO
      int index = _Str2Int(cmdParams[1]);
      int delayVal = _Str2Int(cmdParams[2]);

      StepMotoStep(index, delayVal);
#endif
    }
    else if (sOptTypeVal[OT_LCI2C_INIT] == cmdCH)
    {
#if defined MK_SCREEN_I2C
      int addr = _Str2Int(cmdParams[1]);
      int numCols = _Str2Int(cmdParams[2]);
      int numRows = _Str2Int(cmdParams[3]);
      
      LCI2C_Init(addr, numCols, numRows);
#endif
    }
    else if (sOptTypeVal[OT_LCI2C_DO] == cmdCH)
    {
#if defined MK_SCREEN_I2C
      SCREEN_I2C_DoType doType = (SCREEN_I2C_DoType)_Str2Int(cmdParams[1]);
      LCI2C_DoType(doType);
#endif
    }
    else if (sOptTypeVal[OT_LCI2C_SETCURSOR] == cmdCH)
    {
#if defined MK_SCREEN_I2C
      int col = _Str2Int(cmdParams[1]);
      int row = _Str2Int(cmdParams[2]);
      LCI2C_SetCursor(col, row);
#endif
    }
    else if (sOptTypeVal[OT_LCI2C_SETBACKLIGHT] == cmdCH)
    {
#if defined MK_SCREEN_I2C
      int lt = _Str2Int(cmdParams[1]);
      LCI2C_SetBackLight(lt);
#endif
    }
    else if (sOptTypeVal[OT_LCI2C_PRINT] == cmdCH)
    {
#if defined MK_SCREEN_I2C
      String strVal = cmdStr.substring(3);
      LCI2C_Print(strVal);
#endif
    }
    else if (sOptTypeVal[OT_LCI2C_PRINTBYTE] == cmdCH)
    {
#if defined MK_SCREEN_I2C
      int val = _Str2Int(cmdParams[1]);
      LCI2C_PrintByte(val);
#endif
    }
  }
}
//----------------------------------------------------------------------------
int MK_Arduino::_Str2IO(String &str)
{
  char ch = atoi(str.c_str());
  if (0 == ch)
    return INPUT;
  
  return OUTPUT;
}
//----------------------------------------------------------------------------
int MK_Arduino::_Str2Pin(String &str)
{
  int pinVal = atoi(str.c_str());
  if (0 <= pinVal && pinVal <= P_13)
    return pinVal;
  else if (MK_A0 <= pinVal)
    return (pinVal - MK_A0) + A0;

  return 0;
}
//----------------------------------------------------------------------------
int MK_Arduino::MK_Pin2Pin(MK_Pin pxfPin)
{
  if (P_0 <= pxfPin && pxfPin <= P_13)
    return pxfPin;
  else if (P_A0 <= pxfPin && pxfPin <= P_A6)
    return (pxfPin - P_A0) + A0;

  return 0;
}
//----------------------------------------------------------------------------
MK_Pin MK_Arduino::_Str2MK_Pin(String &str)
{
  int pinVal = atoi(str.c_str());
  if (0 <= pinVal && pinVal <= P_13)
    return (MK_Pin)(P_0 + pinVal);
  else if (MK_A0 <= pinVal)
    return (MK_Pin)((pinVal - MK_A0) + P_A0);
    
  return P_0;
}
//----------------------------------------------------------------------------
bool MK_Arduino::_Str2Bool(String &str)
{
  if (!String("0").compareTo(str))
    return false;

  return true;
}
//----------------------------------------------------------------------------
int MK_Arduino::_Str2Int(String &str)
{
  int iVal = atoi(str.c_str());
  return iVal; 
}
//----------------------------------------------------------------------------
float MK_Arduino::_Str2Float(String &str)
{
  float fVal = (float)atof(str.c_str());
  return fVal;
}
//----------------------------------------------------------------------------
long MK_Arduino::_Str2Long(String &str)
{
  long lVal = (float)atol(str.c_str());
  return lVal;
}
//----------------------------------------------------------------------------
int MK_Arduino::_Str2DirType(String &str)
{
  return atoi(str.c_str());
}
//----------------------------------------------------------------------------
int MK_Arduino::_Str2SimpleDirType(String &str)
{
  return atoi(str.c_str());
}
//----------------------------------------------------------------------------
