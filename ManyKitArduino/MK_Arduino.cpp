// MK_Arduino.cpp

#include "MK_Arduino.h"

MK_Arduino mk;

//----------------------------------------------------------------------------
double Kp = 0.6, Ki = 5, Kd = 0;
//----------------------------------------------------------------------------
MK_Arduino *MK_Arduino::pxfarduino = NULL;
//----------------------------------------------------------------------------
char MK_Arduino::PinStr[P_MAX_TYPE] =
{
  0,
  1,
  2,
  3,
  4,
  5,
  6,
  7,
  8,
  9,
  10,
  11,
  12,
  13,
  30,
  31,
  32,
  33,
  34,
  35,
  36
};
//----------------------------------------------------------------------------
unsigned char MK_Arduino::sOptTypeVal[OT_MAX_TYPE] =
    {
        100, //OT_TOGET_NETID
        101, //OT_RETRUN_NETID
        0,   //OT_PM
        1,   //OT_DW
        2,   //OT_AW
        3,   //OT_RETURN_DR
        4,   //OT_RETURN_AR
        5,   //OT_SVR_I
        6,   //OT_SVR_W
        7,   //OT_DST_I
        8,   //OT_DST_T
        9,   //OT_RETURN_DIST
        10,  //OT_MOTO_I
        11,  //OT_MOTO_RUN
        12,  //OT_MOTO_RUNSIMPLE
        13,  //OT_MOTO_STOP
        14,  //OT_MOTO_I_SPD
        15,  //OT_RETURN_MOTOSPD
        16,  //OT_MOTO_I_DRIVER4567
        17,  //OT_MOTO_I_DRIVER298N
        18,  //OT_MP3_INIT
        19,  //OT_MP3_DO
        20,  //OT_MP3_PLAYFOLDER
        21,  //OT_MP3_SETVOLUME
        24,  //OT_IR_INIT
        25,  //OT_IR_SEND
        26,  //OT_RETURN_IR
        27,  //OT_HX711_I
        28,  //OT_HX711_TEST
        29,  //OT_RETURN_HX711
        30,  //OT_DSTMAT_I
        31,  //OT_RETURN_DSTMAT
        32,  //OT_AXIS_I
        33,  //OT_RETURN_AXIS
        34,  //OT_SET_TIME,
        35,  //OT_RC_INIT
        36,  //OT_RC_SEND
        37,  //OT_RETRUN_RC
        38,  //OT_DHT_I
        39,  //OT_RETURN_DHTTEMP
        40,  //OT_RETURN_DHTHUMI
        41,  //OT_LEDSTRIP_I
        42,  //OT_LEDSTRIP_SET
        43,  //OT_SEGMENT_I
        44,  //OT_SEGMENT_BRIGHTNESS
        45,  //OT_SEGMENT_CLEAR
        46,  //OT_SEGMENT_DISPLAY
        47,  //OT_LEDMATRIX_I
        48,  //OT_LEDMATRIX_BRIGHTNESS
        49,  //OT_LEDMATRIX_CLEARSCREEN
        50,  //OT_LEDMATRIX_LIGHTAT
        51,  //OT_STEPMOTO_I
        52,  //OT_STEPMOTO_ENABLE
        53,  //OT_STEPMOTO_DIR
        54,  //OT_STEPMOTO_STEP
        55,  //OT_LCI2C_INIT
        56,  //OT_LCI2C_DO
        57,  //OT_LCI2C_SETCURSOR
        58,  //OT_LCI2C_SETBACKLIGHT
        59,  //OT_LCI2C_PRINT
        60,  //OT_LCI2C_PRINTBYTE
        150, //OT_MC_INTERNAL_LIGHT
        151, //OT_MC_LIGHT
        152, //OT_MC_SEGMENT
        153, //OT_MC_MOTO
        154, //OT_MC_DISTTEST
        161, //OT_MB_MOTO
        162, //OT_MB_SEND
        163, //OT_MB_BUZZER
        200  //OT_VERSION
};
//----------------------------------------------------------------------------
MK_Arduino::MK_Arduino()
{
  pxfarduino = this;

  mNetID = 12345678;
  
  Init(false);
}
//----------------------------------------------------------------------------
#if defined MK_ESP_NETWORK
String _GetContentType(String filename)
{
  if (mk.mESPServer.hasArg("download"))
    return "application/octet-stream";
    
  else if (filename.endsWith(".htm"))
    return "text/html";
  else if (filename.endsWith(".html"))
    return "text/html";
  else if (filename.endsWith(".css"))
    return "text/css";
  else if (filename.endsWith(".js"))
    return "application/javascript";
  else if (filename.endsWith(".png"))
    return "image/png";
  else if (filename.endsWith(".gif"))
    return "image/gif";
  else if (filename.endsWith(".jpg"))
    return "image/jpeg";
  else if (filename.endsWith(".ico"))
    return "image/x-icon";
  else if (filename.endsWith(".xml"))
    return "text/xml";
  else if (filename.endsWith(".pdf"))
    return "application/x-pdf";
  else if (filename.endsWith(".zip"))
    return "application/x-zip";
  else if (filename.endsWith(".gz"))
    return "application/x-gzip";
  return "text/plain";
}
bool _HandleFileRead(String path)
{
  Serial.println("handleFileRead: " + path);
  if (path.endsWith("/"))
    path += "index.htm";
  String contentType = _GetContentType(path);
  if (SPIFFS.exists(path))
  {
    File file = SPIFFS.open(path, "r");
    size_t sent = mk.mESPServer.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}
//----------------------------------------------------------------------------
const char* host = "MANYKIT";
void _ESPTimerAfter()
{
  IPAddress ip = WiFi.localIP();
  IPAddress ipzero;
  ipzero.fromString("0.0.0.0");
  while (ip == ipzero)
  {
    ip = WiFi.localIP();
    Serial.print("IP address:");
    Serial.println(ip);
    delay(500);
  }

  // file system
  SPIFFS.begin();

  if (!MDNS.begin(host, ip))
  {
    Serial.println("Error setting up MDNS responder");
    while (1)
    {
      delay(500);
    }
  }
  Serial.println("MDNS responder started!");

  mk.mESPServer.on("/ck01", []() {
    String msg;
    uint8_t numArg = mk.mESPServer.args();
    msg += "numarg:" + mk.I2Str(numArg);
    msg += "\n";
    for (uint8_t i = 0; i < numArg; i++)
    {
      msg +=  mk.mESPServer.argName(i) + ":" + mk.mESPServer.arg(i);
      if (i < numArg-1)
         msg += " "; 
    }
    Serial.println(msg);
    mk.mESPServer.send(202, "text/plain", "Suc:" + msg);
  });

  // process root request path & not exist path
  mk.mESPServer.onNotFound([]() {
    if (!_HandleFileRead(mk.mESPServer.uri()))
      mk.mESPServer.send(404, "text/plain", "FileNotFound");
  });

  mk.mESPServer.begin();
  Serial.println("HTTP server started");

  String ipStr = ip.toString();
  MDNS.setInstanceName("MANYKIT IOT:" + ipStr);
  MDNS.addService("http", "tcp", 80);

  mk.mWiFiUDP.begin(mk.mESPUDPPort);

  Serial.println("ESP inited!");
  mk.mIsESPInited = true;
}
#endif
//----------------------------------------------------------------------------
void MK_Arduino::Init(bool isReset)
{
  RecvStr = "";

  mSettedTimeMe = 0;
  mLastSendVersionTime = 0;
  mLastSendGeneralTime = 0;

  mpCMD = NULL;

  mpCMD = 0;

  mPinDistTrigger = 0;
  mPinDistEcho = 0;
  mDist = 0.0f;
  mDistCheckLastTime = 0;

  mPinL0 = 0;
  mPinL1 = 0;
  mPinLSpeed = 0;
  mPinR0 = 0;
  mPinR1 = 0;
  mPinRSpeed = 0;

  mIsUseSpeedEncorder = false;

#if defined MK_MOTO
  mMotoMode = MM_BOARD;
#endif
  
  mPinEncroderLA = 2;
  mPinEncroderLB = 8;
  mPinEncroderRA = 3;
  mPinEncroderRB = 9;

#if defined MK_PID
  mEncoder0PinALastL = 0;
  mEncoder0PinALastR = 0;
  mDurationLTemp = 0;
  mDurationRTemp = 0;
  mAbsDurationL = 0;
  mAbsDurationR = 0;
  mIsStopL = true;
  mIsStopR = true;
  mValOutputL = 0;
  mValOutputR = 0;
  mDirectionL = true;
  mDirectionR = true;

  mPID = 0;
  mPID1 = 0;
#endif

#if defined MK_SSD1306
  mDisplay = 0;                                                     
#endif

#if defined MK_DFMP3
  if (isReset && mMP3DFSerial)
  {
    delete mMP3DFSerial;
  }
  mMP3DFSerial = 0;
#endif

#if defined MK_STEPMOTO
  for (int i=0; i<NumMaxStepMoto; i++)
  {
    mStepMotoVCCPin[i] = 0;
    mStepMotoPLSPin[i] = 0;
    mStepMotoDirPin[i] = 0;
    mStepMotoEnablePin[i] = 0;
    
    mStepMotoEnable[i] = false;
    mStepMotoRunDelay[i] = 100;
  }
#endif

#if defined MK_IR
  if (isReset && mIRrecv)
  {
    delete mIRrecv;
  }
  mIRrecv = 0;
  mIRRecvValue = 0;
#endif

#if defined MK_AXIS
  mPitch = 0.0f;
  mRoll = 0.0f;
  mYaw = 0.0f;
  mAxisTickEvent = -1;
  mAxisLastTick = 0;
#endif

#if defined MK_PID
  if (isReset && mPID)
  {
    delete(mPID);
  }
  mPID = 0;

  if (isReset && mPID1)
  {
    delete(mPID1);
  }
  mPID1 = 0;
#endif

#if defined MK_DHT
  mIsInitedDHT = false;
  mTemperature = 0;
  mHumidity = 0;
#endif

#if defined MK_SCREEN_I2C
  if (isReset && mLiquidCrystal_I2C)
  {
    delete (mLiquidCrystal_I2C);
  }
  mLiquidCrystal_I2C = 0;
#endif

#if defined MA_AXIS
  mk._InitAxis();
#endif

#if defined MK_ESP_NETWORK
  mESPUDPPort = 2333;
  mIsESPInited = false;

  mRemotePort = 0;

  if (isReset)
  {
    Serial.begin(9600);
    Serial.println("manykit arduino use esp mode");
    
    mESPServer = ESP8266WebServer(80);
    if (!AutoConfig())
    {
      SmartConfig();
    }
     mTimer.after(1000, _ESPTimerAfter);
 }  
#else
  if (isReset)
  {
     Serial.begin(9600);
  }
  digitalWrite(13, LOW);
#endif
}
//----------------------------------------------------------------------------
String MK_Arduino::F2Str(float val)
{
  char str[16];
  dtostrf(val, 12, 4, str);
  return str;
}
//----------------------------------------------------------------------------
String MK_Arduino::I2Str(int val)
{
  char str[25];
  itoa(val, str, 10); // 10 is decimal
  return str;
}
//----------------------------------------------------------------------------
void MK_Arduino::_SendNetID()
{
    char str[32];
    memset(str, 0, 32);
    itoa(mNetID, str, 10); // 10 is decimal

    unsigned char cmdCh = sOptTypeVal[OT_RETURN_NETID];
    char strCMDCh[32];
    memset(strCMDCh, 0, 32);
    itoa(cmdCh, strCMDCh, 10);

    String sendStr = String(strCMDCh) + " " + str;
    String lastCmdStr = "0000" + sendStr;
    Serial.println(lastCmdStr);
}
//----------------------------------------------------------------------------
void MK_Arduino::_SendVersion()
{
    unsigned char cmdCh = sOptTypeVal[OT_VERSION];
    char strCMDCh[32];
    memset(strCMDCh, 0, 32);
    itoa(cmdCh, strCMDCh, 10);
        
    Serial.print("0000");
    Serial.println(String(strCMDCh));

#if defined MK_ESP_NETWORK
    if (0 != mRemotePort)
    {
      String dst = String("0000") + String(strCMDCh) + String("\n");

      mWiFiUDP.beginPacket(mRemoteIP, mRemotePort);
      mWiFiUDP.write((const char*)&dst[0], dst.length());
      mWiFiUDP.endPacket();
    }
#endif
}
//----------------------------------------------------------------------------
bool result = false;
bool resultR = false;
bool doSendVersion = true;
//----------------------------------------------------------------------------
void MK_Arduino::Tick()
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
        mk.OnCMD(mCmdParams, cmdStr);
      }
      mk.RecvStr = "";
    }
    else
    {
      mk.RecvStr += c;
    }
  }

  mTimer.update();

#if defined MK_ESP_NETWORK
  doSendVersion = mIsESPInited;    
#endif

   if (doSendVersion)
   {
      unsigned long tick = millis();       
      if (tick - mLastSendVersionTime >= 2000)
      {
          mLastSendVersionTime =tick;
          _SendVersion();
      }  
   }

#if defined MK_STEPMOTO
  for (int i=0; i<NumMaxStepMoto; i++)
  {
    if(mStepMotoEnable[i])
    {
      digitalWrite(mStepMotoPLSPin[i], HIGH);
      delayMicroseconds(mStepMotoRunDelay[i]);
      digitalWrite(mStepMotoPLSPin[i], LOW);
      delayMicroseconds(mStepMotoRunDelay[i]);
    }
  }
#endif
#if defined MK_IR
  decode_results iresultes;
  if (mIRrecv && mIRrecv->decode(&iresultes))
  {
    int val = iresultes.value;
    mIRrecv->resume();
    _SendIRRecv(val);
  }
#endif
#if defined MK_PID
  if (mIsUseSpeedEncorder && mPID && mPID1)
  {
    if (!mIsStopL)
      analogWrite(mPinLSpeed, (int)mValOutputL);
    if (!mIsStopR)
      analogWrite(mPinRSpeed, (int)mValOutputR);

    mAbsDurationL = abs(mDurationLTemp);
    mAbsDurationR = abs(mDurationRTemp);

    result = mPID->Compute();
    resultR = mPID1->Compute();
    if (result)
      mDurationLTemp = 0;
    if (resultR)
      mDurationRTemp = 0;

    if (millis() - mLastSendGeneralTime >= 100)
    {
      mLastSendGeneralTime = millis();
      _VehicleSendSpeed();
    }
  }
#endif
#if defined MK_RCSWITCH
 if (mRCSwitch.available()) 
 {   
    int recvVal = mRCSwitch.getReceivedValue();

    unsigned char cmdCh = sOptTypeVal[OT_RETRUN_RC];
    char strCMDCh[32];
    memset(strCMDCh, 0, 32);
    itoa(cmdCh, strCMDCh, 10);
    
    Serial.print("0000");
    Serial.print(String(strCMDCh)); 
    Serial.print(" ");
    Serial.println(recvVal);

#if defined MK_ESP_NETWORK
    if (0 != mRemotePort)
    {
      String dst = String("0000") + String(strCMDCh) + String(" ") + I2Str(recvVal) + String("\n");

      mWiFiUDP.beginPacket(mRemoteIP, mRemotePort);
      mWiFiUDP.write((const char *)&dst[0], dst.length());
      mWiFiUDP.endPacket();
    }
#endif

    mRCSwitch.resetAvailable();
  }
#endif
#if defined MK_DHT
  _DHTSendTemperatureHumidity();
#endif
#if defined MK_ESP_NETWORK
  if (mIsESPInited)
  {
    mESPServer.handleClient();

    int packetSize = mWiFiUDP.parsePacket();
    if (packetSize > 0)
    {
      mRemoteIP = mWiFiUDP.remoteIP();
      mRemotePort = mWiFiUDP.remotePort();

      int len = mWiFiUDP.read(mIncomingPacket, 536);
      if (len > 0)
      {
        mIncomingPacket[len] = 0;
        String strIncoming(mIncomingPacket);
        String cmdStr = strIncoming.substring(4, strIncoming.length()-1);
        mk.OnCMD(mCmdParams, cmdStr);
      }
    }
  }
#endif
}
//----------------------------------------------------------------------------
void MK_Arduino::_DistInit_(int pinTrig, int pinEcho)
{
  mPinDistTrigger = pinTrig;
  mPinDistEcho = pinEcho;
  pinMode(mPinDistTrigger, OUTPUT); // 鐎规矮绠熺搾鍛紣濞夈垼绶崙楦垮壖
  pinMode(mPinDistEcho, INPUT);     // 鐎规矮绠熺搾鍛紣濞夈垼绶崗銉ㄥ壖

  mDist = 0.0f;
  mDistCheckLastTime = 0;
}
//----------------------------------------------------------------------------
void MK_Arduino::_DistTest()
{
  digitalWrite(mPinDistTrigger, LOW); //娴ｅ酣鐝担搴ｆ暩楠炲啿褰傛稉锟芥稉顏嗙叚閺冨爼妫块懘澶婂暱閸樼睒rigPin
  delayMicroseconds(2);
  digitalWrite(mPinDistTrigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(mPinDistTrigger, LOW);

  float dist = pulseIn(mPinDistEcho, HIGH) / 58.0; //鐏忓棗娲栧▔銏℃闂傚瓨宕茬粻妤佸灇cm
  dist = (int(dist * 100.0)) / 100.0;       //娣囨繄鏆�娑撱倓缍呯亸蹇旀殶
  if (2 <= dist && dist <= 400)
  {
    mDist = dist;
  }
}
#if defined MK_MOTO
//----------------------------------------------------------------------------
void MK_Arduino::MotoInit10111213()
{
  mPinL0 = 12;
  mPinL1 = 12;
  mPinR0 = 13;
  mPinR1 = 13;
  mPinLSpeed = 10;
  mPinRSpeed = 11;

  pinMode(mPinL0, OUTPUT);
  pinMode(mPinR0, OUTPUT);
  pinMode(mPinLSpeed, OUTPUT);
  pinMode(mPinRSpeed, OUTPUT);

  mMotoMode = MM_BOARD;
  mIsUseSpeedEncorder = false;
}
//----------------------------------------------------------------------------
void MK_Arduino::MotoInit4567()
{
  mPinL0 = 4;
  mPinL1 = 4;
  mPinR0 = 7;
  mPinR1 = 7;
  mPinLSpeed = 5;
  mPinRSpeed = 6;

  pinMode(mPinL0, OUTPUT);
  pinMode(mPinR0, OUTPUT);
  pinMode(mPinLSpeed, OUTPUT);
  pinMode(mPinRSpeed, OUTPUT);

#if defined MK_AXIS
  mPitch = 0.0f;
  mRoll = 0.0f;
  mYaw = 0.0f;
#endif

  mMotoMode = MM_BOARD;
  mIsUseSpeedEncorder = false;
}
//----------------------------------------------------------------------------
void MK_Arduino::MotoInit298N(int pinL, int pinL1, int pinLS, int pinR, 
  int pinR1, int pinRS)
{
  mPinL0 = pinL;
  mPinL1 = pinL1;
  mPinLSpeed = pinLS;
  
  mPinR0 = pinR;
  mPinR1 = pinR1;
  mPinRSpeed = pinRS;

  pinMode(mPinL0, OUTPUT);
  pinMode(mPinL1, OUTPUT);
  pinMode(mPinLSpeed, OUTPUT);
    
  pinMode(mPinR0, OUTPUT);
  pinMode(mPinR1, OUTPUT);
  pinMode(mPinRSpeed, OUTPUT);

  mMotoMode = MM_298N;
  mIsUseSpeedEncorder = false;

#if defined MK_AXIS
  mPitch = 0.0f;
  mRoll = 0.0f;
  mYaw = 0.0f;
#endif
}
#endif
//----------------------------------------------------------------------------
#if defined MK_PID
void wheelSpeedL()
{
  int lstate = digitalRead(MK_Arduino::pxfarduino->mPinEncroderLA);
  if((MK_Arduino::pxfarduino->mEncoder0PinALastL == LOW) && lstate==HIGH)
  {
    int val = digitalRead(MK_Arduino::pxfarduino->mPinEncroderLB);
    if(val == LOW && MK_Arduino::pxfarduino->mDirectionL)
    {
      MK_Arduino::pxfarduino->mDirectionL = false; //Reverse
    }
    else if(val == HIGH && !MK_Arduino::pxfarduino->mDirectionL)
    {
      MK_Arduino::pxfarduino->mDirectionL = true;  //Forward
    }
  }
  MK_Arduino::pxfarduino->mEncoder0PinALastL = lstate;
 
  if(!MK_Arduino::pxfarduino->mDirectionL) 
    MK_Arduino::pxfarduino->mDurationLTemp++;
  else  
    MK_Arduino::pxfarduino->mDurationLTemp--;
}
void wheelSpeedR()
{
  int lstate = digitalRead(MK_Arduino::pxfarduino->mPinEncroderRA);
  if((MK_Arduino::pxfarduino->mEncoder0PinALastR == LOW) && lstate==HIGH)
  {
    int val = digitalRead(MK_Arduino::pxfarduino->mPinEncroderRB);
    if(val == LOW && !MK_Arduino::pxfarduino->mDirectionR)
    {
      MK_Arduino::pxfarduino->mDirectionR = true; //Reverse
    }
    else if(val == HIGH && MK_Arduino::pxfarduino->mDirectionR)
    {
      MK_Arduino::pxfarduino->mDirectionR = false;  //Forward
    }
  }
  MK_Arduino::pxfarduino->mEncoder0PinALastR = lstate;
 
  if(!MK_Arduino::pxfarduino->mDirectionR)
    MK_Arduino::pxfarduino->mDurationRTemp++;
  else 
   MK_Arduino::pxfarduino->mDurationRTemp--;
}
#endif
#if defined MK_MOTO
double MK_Arduino::VehicleGetSpeed(int i)
{
#if defined MK_PID
  if (0 == i)
    return mAbsDurationL;
  else if (1 == i)
    return mAbsDurationR;
#endif

  return 0.0;
}
int MK_Arduino::VehicleGetDir(int i)
{
#if defined MK_PID
  if (0 == i)
    return mDirectionL == true ? 0 : 1;
  else if (1 == i)
    return mDirectionR == true ? 0 : 1;
#endif

  return 0;
}
void MK_Arduino::_VehicleSendSpeed()
{
#if defined MK_PID
  unsigned char cmdCh = sOptTypeVal[OT_RETURN_MOTOSPD];
  char strCMDCh[32];
  memset(strCMDCh, 0, 32);
  itoa(cmdCh, strCMDCh, 10);

  Serial.print("0000");
  Serial.print(String(strCMDCh));
  Serial.print(" ");
  Serial.print(mAbsDurationL);
  Serial.print(" ");
  Serial.print(mDirectionL ? 1:0);
  Serial.print(" ");
  Serial.println(mAbsDurationR);
  Serial.print(" ");
  Serial.println(mDirectionR ? 1:0);

#if defined MK_ESP_NETWORK
  if (0 != mRemotePort)
  {
    String dst = String("0000") + String(strCMDCh) +
                 String(" ") + F2Str(mAbsDurationL) +
                 String(" ") + F2Str(mDirectionL ? 1 : 0) +
                 String(" ") + F2Str(mAbsDurationR) +
                 String(" ") + F2Str(mDirectionR ? 1 : 0) +
                 String("\n");

    mWiFiUDP.beginPacket(mRemoteIP, mRemotePort);
    mWiFiUDP.write((const char *)&dst[0], dst.length());
    mWiFiUDP.endPacket();
  }
#endif

#if defined MK_ESP_NETWORK
  if (0 != mRemotePort)
  {
    String dst = String("0000") + String(strCMDCh) + 
      String(" ") + I2Str(recvVal) + String("\n");

    mWiFiUDP.beginPacket(mRemoteIP, mRemotePort);
    mWiFiUDP.write((const char *)&dst[0], dst.length());
    mWiFiUDP.endPacket();
  }
#endif

#endif
}
//----------------------------------------------------------------------------
void MK_Arduino::MotoSpeedInit(int encorderLA, int encorderLB,
  int encorderRA, int encorderRB)
{
  mPinEncroderLA = encorderLA;
  mPinEncroderLB = encorderLB;
  mPinEncroderRA = encorderRA;
  mPinEncroderRB = encorderRB;

  if (encorderLA == encorderLB && encorderRA==encorderRB)
  {
    mIsUseSpeedEncorder = false;

#if defined MK_PID
    if (mPID)
    {
      delete(mPID);
    }
    mPID = 0;
    if (mPID1)
    {
      delete(mPID1);
    }
    mPID1 = 0;
    mEncoder0PinALastL = 0;
    mEncoder0PinALastR = 0;
    mDurationLTemp = 0;
    mDurationRTemp = 0;
    mAbsDurationL = 0;
    mAbsDurationR = 0;
    mValOutputL = 0;
    mValOutputR = 0;
    mDirectionL = true;
    mDirectionR = true;
#endif
  }
  else
  {
    pinMode(mPinEncroderLB, INPUT);
    pinMode(mPinEncroderRB, INPUT);

    mIsUseSpeedEncorder = true;

#if defined MK_PID
    mEncoder0PinALastL = 0;
    mEncoder0PinALastR = 0;
    mDurationLTemp = 0;
    mDurationRTemp = 0;
    mAbsDurationL = 0;
    mAbsDurationR = 0;
    mValOutputL = 0;
    mValOutputR = 0;
    mDirectionL = true;
    mDirectionR = true;

    if (mPID)
    {
      delete(mPID);
      mPID = 0;
    }
    mPID = new PID(&mAbsDurationL, &mValOutputL, &mSetPoint_L, Kp, Ki, Kd, DIRECT);

    if (mPID1)
    {
      delete(mPID1);
      mPID1 = 0;
    }
    mPID1 = new PID(&mAbsDurationR, &mValOutputR, &mSetPoint_R, Kp, Ki, Kd, DIRECT);

    mPID->SetMode(AUTOMATIC);
    mPID->SetSampleTime(100);
    mPID1->SetMode(AUTOMATIC);
    mPID1->SetSampleTime(100);
    
    mDirectionL = true;
    pinMode(mPinEncroderLB, INPUT);  
    attachInterrupt(0, wheelSpeedL, CHANGE);
  
    mDirectionR = true;
    pinMode(mPinEncroderRB, INPUT);  
    attachInterrupt(1, wheelSpeedR, CHANGE);
#endif 
  }
}
void MK_Arduino::VehicleRun(int i, int dir, int spd)
{
  if (0 == dir)
    spd = 0;

  if (0 == i)
  {
    LeftRun(dir, spd);
  }
  else if (1 == i)
  {
    RightRun(dir, spd);
  }
}
//----------------------------------------------------------------------------
void MK_Arduino::VehicleSimpleRun(int dir, int spd)
{
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
}
//----------------------------------------------------------------------------
void MK_Arduino::VehicleStop()
{
  LeftRun(0, 0);
  RightRun(0, 0);
}
//----------------------------------------------------------------------------
void MK_Arduino::LeftRun(int val, int spd)
{
  if (MM_BOARD == mMotoMode)
  {
    if (mIsUseSpeedEncorder)
    {
      mSetPoint_L = spd;

      if (0 == val)
      {
        analogWrite(mPinLSpeed, 0);
        mSetPoint_L = 0;
        #if defined MK_PID
        mIsStopL=true;
        #endif
      }
      else if (1 == val)
      {
        digitalWrite(mPinL0, HIGH);
        #if defined MK_PID
        mIsStopL=false; 
        #endif
      }
      else if (2 == val)
      {
        digitalWrite(mPinL0, LOW);
        #if defined MK_PID
        mIsStopL=false;
        #endif
      }
    }
    else
    {
      mSetPoint_L = spd;
      analogWrite(mPinLSpeed, (int)mSetPoint_L);

      if (0 == val)
      {
        #if defined MK_PID
        mIsStopL=true;
        #endif
      }
      if (1 == val)
      {
        digitalWrite(mPinL0, HIGH);
        #if defined MK_PID
        mIsStopL=false; 
        #endif
      }
      else if (2 == val)
      {
        digitalWrite(mPinL0, LOW); 
        #if defined MK_PID
        mIsStopL=false; 
        #endif
      }
    }
  }
  else if (MM_298N == mMotoMode)
  {
    if (mIsUseSpeedEncorder)
    {
      mSetPoint_L = spd;
      
      if (0 == val)
      {
        digitalWrite(mPinL0, LOW);
        digitalWrite(mPinL1, LOW);
        analogWrite(mPinLSpeed, 0);
        mSetPoint_L = 0;
#if defined MK_PID
      mIsStopL=true;
#endif
      }
      else if (1 == val)
      {
        digitalWrite(mPinL0, HIGH);
        digitalWrite(mPinL1, LOW);
#if defined MK_PID
        mIsStopL=false;
#endif
      }
      else if (2 == val)
      {
        digitalWrite(mPinL0, LOW);
        digitalWrite(mPinL1, HIGH);
#if defined MK_PID
        mIsStopL=false; 
#endif
      }
    }
    else
    {
      mSetPoint_L = spd;
      
      if (0 == val)
      {
        digitalWrite(mPinL0, LOW);
        digitalWrite(mPinL1, LOW);
        mSetPoint_L = 0;

#if defined MK_PID
        mIsStopL=true; 
#endif
      }
      else if (1 == val)
      {
        digitalWrite(mPinL0, HIGH);
        digitalWrite(mPinL1, LOW);

#if defined MK_PID
        mIsStopL=false; 
#endif
      }
      else if (2 == val)
      {
        digitalWrite(mPinL0, LOW);
        digitalWrite(mPinL1, HIGH);

#if defined MK_PID
        mIsStopL=false; 
#endif
      }

      analogWrite(mPinLSpeed, (int)mSetPoint_L);
    }
  }
}
//----------------------------------------------------------------------------
void MK_Arduino::RightRun(int val, int spd)
{
  if (MM_BOARD == mMotoMode)
  {
    if (mIsUseSpeedEncorder)
    {
      mSetPoint_R = spd;

      if (0 == val)
      {
        analogWrite(mPinRSpeed, 0); 
#if defined MK_PID
        mIsStopR=true;
#endif
        mSetPoint_R = 0;
      }
      else if (1 == val)
      {
        digitalWrite(mPinR0, HIGH);
#if defined MK_PID
       mIsStopR=false; 
#endif
      }
      else if (2 == val)
      {
        digitalWrite(mPinR0, LOW); 
#if defined MK_PID
        mIsStopR=false; 
#endif
      }
    }
    else
    {
      mSetPoint_R = spd;
      analogWrite(mPinRSpeed, (int)mSetPoint_R);

      if (0 == val)
      {
#if defined MK_PID
        mIsStopR=true;
#endif
      }
      if (1 == val)
      {
        digitalWrite(mPinR0, HIGH);
#if defined MK_PID
        mIsStopR=false; 
#endif
      }
      else if (2 == val)
      {
        digitalWrite(mPinR0, LOW); 
#if defined MK_PID
        mIsStopR=false; 
#endif
      }
    }
  }
  else if (MM_298N == mMotoMode)
  {
    if (mIsUseSpeedEncorder)
    { 
      mSetPoint_R = spd;       
          
      if (0 == val)
      {
        digitalWrite(mPinR0, LOW);
        digitalWrite(mPinR1, LOW);
        analogWrite(mPinRSpeed, 0); 
        mSetPoint_R = 0;
#if defined MK_PID
        mIsStopR=true;
#endif
      }
      else if (1 == val)
      {
        digitalWrite(mPinR0, LOW);
        digitalWrite(mPinR1, HIGH);
#if defined MK_PID
        mIsStopR=false;
#endif
      }
      else if (2 == val)
      {
        digitalWrite(mPinR0, HIGH);
        digitalWrite(mPinR1, LOW);

#if defined MK_PID
        mIsStopR=false;
#endif
      }
    }
    else
    {
      mSetPoint_R = spd;

      if (0 == val)
      {
        digitalWrite(mPinR0, LOW);
        digitalWrite(mPinR1, LOW);
        mSetPoint_R = 0;
        
#if defined MK_PID
        mIsStopR=true;
#endif
      }
      else if (1 == val)
      {
        digitalWrite(mPinR0, LOW);
        digitalWrite(mPinR1, HIGH);

#if defined MK_PID
        mIsStopR=false;
#endif
      }
      else if (2 == val)
      {
        digitalWrite(mPinR0, HIGH);
        digitalWrite(mPinR1, LOW);
        
#if defined MK_PID
        mIsStopR=false;
#endif
      }

      analogWrite(mPinRSpeed, (int)mSetPoint_R);
    }
  }
}
#endif
//----------------------------------------------------------------------------
void sAxisTickTimer()
{
#if defined MK_AXIS
  if (0==MK_Arduino::pxfarduino->mSettedTimeMe)
    return;

  Vector norm = MK_Arduino::pxfarduino->mMPU.readNormalizeGyro();
  unsigned long curTimer = millis();
  unsigned long timeStep = curTimer - MK_Arduino::pxfarduino->mAxisLastTick;
  MK_Arduino::pxfarduino->mAxisLastTick = curTimer;
  float fTime = timeStep*0.001;

  MK_Arduino::pxfarduino->mPitch = MK_Arduino::pxfarduino->mPitch + norm.YAxis * fTime;
  MK_Arduino::pxfarduino->mRoll = MK_Arduino::pxfarduino->mRoll + norm.XAxis * fTime;
  MK_Arduino::pxfarduino->mYaw = MK_Arduino::pxfarduino->mYaw + norm.ZAxis * fTime;

  char cmdCh = MK_Arduino::pxfarduino->sOptTypeVal[OT_RETURN_AXIS]; 
  char strCMDCh[32];
  memset(strCMDCh, 0, 32);
  itoa(cmdCh, strCMDCh, 10);

  unsigned long sendTime = curTimer - MK_Arduino::pxfarduino->mSettedTimeMe;
  Serial.print("0000");
  Serial.print(String(strCMDCh));
  Serial.print(" ");
  Serial.print(sendTime);
  Serial.print(" ");
  Serial.print(norm.YAxis);
  Serial.print(" ");
  Serial.print(norm.XAxis);
  Serial.print(" ");
  Serial.print(norm.ZAxis);
  Serial.print(" ");
  Serial.print(MK_Arduino::pxfarduino->mPitch);
  Serial.print(" ");
  Serial.print(MK_Arduino::pxfarduino->mRoll);  
  Serial.print(" ");
  Serial.println(MK_Arduino::pxfarduino->mYaw);
#endif
}
//----------------------------------------------------------------------------
void MK_Arduino::_SetTime()
{
  mSettedTimeMe = millis();
  if (0.0==mSettedTimeMe)
    mSettedTimeMe = 0.001;
}
#if defined MA_AXIS
//----------------------------------------------------------------------------
void MK_Arduino::_InitAxis()
{
  mMPU.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
  while(!mMPU.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    delay(1000);
  }

  mMPU.calibrateGyro();
  mMPU.setThreshold(3);

  mPitch = 0.0f;
  mRoll = 0.0f;
  mYaw = 0.0f;

  mAxisLastTick = 0;
  if (-1 != mAxisTickEvent)
    mTimer.stop(mAxisTickEvent);
  mAxisTickEvent = -1;
  mAxisTickEvent = mTimer.every(50, sAxisTickTimer);

  mAxisLastTick = millis();
}
#endif
//----------------------------------------------------------------------------
void MK_Arduino::_Delay(float seconds)
{
    long endTime = millis() + seconds * 1000;
    while(millis() < endTime)
      _Loop();
}
//----------------------------------------------------------------------------
void MK_Arduino::_Loop()
{
}
//----------------------------------------------------------------------------
#if defined MK_STEPMOTO
void MK_Arduino::StepMotoInit(int index, int pinVCC, int pincPLS, 
  int pinDir, int pinEnable)
{
  if (0<=index && index<NumMaxStepMoto)
  {
    mStepMotoVCCPin[index] = pinVCC;
    mStepMotoPLSPin[index] = pincPLS;
    mStepMotoDirPin[index] = pinDir;
    mStepMotoEnablePin[index] = pinEnable;
    
    mStepMotoEnable[index] = false;
    mStepMotoRunDelay[index] = 100;
    mStepMotoRunTick[index] = millis();
    mStepMotoRunVal[index] = 0;

    pinMode(pinVCC, OUTPUT);
    pinMode(pincPLS, OUTPUT);
    pinMode(pinDir, OUTPUT);
    pinMode(pinEnable, OUTPUT);
    
    digitalWrite(pinVCC, HIGH);
    digitalWrite(pincPLS, LOW);
    digitalWrite(pinDir, HIGH);
    delay(20);
    digitalWrite(pinEnable, LOW);
  }
}
//----------------------------------------------------------------------------
void MK_Arduino::StepMotoEnable(int index, bool enable)
{
  if (0<=index && index<NumMaxStepMoto)
  {
    mStepMotoEnable[index] = enable;
    digitalWrite(mStepMotoEnablePin[index], enable?HIGH:LOW);
  }
}
//----------------------------------------------------------------------------
void MK_Arduino::StepMotoDir(int index, bool forward)
{
  if (0<=index && index<NumMaxStepMoto)
  {
    digitalWrite(mStepMotoDirPin[index], forward?HIGH:LOW);
  }
}
//----------------------------------------------------------------------------
void MK_Arduino::StepMotoStep(int index, int delayVal)
{
  if (0<=index && index<NumMaxStepMoto)
  {
     mStepMotoRunDelay[index] = delayVal;
  }
}
//----------------------------------------------------------------------------
#endif
