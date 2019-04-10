// MK_Arduino.h

#ifndef MK_ARDUINO_H
#define MK_ARDUINO_H

#include <Arduino.h>

#include "MK_ArduinoConfig.h"

#include "MK_Timer.h"

// server
#if defined MK_SERVO
#include <Servo.h>
#endif

// display
#if defined MK_SSD1306
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "MK_Adafruit_SSD1306.h"
#endif

#if defined MK_DFMP3
#include "MK_DFPlayer_Mini_Mp3.h"
#include <SoftwareSerial.h>
#endif

#if defined MK_IR
#include "MK_IRremote.h"
#endif

#if defined MK_XH711
#include "MK_HX711.h"
#endif

#if defined PX2_POLOLUBUZZER
#include "MK_PololuBuzzer.h"
#endif

#if defined MK_AXIS
#include <Wire.h>
#include "PXMPU6050.h"
#endif

#if defined MK_PID
#include "MK_PID_v1.h"
#endif

#if defined MK_RCSWITCH
#include "MK_RCSwitch.h"
#endif

#if defined MK_DHT
#define MK_DHTTYPE DHT11 
#include "MK_DHT.h"
#endif

#if defined MK_LEDSTRIP
#include "MK_WS2812.h"
#endif

#if defined MK_LEDMATRIX
#include "MK_LEDMatrix.h"
#endif

#if defined MK_SEGMENT7
#include "MK_SegmentDisplay.h"
#endif

#if defined MK_MP3
#include "MK_MP3_KT403A.h"
#endif

enum MK_Pin
{
    P_0 = 0,
    P_1,
    P_2,
    P_3,
    P_4,
    P_5,
    P_6,
    P_7,
    P_8,
    P_9,
    P_10,
    P_11,
    P_12,
    P_13,
    P_A0,
    P_A1,
    P_A2,
    P_A3,
    P_A4,
    P_A5,
    P_A6,
    P_MAX_TYPE
};
#define MK_A0 30

enum MK_PMode
{
    PM_INPUT = 0,
    PM_OUTPUT,
    PM_MAX_TYPE
};

#define NumMaxCMDs 10
#define GeneralServerMsgID 2

enum MK_MotoType
{
  MT_0,
  MT_1,
  MT_MAX_TYPE,
};

enum MK_DirectionType
{
  DT_NONE,
  DT_FORWORD,
  DT_BACKWORD,
  DT_MAX_TYPE
};

enum MK_SimpleDirectionType
{
  SDT_NONE,
  SDT_FORWORD,
  SDT_BACKWORD,
  SDT_LEFT,
  SDT_RIGHT,
  SDT_MAX_TYPE
};

enum OptionType
{
  OT_TOGET_NETID,
  OT_RETURN_NETID,
  OT_PM,
  OT_DW,
  OT_AW,
  OT_RETURN_DR,
  OT_RETURN_AR,
  OT_SVR_I,
  OT_SVR_W,
  OT_DST_I,
  OT_DST_T,
  OT_RETURN_DIST,
  OT_MOTO_I,
  OT_MOTO_RUN,
  OT_MOTO_RUNSIMPLE,
  OT_MOTO_STOP,
  OT_MOTO_I_SPD,
  OT_RETURN_MOTOSPD,
  OT_MOTO_I_DRIVER4567,
  OT_MOTO_I_DRIVER298N,
  OT_MP3_INIT,
  OT_MP3_DO,
  OT_MP3_PLAYFOLDER,
  OT_MP3_SETVOLUME,
  OT_IR_INIT,
  OT_IR_SEND,
  OT_RETURN_IR,
  OT_HX711_I,
  OT_HX711_TEST,
  OT_RETURN_HX711,
  OT_DSTMAT_I,
	OT_RETURN_DSTMAT,
  OT_AXIS_I,
  OT_RETURN_AXIS,
  OT_SET_TIME,
  OT_RC_INIT,
  OT_RC_SEND,
  OT_RETRUN_RC,
  OT_DHT_I,
	OT_RETURN_DHTTEMP,
	OT_RETURN_DHTHUMI,
  OT_LEDSTRIP_I,
  OT_LEDSTRIP_SET,
  OT_SEGMENT_I,
  OT_SEGMENT_BRIGHTNESS,
  OT_SEGMENT_CLEAR,
  OT_SEGMENT_DISPLAY,
  OT_LEDMATRIX_I,
  OT_LEDMATRIX_BRIGHTNESS,
  OT_LEDMATRIX_CLEARSCREEN,
  OT_LEDMATRIX_LIGHTAT,
  OT_MC_INTERNAL_LIGHT, // makerclock
  OT_MC_LIGHT,
  OT_MC_SEGMENT,
  OT_MC_MOTO,
  OT_MC_DISTTEST,
  OT_MB_MOTO,	// mbot
  OT_MB_SEND,
  OT_MB_BUZZER,
  OT_VERSION,
  OT_MAX_TYPE
};

enum Mp3PlayType
{
  MPT_PLAY,
  MPT_PAUSE,
  MPT_STOP,
  MPT_NEXT,
  MPT_BEFORE,
  MPT_RANDOM,
  MPT_LOOP_SINGLE,
  MPT_LOOP_SINGLE_CLOSE,
  MPT_LOOP_ALL,
  MPT_LOOP_ALL_CLOSE,
  MPT_VOLUME_INCREASE,
  MPT_VOLUME_DECREASE,
  MPT_MAX_TYPE
};

class MK_Arduino
{
public:
  MK_Arduino();

  static MK_Arduino *pxfarduino;

  static char PinStr[P_MAX_TYPE];
  static unsigned char sOptTypeVal[OT_MAX_TYPE];

  // cmds process
  void OnCMDGroup(String &cmdStr);
  void OnCMD(String &cmdStr);

  // to get pin values by timer
  void CalPinVals();

  void _PinMode(MK_Pin pin, MK_PMode mode);
  void _DigitalWrite(MK_Pin pin, bool isHigh);
  int _DigitalRead(MK_Pin pin);
  void _PwmWrite(MK_Pin pin, int val);
  void _AnalogWrite(MK_Pin pin, int val);
  int _AnalogRead(MK_Pin pin);
  void _ServerInit(int index, MK_Pin pin);
  void _ServerWrite(int index, int val);
  void _DistInit(MK_Pin trigger, MK_Pin echo);
  float _GetDist();
  void _VehicleSpeedEncorderInit(MK_Pin encorderLA, MK_Pin encorderLB, MK_Pin encorderRA, MK_Pin encorderRB);
  void _VehicleRun(int index, MK_DirectionType dir, int speed);
  void _VehicleSimpleRun(MK_SimpleDirectionType dir, int speed);
  float _VehicleGetSpeed(int index);
  void _VehicleStop();
  void _WeightInit(int index, MK_Pin pinR, MK_Pin pinT);
  void _WeightTest(int index);
  float _GetWeight(int index);

#if defined MK_DHT
  void _DHTInit(MK_Pin pin);
  void _DHTSendTemperatureHumidity();
#endif

#if defined MK_LEDSTRIP
  void _RGBLEDInit(MK_Pin pin, int num);
  void _RGBLEDSetColor(int index, int r, int g, int b);
#endif

#if defined MK_LEDMATRIX
  void _LEDMatrixInit(int sckPin, int dinPin);
  void _LEDMatrixSetBrightness(int brightness);
  void _LEDMatrixClearScreen();
  void _LEDMatrixSetColorIndex(int iColor_Number);
  
  void _LEDMatrixDrawBitmap(int8_t x, int8_t y, uint8_t bitmap_Width, uint8_t *bitmap);
  void _LEDMatrixDrawStr(int16_t x_position, int8_t y_position, const char *str);
  void _LEDMatrixShowClock(uint8_t hour, uint8_t minute, bool isPointON = true);
  void _LEDMatrixShowNum(float value,uint8_t val= 3);

  void _LEDMatrixLightPos(int8_t x, int8_t y, int width, bool onOff);
#endif

#if defined MK_SEGMENT7
  void _SegmentInit(int clkPin, int dataPin);
  void _SegmentSetBrightness(int brightness);
  void _SegmentClear();
  void _SegmentDisplayInt(int val);
  void _SegmentDisplayFloat(float val);
#endif

  MK_PMode PinModes[P_MAX_TYPE];

private:
  String I2Str(int val);
  int _Str2IO(String &str);
  int _Str2Pin(String &str);
  MK_Pin _Str2MK_Pin(String &str);
  int MK_Pin2Pin(MK_Pin pxfPin);
  bool _Str2Bool(String &str);
  int _Str2Int(String &str);
  float _Str2Float(String &str);
  long _Str2Long(String &str);
  int _Str2DirType(String &str);
  int _Str2SimpleDirType(String &str);

  char *mpCMD;

#define NumCMDParams 7
  String mCmdParams[NumCMDParams];
  int mCMDIndexTemp;
  char *mpCMDParam;

public:
  void Init(bool isReset=false);
  void Reset();
  void Tick();

public:
  void _SendVersion();
  void _SendNetID();
  void _DistInit_(int pinTrig, int pinEcho);
  void _DistTest();
  enum MotoMode
  {
    MM_BOARD,
    MM_298N,
    MM_MAX_TYPE
  };
  void _MotoInit4567();
  void _MotoInit10111213();
  void _MotoInit298N(int pinL, int pinL1, int pinLS, int pinR, int pinR1, int pinRS);
  void _MotoSpeedInit(int encorderLA, int encorderLB, int encorderRA, int encorderRB);
  void _LeftRun(int val, int spd);
  void _RightRun(int val, int spd);
  
#if defined MK_SSD1306
  void _ScreenInit();
#endif

  void _MP3Init_(int pinR, int pinT);
  void _MP3Do(Mp3PlayType type);
  void _MP3FolderPlay(int folder, int index);
  void _MP3SetVolime(int val);
  
  void _IRInit_(int pinR);
  void _IRSend(int val);
  void _IRRecv(int val);
  void _Delay(float seconds);
  void _Loop();
  void _SendCMD(String &cmdStr);
  void _HX711Init(int index, int pinOut, int pinClk);
  float _ReadHX711(int index);
  void _HXSend(int index, float val);
  void _SetTime();

public:
  int mPinEncroderLA;
  int mPinEncroderLB;
  int mPinEncroderRA;
  int mPinEncroderRB;
  double mSetPoint_L;
  double mSetPoint_R;

#if defined MK_PID
  PID *mPID;
  PID *mPID1;
  byte mEncoder0PinALastL;
  byte mEncoder0PinALastR;
  double mDurationL;
  double mDurationR;
  double mAbsDurationL;
  double mAbsDurationR;
  double mValOutputL;
  double mValOutputR;
  bool mDirectionL;
  bool mDirectionR;
  bool mIsStopL;
  bool mIsStopR;
#endif

  unsigned long mSettedTimeMe;

private:
  int mNetID;
  unsigned long mLastSendVersionTime;
  Timer mTimer;

#if defined MK_SERVO
  #define NumMaxServer 5
  Servo mServo[NumMaxServer];
#endif

  int mPinDistTrigger;
  int mPinDistEcho;
  float mDist;
  int mDistCheckLastTime;

  int mPinL0;
  int mPinL1;
  int mPinLSpeed;
  int mPinR0;
  int mPinR1;
  int mPinRSpeed;

  bool mIsUseSpeedEncorder;

  MotoMode mMotoMode;

#if defined MK_SSD1306
  Adafruit_SSD1306 *mDisplay;
#endif

#if defined MK_DFMP3
  SoftwareSerial *mMP3DFSerial;
#endif

#if defined MK_IR
  IRrecv *mIRrecv;
  IRsend mIRsend;
#endif

#if defined MK_XH711
  HX711 mXH711_0;
  HX711 mXH711_1;
  HX711 mXH711_2;
  HX711 mXH711_3;
#endif

#if defined MK_DHT
  MK_DHT mDHT;
  bool mIsInitedDHT;
#endif

#if defined MK_LEDSTRIP
  WS2812 mWS2812;
#endif

#if defined MK_LEDMATRIX
  LEDMatrix mLEDMatrix;
#endif

#if defined MK_MP3
  MP3 mMP3;
#endif

public:
  void _InitAxis();

#if defined MK_AXIS
  unsigned long mAxisLastTick;
  float mPitch;
  float mRoll;
  float mYaw;
  MPU6050 mMPU;
  int mAxisTickEvent;
#endif

#if defined MK_RCSWITCH
public:
  void _InitRCSwitchReceive(int pinTimerIndex);
  void _RCInit(int pin);
  void _RCSend(long val);

  RCSwitch mRCSwitch;
#endif

#if defined MK_SEGMENT7
  SegmentDisplay mSegmentDisplay;
#endif
};

#endif
