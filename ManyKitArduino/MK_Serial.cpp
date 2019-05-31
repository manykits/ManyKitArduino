// MK_Serial.cpp

#include "MK_Serial.h"

#if defined MK_MP3

#include "MK_Serial.h"
#include <stdio.h>
#include <stdarg.h>

//----------------------------------------------------------------------------
MK_Serial::MK_Serial(void):
SoftwareSerial(0, 0)
{
}
//----------------------------------------------------------------------------
#define PORT_11 (0x0b)
#define PORT_12 (0x0c)
#define PORT_13 (0x0d)
#define PORT_14 (0x0e)
#define PORT_15 (0x0f)
#define PORT_16 (0x10)
//----------------------------------------------------------------------------
MK_Serial::MK_Serial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic):
 SoftwareSerial(receivePin, transmitPin, inverse_logic)
{
  _scratch = false;
  _hard = false;
  _polling = false;
  _RxPin = receivePin;
  _TxPin = transmitPin;
#if defined(__AVR_ATmega32U4__)
   _polling = false;
  if((receivePin == 0) && (transmitPin == 1))
  {
    _hard = true;
  }
  else
  {
    _hard = false;
  }
#else
  if((receivePin == 0) && (transmitPin == 1))
  {
    _hard = true;
  }
  else
  {
    _hard = false;
  }
#endif
}
//----------------------------------------------------------------------------
void MK_Serial::setHardware(bool mode)
{
  _hard = mode;
}
//----------------------------------------------------------------------------
int MK_Serial::getPort()
{
  return PORT_15;
}
//----------------------------------------------------------------------------
void MK_Serial::begin(long baudrate)
{
  _bitPeriod = 1000000 / baudrate;
  if (_hard)
  {
#if defined(__AVR_ATmega32U4__)
    _scratch ? Serial.begin(baudrate) : Serial1.begin(baudrate);
#elif defined(__AVR_ATmega2560__)
    if (getPort() == PORT_15)
    {
      Serial3.begin(baudrate);
    }
    else if (getPort() == PORT_16)
    {
      Serial.begin(baudrate);
    }
    else
    {
      Serial2.begin(baudrate);
    }
#else
    Serial.begin(baudrate);
#endif
  }
  else
  {
    SoftwareSerial::begin(baudrate);
  }
}
//----------------------------------------------------------------------------
void MK_Serial::end(void)
{
  if (_hard)
  {
#if defined(__AVR_ATmega32U4__)
    Serial1.end();
#elif defined(__AVR_ATmega2560__)
    if (getPort() == PORT_15)
    {
      Serial3.end();
    }
    else if (getPort() == PORT_16)
    {
      Serial.end();
    }
    else
    {
      Serial2.end();
    }
#else
    Serial.end();
#endif
  }
  else
  {
    SoftwareSerial::end();
  }
}
//----------------------------------------------------------------------------
size_t MK_Serial::write(uint8_t byte)
{
  if (_hard)
  {
#if defined(__AVR_ATmega32U4__)
    return (_scratch ? Serial.write(byte) : Serial1.write(byte) );
#elif defined(__AVR_ATmega2560__)
    if (getPort() == PORT_15)
    {
      return (Serial3.write(byte) );
    }
    else if (getPort() == PORT_16)
    {
      return (Serial.write(byte) );
    }
    else
    {
      return (Serial2.write(byte) );
    }
#else
    return (Serial.write(byte) );
#endif
  }
  else
  {
    return (SoftwareSerial::write(byte) );
  }
}
//----------------------------------------------------------------------------
int16_t MK_Serial::read(void)
{
  if (_polling)
  {
    int16_t temp = _byte;
    _byte = -1;
    return (temp > -1 ? temp : poll() );
  }
  if (_hard)
  {
#if defined(__AVR_ATmega32U4__)
    return (_scratch ? Serial.read() : Serial1.read() );
#elif defined(__AVR_ATmega2560__)
    if (getPort() == PORT_15)
    {
      return (Serial3.read() );
    }
    else if(getPort() == PORT_16)
    {
      return (Serial.read() );
    }
    else
    {
      return (Serial2.read() );
    }
#else
    return (Serial.read() );
#endif
  }
  else
  {
    return (SoftwareSerial::read() );
  }
}
//----------------------------------------------------------------------------
int16_t MK_Serial::available(void)
{
  if (_polling)
  {
    _byte = poll();
    return (_byte > -1 ? 1 : 0);
  }
  if (_hard)
  {
#if defined(__AVR_ATmega32U4__)
    return (_scratch ? Serial.available() : Serial1.available() );
#elif defined(__AVR_ATmega2560__)
    if (getPort() == PORT_15)
    {
      return (Serial3.available() );
    }
    else if(getPort() == PORT_16)
    {
      return (Serial.available() );
    }
    else
    {
      return (Serial2.available() );
    }
#else
    return (Serial.available() );
#endif
  }
  else
  {
    return (SoftwareSerial::available() );
  }
}
//----------------------------------------------------------------------------
bool MK_Serial::listen(void)
{
  if (_hard)
  {
    return (true);
  }
  else
  {
    return (SoftwareSerial::listen() );
  }
}
//----------------------------------------------------------------------------
bool MK_Serial::isListening(void)
{
  if (_hard)
  {
    return (true);
  }
  else
  {
    return (SoftwareSerial::isListening() );
  }
}
//----------------------------------------------------------------------------
int16_t MK_Serial::poll(void)
{
  int16_t val = 0;
  int16_t bitDelay = _bitPeriod - clockCyclesToMicroseconds(50);
  if (digitalRead(_RxPin) == LOW)
  {
    for (int16_t offset = 0; offset < 8; offset++)
    {
      delayMicroseconds(bitDelay);
      val |= digitalRead(_RxPin) << offset;
    }
    delayMicroseconds(bitDelay);
    return (val & 0xff);
  }
  return (-1);
}
//----------------------------------------------------------------------------
void MK_Serial::sendString(char *str)
{
  while(*str)
  {
    write(*str++);
  }
}
//----------------------------------------------------------------------------
void MK_Serial::printf(char *fmt,...)
{
  va_list ap;
  char string[128];
  va_start(ap,fmt);
  vsprintf(string,fmt,ap);
  sendString(string);
  va_end(ap);
}
//----------------------------------------------------------------------------
boolean MK_Serial::dataLineAvailable(void)
{
  if(available())
  {
    char c = read();
    if(c=='\n')
    {
      buffer[bufferIndex] = 0;
      return true;
    }
    else
    {
      buffer[bufferIndex]=c;
      bufferIndex++;
    }
  }
  return false;
}
//----------------------------------------------------------------------------
String MK_Serial::readDataLine(void)
{
  if(bufferIndex>0)
  {
    lastLine = buffer;
  }
  bufferIndex = 0;
  memset(buffer,0,64);
  return lastLine;
}
//----------------------------------------------------------------------------
float MK_Serial::getValue(String key)
{
  String s = readDataLine();
  if(stringLength(s)>2)
  {
    char * tmp;
    char * str;
    str = strtok_r((char*)s.c_str(), "=", &tmp);
    if(str!=NULL && strcmp(str,key.c_str())==0)
    {
      float v = atof(tmp);
      return v;
    }
  }
  return 0;
}
//----------------------------------------------------------------------------
boolean MK_Serial::equalString(String s1,String s2)
{
  return s1.equals(s2);
}
//----------------------------------------------------------------------------
String MK_Serial::concatenateWith(String s1,String s2)
{
  return s1+s2;
}
//----------------------------------------------------------------------------
char MK_Serial::letterOf(int i,String s)
{
  return s.charAt(i);
}
//----------------------------------------------------------------------------
int MK_Serial::stringLength(String s)
{
  return s.length();
}
//----------------------------------------------------------------------------

#endif
