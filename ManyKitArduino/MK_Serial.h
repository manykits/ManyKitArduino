/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * \class MK_Serial
 * \brief   Driver for serial.
 * @file    MK_Serial.h
 * @author  MakeBlock
 * @version V1.0.1
 * @date    2015/01/20
 * @brief   Header for for MK_Serial.cpp module
 *
 * \par Copyright
 * This software is Copyright (C), 2012-2016, MakeBlock. Use is subject to license \n
 * conditions. The main licensing options available are GPL V2 or Commercial: \n
 *
 * \par Open Source Licensing GPL V2
 * This is the appropriate option if you want to share the source code of your \n
 * application with everyone you distribute it to, and you also want to give them \n
 * the right to share who uses it. If you wish to use this software under Open \n
 * Source Licensing, you must contribute all your source code to the open source \n
 * community in accordance with the GPL Version 2 when your application is \n
 * distributed. See http://www.gnu.org/copyleft/gpl.html
 *
 * \par Description
 * This file is a drive for serial, It support hardware and software serial
 *
 * \par Method List:
 *
 *    1. void MK_Serial::setHardware(bool mode)
 *    2. void MK_Serial::begin(long baudrate)
 *    3. void MK_Serial::end(void)
 *    4. size_t MK_Serial::write(uint8_t byte)
 *    5. int16_t MK_Serial::read(void)
 *    6. int16_t MK_Serial::available(void)
 *    7. bool MK_Serial::listen(void)
 *    8. bool MK_Serial::isListening(void)
 *    9. int16_t MK_Serial::poll(void)
 *
 * \par History:
 * <pre>
 * `<Author>`         `<Time>`        `<Version>`        `<Descr>`
 * Mark Yan         2015/09/08     1.0.0            Rebuild the old lib.
 * Mark Yan         2016/01/20     1.0.1            support arduino pin-setting.
 * </pre>
 */
#ifndef MeSerial_H
#define MeSerial_H

#include "MK_ArduinoConfig.h"

#if defined MK_MP3

#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>
#include <SoftwareSerial.h>

class MK_Serial : public SoftwareSerial
{
public:
  MK_Serial(void);
  MK_Serial(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic = false);

  void setHardware(bool mode);

  void begin(long baudrate);

  size_t write(uint8_t byte);

  int read();

  int available();

  int16_t poll(void);

  void end(void);

  bool listen(void);

  bool isListening(void);

  void sendString(char *str);

  void printf(char *fmt,...);

  boolean dataLineAvailable(void);
  String readDataLine(void);
  String concatenateWith(String s1,String s2);
  char letterOf(int i,String s);
  int stringLength(String s);
  boolean equalString(String s1,String s2);
  float getValue(String key);

protected:
  int getPort();

  bool _hard;
  bool _polling;
  bool _scratch;
  int16_t _bitPeriod;
  int16_t _byte;
  long _lastTime;
  char buffer[64];
  String lastLine;
  int bufferIndex;

private:
  volatile uint8_t _RxPin;
  volatile uint8_t _TxPin; 
};

#endif

#endif
