// MK_ArduinoConfig.h

#ifndef MK_ARDUINOCONFIG_H
#define MK_ARDUINOCONFIG_H

//#define MK_ESP_NETWORK

#if defined MK_ESP_NETWORK 
#define MK_RCSWITCH
#else
#define MK_SERVO
#define MK_MOTO
#define MK_PID
#define MK_IR
#define MK_STEPMOTO
#define MK_RCSWITCH
#define MK_DHT
#define MK_SCREEN_I2C
#define MK_RGBLED
#define MK_SEGMENT
//#define MK_LEDMATRIX
//#define MK_MP3
// --------------- not use down
//#define MK_DFMP3
// weight
//#define MK_XH711 1
//#define PX2_POLOLUBUZZER 1
//#define MK_AXIS 1
#endif

#endif
