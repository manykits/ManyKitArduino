// ManyKitESPSerial.ino

#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <FS.h>
#include "MKTimer.h"

//#define SMART_CONFIG 1

#if not defined SMART_CONFIG

//! change to your ip you wanted
IPAddress ip(192, 168, 6, 123);
IPAddress gateway(192, 168, 6, 1);

//! router to connect
#define url "YaoQuanAIOT"
#define password "12345678"

#endif

//! set this to make sure if need to send to the cloud
bool isSendToAServer = true;
IPAddress ipToSendToAServer(182, 254, 213, 85);
uint16_t portToSendToAServer = 2336;
// this's tag means me
String tagHelloItIsMe("hello i am esp");

const char* host = "manykit_esp_serial";
WiFiUDP Udp;

String serialReceiveStr;
String udpReceiveStr;
String udpSendStr;

unsigned int localUdpPort = 2334; // local udp port
char incomingPacket[537];         // udp receive buffer

Timer mTimer;
int mSendEventID = 0;

struct RMOBJ {
  IPAddress remoteIP;
  uint16_t remotePort;
};
#define NumMaxRMOBJ 6
RMOBJ rmobjs[NumMaxRMOBJ];
int numrmObjs = 0;

bool isHasRMObj(IPAddress adr, uint16_t port){
    int i=0;
    for (; i<numrmObjs; i++){
      if (rmobjs[i].remoteIP == adr && rmobjs[i].remotePort == port) {
        return true;  
      }
    }

    return false;
}

ESP8266WebServer server(80);

String _GetContentType(String filename)
{
  if (server.hasArg("download"))
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
  if (path.endsWith("/"))
    path += "index.htm";
  String contentType = _GetContentType(path);
  if (SPIFFS.exists(path))
  {
    File file = SPIFFS.open(path, "r");
    size_t sent = server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

bool autoConfig()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin();
  for (int i = 0; i < 20; i++)
  {
    pinMode(0, INPUT);
    int val = digitalRead(0);
    if (0 == val)
    {
      Serial.println("mk eraseconfig_begin");
      WiFi.disconnect();
      delay(2000);
      ESP.restart();
      Serial.println("mk eraseconfig_done");
    }
    int wstatus = WiFi.status();
    if (wstatus == WL_CONNECTED)
    {
      Serial.println("mk autoconfig_success");
      Serial.printf("mk ssid:%s\r\n", WiFi.SSID().c_str());
      Serial.printf("mk psw:%s\r\n", WiFi.psk().c_str());
      WiFi.printDiag(Serial);
      return true;
    }
    else
    {
      Serial.print("mk autoconfig_waiting");
      Serial.println(wstatus);
      delay(500);
    }
  }
  Serial.println("mk autoconfig_faild");
  return false;
}

void smartConfig()
{
  Serial.println("mk wait_for_smartconfig");
  WiFi.beginSmartConfig();
  bool isSmartDone = WiFi.smartConfigDone();
  while (!isSmartDone)
  {
    Serial.print(".");
    isSmartDone = WiFi.smartConfigDone();

    if (isSmartDone)
    {
      Serial.println("mk smartconfig_success!");
      Serial.printf("ssid:%s\r\n", WiFi.SSID().c_str());
      Serial.printf("psw:%s\r\n", WiFi.psk().c_str());
      WiFi.setAutoConnect(true);
    }
    delay(1500);
  }
}

char string[32];
bool isInited = false;

void timerAfter()
{
  IPAddress ip = WiFi.localIP();
  IPAddress ipzero;
  ipzero.fromString("0.0.0.0");
  while (ip == ipzero)
  {
    ip = WiFi.localIP();
    Serial.print("mk ip_address:");
    Serial.println(ip);
    delay(500);
  }

  // file system
  SPIFFS.begin();

  if (!MDNS.begin(host, ip))
  {
    Serial.println("mk error_setting_up_MDNS_responder");
    while (1)
    {
      delay(1000);
    }
  }
  Serial.println("mk mDNS_responder_started");

  server.on ("/version", []() {
    server.send(200, "text/plain", "1.0.0");
  });

  server.on ("/receive", []() {
    server.send(200, "text/plain", udpReceiveStr);
  });

  server.on ("/send", []() {
    server.send(200, "text/plain", udpSendStr);
  });

  server.onNotFound([]() {
    if (!_HandleFileRead(server.uri()))
      server.send(404, "text/plain", "FileNotFound");
  });

  server.begin();
  Serial.println("mk http_server_started");
  String ipStr = ip.toString();
  Serial.println("mk ip:" + ipStr);
  MDNS.setInstanceName("manykitespserial:" + ipStr);
  MDNS.addService("http", "tcp", 80);

  Udp.begin(localUdpPort);

  if (isSendToAServer){
     Udp.beginPacket(ipToSendToAServer, portToSendToAServer);
     Udp.write(tagHelloItIsMe.c_str(), tagHelloItIsMe.length());
     Udp.endPacket();
  }

  isInited = true;
}

void setup() {
  Serial.begin(9600);
  numrmObjs = 0;

#if defined SMART_CONFIG
  if (!autoConfig())
  {
    Serial.println("mk start_module");
    smartConfig();
  }
#else
  WiFi.mode(WIFI_STA);
  Serial.print("mk setting_static_ip_to:");
  Serial.println(ip);
  IPAddress subnet(255, 255, 255, 0); // set subnet mask to match your network
  WiFi.config(ip, gateway, subnet);
  WiFi.begin(url, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
#endif

  isInited = false;
  mTimer.after(1000, timerAfter);
}

#define NumCMDParams 7
String CmdParams[NumCMDParams];

void loop() {
  mTimer.update();

  if (!isInited)
    return;

  while (Serial.available()) {
    char c = Serial.read();
    if ('\n' == c)
    {
      serialReceiveStr += '\n';
      if (serialReceiveStr.length() > 0)
      {
        udpSendStr = serialReceiveStr;
        String strs = serialReceiveStr;

        int i=0;
        for (i; i<NumCMDParams; i++)
        {
          CmdParams[i] = "";
        }
        int cmdIndexTemp = 0;
        char *pCMDParam = strtok((char *)strs.c_str(), " ");
        while (pCMDParam)
        {
          CmdParams[cmdIndexTemp] = String(pCMDParam);
          cmdIndexTemp++;
          pCMDParam = strtok(NULL, " ");
      
          if (cmdIndexTemp > NumCMDParams)
            break;
        }
        bool isSys = false;
        if (cmdIndexTemp > 0)
        {
          String cmd = CmdParams[0];
          if (String("sys") == cmd){ 
            isSys = true;
          }
        }

        if (isSys)
        {
          String doType = CmdParams[1];
          if (String("setip") == doType){
              String ip = CmdParams[2];
              String portStr = CmdParams[3];
              uint16_t port = atoi(portStr.c_str());
              ipToSendToAServer = IPAddress((const uint8_t *)ip.c_str());
              portToSendToAServer = port;
          }
        }
        else
        {
          if (isSendToAServer){
             Udp.beginPacket(ipToSendToAServer, portToSendToAServer);
             Udp.write(serialReceiveStr.c_str(), serialReceiveStr.length());
             Udp.endPacket();
           }

          int iRMOBJ = 0;
          bool isNeedSend = false;
          for (iRMOBJ=0; iRMOBJ<numrmObjs; iRMOBJ++){
             RMOBJ obj = rmobjs[iRMOBJ];
             // to cloud only need send once
             isNeedSend = true;     
             if (isSendToAServer && obj.remoteIP == ipToSendToAServer && obj.remotePort == portToSendToAServer){
              isNeedSend = false;
             }
             if (isNeedSend)
             {
               Udp.beginPacket(obj.remoteIP, obj.remotePort);
               Udp.write(serialReceiveStr.c_str(), serialReceiveStr.length());
               Udp.endPacket();
             }
          } 
        }
      }
      serialReceiveStr = "";
    }
    else if ('\r' == c)
    {
      /*_*/
    }
    else
    {
      serialReceiveStr += c;
    }
  }

  server.handleClient();

  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    RMOBJ obj;
    obj.remoteIP =  Udp.remoteIP();
    obj.remotePort = Udp.remotePort();
    if (!isHasRMObj( obj.remoteIP, obj.remotePort) &&
        numrmObjs<NumMaxRMOBJ-1){
       rmobjs[numrmObjs] = obj;
       numrmObjs++; 
    }

    int len = Udp.read(incomingPacket, 536);
    if (len > 0)
    {
      incomingPacket[len] = 0;

      udpReceiveStr = String(incomingPacket);
      String strs = udpReceiveStr;    
      
      int i=0;
      for (i; i<NumCMDParams; i++)
      {
        CmdParams[i] = "";
      }
      int cmdIndexTemp = 0;
      char *pCMDParam = strtok((char *)strs.c_str(), " ");
      while (pCMDParam)
      {
        CmdParams[cmdIndexTemp] = String(pCMDParam);
        cmdIndexTemp++;
        pCMDParam = strtok(NULL, " ");
    
        if (cmdIndexTemp > NumCMDParams)
          break;
      }
      bool isSys = false;
      if (cmdIndexTemp > 0)
      {
        String cmd = CmdParams[0];
        if (String("sys") == cmd){ 
          isSys = true;
        }
      }

      if (isSys){
        String doType = CmdParams[1];
        if (String("send") == doType){
            String ip = CmdParams[2];
            String portStr = CmdParams[3];
            String valStr =  CmdParams[4];
            uint16_t port = atoi(portStr.c_str());
           
            Udp.beginPacket((const uint8_t *)ip.c_str(), port);
            Udp.write(valStr.c_str(), valStr.length());
            Udp.endPacket();
        }
      }
      else{
          Serial.write(udpReceiveStr.c_str(), len);
      }
    }
  }
}
