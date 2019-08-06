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
#define IP0 6
#define IP1 66
#define url "YaoQuanAIOT"
#define password "12345678"
#endif

const char* host = "manykit_esp_serial";

WiFiUDP Udp;

String serialReceiveStr;

String receiveStr;
String sendStr;

unsigned int localUdpPort = 2334; // local udp port
char incomingPacket[537];         // udp receive buffer

Timer mTimer;
int mSendEventID = 0;

IPAddress remoteIP;
uint16_t remotePort = 0; 

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
    while(1) 
    { 
      delay(1000);
    }
  }
  Serial.println("mk mDNS_responder_started");

  server.on ("/version", []() {
     server.send(200, "text/plain", "1.0.0");
  });
  
  server.on ("/receive", []() {
     server.send(200, "text/plain", receiveStr);
  });

  server.on ("/send", []() {
     server.send(200, "text/plain", sendStr);
  });
  
  server.onNotFound([](){
    if(!_HandleFileRead(server.uri()))
      server.send(404, "text/plain", "FileNotFound");
  });
  
  server.begin();
  Serial.println("mk http_server_started");
  String ipStr = ip.toString();
  Serial.println("mk ip:" + ipStr);
  MDNS.setInstanceName("manykitespserial:"+ipStr);
  MDNS.addService("http", "tcp", 80);
 
  Udp.begin(localUdpPort);

  isInited = true;
}

void setup() { 
  Serial.begin(9600);

 #if defined SMART_CONFIG
  if (!autoConfig())
  {
    Serial.println("mk start_module");
    smartConfig();
  }
 #else
  WiFi.mode(WIFI_STA);
  IPAddress ip(192, 168, IP0, IP1); 
  IPAddress gateway(192, 168, IP0, 1);
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

void loop() {
  mTimer.update();
  
  if (!isInited)
    return;

  while (Serial.available()){
    char c = Serial.read();
    if ('\n' == c)
    {
      if (serialReceiveStr.length() > 0)
      {
        sendStr = serialReceiveStr;
                
        Udp.beginPacket(remoteIP, remotePort);
        Udp.write(serialReceiveStr.c_str(), serialReceiveStr.length()); 
        Udp.endPacket();
      }
      serialReceiveStr = "";
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
    remoteIP =  Udp.remoteIP();
    remotePort = Udp.remotePort();

    int len = Udp.read(incomingPacket, 536);
    if (len > 0)
    {
      incomingPacket[len] = 0;
      
      receiveStr = String(incomingPacket);
      
      Serial.write(incomingPacket, len);
    }
  }
}