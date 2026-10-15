// BB-8 ESP32-S2 remote: live Body Pi status, four modes, hold encoder for status.
// Libraries: Adafruit GFX, Adafruit ST7735/ST7789; Arduino ESP32 core.
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <ArduinoJson.h>

const char* WIFI_SSID = "PrivatePiNet";
const char* WIFI_PASSWORD = "Password";
const char* COMMAND_TOKEN = "BB8_is_awesome"; // Change in BOTH ESP32 and Pi config.
const IPAddress BODY_IP(10,42,0,27);
constexpr uint16_t BODY_PORT=5100, LOCAL_PORT=5101;
constexpr int TFT_CLK=12,TFT_MOSI=11,TFT_CS=10,TFT_DC=9,TFT_RST=8,TFT_BL=7;
constexpr int ENC_CLK=4,ENC_DT=5,ENC_SW=6,ESTOP_PIN=14;
constexpr unsigned long LONG_PRESS_MS=900, STATUS_INTERVAL_MS=800;
constexpr unsigned long STATUS_STALE_MS=2500, HEARTBEAT_INTERVAL_MS=500;

Adafruit_ST7735 tft(&SPI,TFT_CS,TFT_DC,TFT_RST);
WiFiUDP udp;
const char* const modes[]={"PS5","APRILTAG","BEACON","DEMO"};
const char* const labels[]={"PS5 Controller","AprilTag Follow","Beacon Follow","Movement Demo"};
int selected=0;
bool statusScreen=false,estopLatched=false, wifiWasConnected=false,udpStarted=false;
String reportedMode="UNKNOWN",reportedState="UNKNOWN",lastAction="READY";
bool reportedHead=false,reportedRelay=false,reportedAlive=false;
unsigned long lastStatusRx=0,lastStatusRequest=0,lastHeartbeat=0,lastWifiAttempt=0;
unsigned long pressStarted=0,lastEncoderEdge=0,lastButtonTransition=0,lastEstopTransition=0;
int previousClk=HIGH,stableButton=HIGH,rawButton=HIGH,stableEstop=HIGH,rawEstop=HIGH;
bool redrawAll=true,headerDirty=true,detailsDirty=true;
int lastSelection=-1;

bool bodyOnline(unsigned long now){return WiFi.status()==WL_CONNECTED && lastStatusRx!=0 && now-lastStatusRx<STATUS_STALE_MS;}
void sendCommand(const char* command,const char* mode=nullptr){
  if(WiFi.status()!=WL_CONNECTED || !udpStarted){lastAction="NO WIFI";detailsDirty=true;return;}
  String json="{\"token\":\""+String(COMMAND_TOKEN)+"\",\"command\":\""+String(command)+"\"";
  if(mode) json+=",\"mode\":\""+String(mode)+"\"";
  json+="}";
  if(!udp.beginPacket(BODY_IP,BODY_PORT)){lastAction="SEND ERROR";detailsDirty=true;return;}
  udp.write((const uint8_t*)json.c_str(),json.length());
  bool ok=udp.endPacket();
  if(String(command)!="HEARTBEAT" && String(command)!="STATUS"){
    lastAction=ok ? String(command)+(mode?" "+String(mode):"") : "SEND ERROR";
    detailsDirty=true;
    Serial.printf("Sent %s %s (%s)\n",command,mode?mode:"",ok?"UDP sent":"failed");
  }
}
void receiveStatus(){
  int n;
  while((n=udp.parsePacket())>0){
    IPAddress from=udp.remoteIP();
    if(n>1024){while(udp.available())udp.read();continue;}
    char buffer[1025];int len=udp.read((uint8_t*)buffer,min(n,1024));
    if(len<=0 || from!=BODY_IP)continue;
    buffer[len]='\0';
    JsonDocument doc;
    if(deserializeJson(doc,buffer))continue;
    if(String((const char*)(doc["type"]|""))!="STATUS_REPLY")continue;
    reportedState=String((const char*)(doc["state"]|"UNKNOWN"));
    const char* m=doc["mode"] | "";
    reportedMode=(m && *m)?String(m):"IDLE";
    reportedHead=doc["head_connected"]|false;
    reportedRelay=doc["relay_enabled"]|false;
    reportedAlive=doc["mode_process_alive"]|false;
    lastStatusRx=millis();
    headerDirty=true;detailsDirty=true;
  }
}
void printFit(String s,int maxChars){if((int)s.length()>maxChars)s=s.substring(0,maxChars);tft.print(s);}
void drawHeader(){
  tft.fillRect(0,0,160,23,ST77XX_BLACK);
  tft.setTextSize(1);tft.setTextWrap(false);
  bool wifi=WiFi.status()==WL_CONNECTED;
  tft.setCursor(3,5);tft.setTextColor(wifi?ST77XX_GREEN:ST77XX_RED);
  tft.print(wifi?"WiFi:ON":"WiFi:OFF");
  tft.setCursor(76,5);
  bool online=bodyOnline(millis());
  tft.setTextColor(online?ST77XX_CYAN:ST77XX_YELLOW);
  String text=online?reportedMode:"OFFLINE";
  tft.print("BB8:");printFit(text,9);
  tft.drawFastHLine(0,20,160,ST77XX_BLUE);
  headerDirty=false;
}
void drawMenuRow(int i){
  int y=30+i*21;
  tft.fillRect(2,y-2,156,19,ST77XX_BLACK);
  if(i==selected)tft.fillRoundRect(2,y-2,156,19,3,ST77XX_BLUE);
  tft.setTextSize(1);tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(7,y+3);tft.print(i==selected?"> ":"  ");tft.print(labels[i]);
}
void drawStatusDetails(){
  tft.fillRect(0,24,160,104,ST77XX_BLACK);
  tft.setTextSize(1);tft.setTextColor(ST77XX_WHITE);
  bool online=bodyOnline(millis());
  tft.setCursor(4,28);tft.print("BODY: ");tft.print(online?"ONLINE":"OFFLINE");
  tft.setCursor(4,42);tft.print("STATE: ");printFit(online?reportedState:"UNKNOWN",17);
  tft.setCursor(4,56);tft.print("MODE: ");printFit(online?reportedMode:"UNKNOWN",17);
  tft.setCursor(4,70);tft.print("HEAD: ");tft.print(online?(reportedHead?"OK":"LOST"):"--");
  tft.setCursor(4,84);tft.print("RELAY: ");tft.print(online?(reportedRelay?"ON":"OFF"):"--");
  tft.setCursor(4,98);tft.print("PROCESS: ");tft.print(online?(reportedAlive?"RUNNING":"STOPPED"):"--");
  tft.setCursor(4,113);tft.setTextColor(estopLatched?ST77XX_RED:ST77XX_YELLOW);
  tft.print(estopLatched?"STOP LATCHED":"Hold dial: Menu");
  detailsDirty=false;
}
void drawMenuFooter(){
  tft.fillRect(0,113,160,15,ST77XX_BLACK);
  tft.setTextSize(1);tft.setCursor(3,116);
  tft.setTextColor(estopLatched?ST77XX_RED:ST77XX_YELLOW);
  tft.print(estopLatched?"STOP LATCHED":"Press:Go Hold:Status");
  detailsDirty=false;
}
void draw(){
  if(redrawAll){
    tft.fillScreen(ST77XX_BLACK);headerDirty=true;
    if(statusScreen)drawStatusDetails();
    else {for(int i=0;i<4;i++)drawMenuRow(i);drawMenuFooter();lastSelection=selected;}
    redrawAll=false;
  }else if(!statusScreen){
    if(lastSelection!=selected){if(lastSelection>=0)drawMenuRow(lastSelection);drawMenuRow(selected);lastSelection=selected;}
    if(detailsDirty)drawMenuFooter();
  }else if(detailsDirty)drawStatusDetails();
  if(headerDirty)drawHeader();
}
void shortPress(){
  if(statusScreen)return;
  if(estopLatched){lastAction="STOP LATCHED";detailsDirty=true;return;}
  sendCommand("START",modes[selected]);
  // Do NOT change reportedMode here: only a Body Pi STATUS_REPLY may do so.
}
void onWiFiEvent(WiFiEvent_t event,WiFiEventInfo_t info){
  if(event==ARDUINO_EVENT_WIFI_STA_DISCONNECTED)Serial.printf("WiFi disconnected (%d)\n",info.wifi_sta_disconnected.reason);
}
void setup(){
  Serial.begin(115200);delay(400);
  pinMode(TFT_BL,OUTPUT);digitalWrite(TFT_BL,HIGH);
  pinMode(ENC_CLK,INPUT_PULLUP);pinMode(ENC_DT,INPUT_PULLUP);
  pinMode(ENC_SW,INPUT_PULLUP);pinMode(ESTOP_PIN,INPUT_PULLUP);
  previousClk=digitalRead(ENC_CLK);stableButton=rawButton=digitalRead(ENC_SW);
  stableEstop=rawEstop=digitalRead(ESTOP_PIN);
  SPI.begin(TFT_CLK,-1,TFT_MOSI,TFT_CS);
  tft.initR(INITR_BLACKTAB);tft.setRotation(1);tft.setTextWrap(false);
  draw();
  WiFi.mode(WIFI_STA);WiFi.setSleep(false);WiFi.setAutoReconnect(true);
  WiFi.onEvent(onWiFiEvent);WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
  lastWifiAttempt=millis();
}
void loop(){
  unsigned long now=millis();
  bool wifi=WiFi.status()==WL_CONNECTED;
  if(wifi!=wifiWasConnected){
    wifiWasConnected=wifi;headerDirty=true;detailsDirty=true;
    if(wifi){udpStarted=udp.begin(LOCAL_PORT);lastStatusRx=0;lastStatusRequest=0;lastHeartbeat=0;Serial.println("WiFi connected: "+WiFi.localIP().toString());}
    else {udp.stop();udpStarted=false;lastStatusRx=0;}
  }
  if(!wifi && now-lastWifiAttempt>15000){WiFi.reconnect();lastWifiAttempt=now;}
  if(wifi && udpStarted){
    receiveStatus();
    if(now-lastHeartbeat>=HEARTBEAT_INTERVAL_MS){sendCommand("HEARTBEAT");lastHeartbeat=now;}
    if(now-lastStatusRequest>=STATUS_INTERVAL_MS){sendCommand("STATUS");lastStatusRequest=now;}
  }
  // E-stop button: retain existing STOP command and local restart latch.
  int er=digitalRead(ESTOP_PIN);
  if(er!=rawEstop){rawEstop=er;lastEstopTransition=now;}
  if(now-lastEstopTransition>=35 && er!=stableEstop){
    stableEstop=er;
    if(er==LOW){estopLatched=true;sendCommand("STOP");detailsDirty=true;}
  }
  int clk=digitalRead(ENC_CLK);
  if(clk!=previousClk){
    if(clk==LOW && now-lastEncoderEdge>3){
      int dt=digitalRead(ENC_DT);
      selected=(selected+(dt!=clk?1:3))%4;
      lastEncoderEdge=now;
    }
    previousClk=clk;
  }
  int sw=digitalRead(ENC_SW);
  if(sw!=rawButton){rawButton=sw;lastButtonTransition=now;}
  if(now-lastButtonTransition>=35 && sw!=stableButton){
    stableButton=sw;
    if(sw==LOW)pressStarted=now;
    else if(pressStarted){
      unsigned long held=now-pressStarted;pressStarted=0;
      if(held>=LONG_PRESS_MS){statusScreen=!statusScreen;redrawAll=true;}
      else shortPress();
    }
  }
  static bool lastOnline=false;
  bool online=bodyOnline(now);
  if(online!=lastOnline){lastOnline=online;headerDirty=true;detailsDirty=true;}
  if(redrawAll||headerDirty||detailsDirty||(!statusScreen&&lastSelection!=selected))draw();
  delay(2);
}
