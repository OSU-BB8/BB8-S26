// BB-8 ESP32-S2 Remote Controller
// Libraries: Adafruit GFX, Adafruit ST7735/ST7789, ArduinoJson v7

#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <ArduinoJson.h>

const char* WIFI_SSID = "PrivatePiNet";
const char* WIFI_PASSWORD = "password";
const char* COMMAND_TOKEN = "BB8_is_awesome";

const IPAddress BODY_IP(10, 42, 0, 27);
constexpr uint16_t BODY_PORT = 5100;
constexpr uint16_t LOCAL_PORT = 5101;

// TFT display
constexpr int TFT_CLK = 12;
constexpr int TFT_MOSI = 11;
constexpr int TFT_CS = 10;
constexpr int TFT_DC = 9;
constexpr int TFT_RST = 8;

// Encoder and emergency-stop input
constexpr int ENC_CLK = 4;
constexpr int ENC_DT = 5;
constexpr int ENC_SW = 6;
constexpr int ESTOP_PIN = 14;

constexpr unsigned long LONG_PRESS_MS = 900;
constexpr unsigned long STATUS_INTERVAL_MS = 800;
constexpr unsigned long STATUS_STALE_MS = 2500;
constexpr unsigned long HEARTBEAT_INTERVAL_MS = 500;

Adafruit_ST7735 tft(&SPI, TFT_CS, TFT_DC, TFT_RST);
WiFiUDP udp;

const int MODE_COUNT = 5;

const char* const modes[MODE_COUNT] = {
  "PS5",
  "APRILTAG",
  "BEACON",
  "DEMO",
  "STOP"
};

const char* const labels[MODE_COUNT] = {
  "PS5 Controller",
  "AprilTag Follow",
  "Beacon Follow",
  "Movement Demo",
  "Stop BB-8"
};

int selected = 0;

bool statusScreen = false;
bool estopLatched = false;
bool wifiWasConnected = false;
bool udpStarted = false;

String reportedMode = "UNKNOWN";
String reportedState = "UNKNOWN";
String lastAction = "READY";

bool reportedHead = false;
bool reportedRelay = false;
bool reportedAlive = false;

unsigned long lastStatusRx = 0;
unsigned long lastStatusRequest = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastWifiAttempt = 0;

unsigned long pressStarted = 0;
unsigned long lastEncoderEdge = 0;
unsigned long lastButtonTransition = 0;
unsigned long lastEstopTransition = 0;

int previousClk = HIGH;
int stableButton = HIGH;
int rawButton = HIGH;
int stableEstop = HIGH;
int rawEstop = HIGH;

bool redrawAll = true;
bool headerDirty = true;
bool detailsDirty = true;

int lastSelection = -1;

// ----------------------------------------------------
// Body Pi connection status
// ----------------------------------------------------

bool bodyOnline(unsigned long now) {
  return WiFi.status() == WL_CONNECTED &&
         lastStatusRx != 0 &&
         now - lastStatusRx < STATUS_STALE_MS;
}

// ----------------------------------------------------
// Send UDP command
// ----------------------------------------------------

void sendCommand(const char* command,
                 const char* mode = nullptr) {

  if (WiFi.status() != WL_CONNECTED || !udpStarted) {
    lastAction = "NO WIFI";
    detailsDirty = true;
    return;
  }

  JsonDocument doc;
  doc["token"] = COMMAND_TOKEN;
  doc["command"] = command;

  if (mode != nullptr) {
    doc["mode"] = mode;
  }

  String message;
  serializeJson(doc, message);

  if (!udp.beginPacket(BODY_IP, BODY_PORT)) {
    lastAction = "SEND ERROR";
    detailsDirty = true;
    return;
  }

  udp.write((const uint8_t*)message.c_str(),
            message.length());

  bool ok = udp.endPacket();

  if (strcmp(command, "HEARTBEAT") != 0 &&
      strcmp(command, "STATUS") != 0) {

    lastAction = ok ? String(command) +
                      (mode ? " " + String(mode) : "")
                    : "SEND ERROR";

    detailsDirty = true;

    Serial.printf(
      "Sent %s %s (%s)\n",
      command,
      mode ? mode : "",
      ok ? "UDP sent" : "failed"
    );
  }
}

// ----------------------------------------------------
// Receive actual status from Body Pi
// ----------------------------------------------------

void receiveStatus() {

  int n;

  while ((n = udp.parsePacket()) > 0) {

    IPAddress from = udp.remoteIP();

    if (n > 1024) {
      while (udp.available()) udp.read();
      continue;
    }

    char buffer[1025];

    int len = udp.read(
      (uint8_t*)buffer,
      min(n, 1024)
    );

    if (len <= 0 || from != BODY_IP) {
      continue;
    }

    buffer[len] = '\0';

    JsonDocument doc;

    if (deserializeJson(doc, buffer)) {
      continue;
    }

    if (String((const char*)(doc["type"] | "")) !=
        "STATUS_REPLY") {
      continue;
    }

    reportedState =
      String((const char*)(doc["state"] | "UNKNOWN"));

    const char* m = doc["mode"] | "";

    reportedMode =
      (m && *m) ? String(m) : "IDLE";

    reportedHead = doc["head_connected"] | false;
    reportedRelay = doc["relay_enabled"] | false;
    reportedAlive = doc["mode_process_alive"] | false;

    lastStatusRx = millis();

    headerDirty = true;
    detailsDirty = true;
  }
}

// ----------------------------------------------------
// Display helpers
// ----------------------------------------------------

void printFit(String s, int maxChars) {

  if ((int)s.length() > maxChars) {
    s = s.substring(0, maxChars);
  }

  tft.print(s);
}

// ----------------------------------------------------
// Top status bar
// ----------------------------------------------------

void drawHeader() {

  tft.fillRect(0, 0, 160, 23, ST77XX_BLACK);

  tft.setTextSize(1);
  tft.setTextWrap(false);

  bool wifi = WiFi.status() == WL_CONNECTED;

  tft.setCursor(3, 5);

  tft.setTextColor(
    wifi ? ST77XX_GREEN : ST77XX_RED
  );

  tft.print(wifi ? "WiFi:ON" : "WiFi:OFF");

  tft.setCursor(76, 5);

  bool online = bodyOnline(millis());

  tft.setTextColor(
    online ? ST77XX_CYAN : ST77XX_YELLOW
  );

  String modeText =
    online ? reportedMode : "OFFLINE";

  tft.print("BB8:");
  printFit(modeText, 9);

  tft.drawFastHLine(
    0, 20, 160, ST77XX_BLUE
  );

  headerDirty = false;
}

// ----------------------------------------------------
// Draw one menu item
// ----------------------------------------------------

void drawMenuRow(int i) {

  int y = 26 + i * 17;

  tft.fillRect(
    2, y - 1, 156, 16, ST77XX_BLACK
  );

  if (i == selected) {
    tft.fillRoundRect(
      2, y - 1, 156, 16, 3, ST77XX_BLUE
    );
  }

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);

  tft.setCursor(7, y + 2);

  tft.print(i == selected ? "> " : "  ");
  tft.print(labels[i]);
}

// ----------------------------------------------------
// Status screen
// ----------------------------------------------------

void drawStatusDetails() {

  tft.fillRect(
    0, 24, 160, 104, ST77XX_BLACK
  );

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);

  bool online = bodyOnline(millis());

  tft.setCursor(4, 28);
  tft.print("BODY: ");
  tft.print(online ? "ONLINE" : "OFFLINE");

  tft.setCursor(4, 42);
  tft.print("STATE: ");
  printFit(
    online ? reportedState : "UNKNOWN", 17
  );

  tft.setCursor(4, 56);
  tft.print("MODE: ");
  printFit(
    online ? reportedMode : "UNKNOWN", 17
  );

  tft.setCursor(4, 70);
  tft.print("HEAD: ");
  tft.print(
    online ? (reportedHead ? "OK" : "LOST") : "--"
  );

  tft.setCursor(4, 84);
  tft.print("RELAY: ");
  tft.print(
    online ? (reportedRelay ? "ON" : "OFF") : "--"
  );

  tft.setCursor(4, 98);
  tft.print("PROCESS: ");
  tft.print(
    online ? (reportedAlive ? "RUNNING" : "STOPPED")
           : "--"
  );

  tft.setCursor(4, 113);

  tft.setTextColor(
    estopLatched ? ST77XX_RED : ST77XX_YELLOW
  );

  tft.print(
    estopLatched ? "STOP LATCHED" : "Hold dial: Menu"
  );

  detailsDirty = false;
}

// ----------------------------------------------------
// Menu footer
// ----------------------------------------------------

void drawMenuFooter() {

  tft.fillRect(
    0, 113, 160, 15, ST77XX_BLACK
  );

  tft.setTextSize(1);
  tft.setCursor(3, 116);

  tft.setTextColor(
    estopLatched ? ST77XX_RED : ST77XX_YELLOW
  );

  tft.print(
    estopLatched ? "STOP LATCHED"
                 : "Press:Go Hold:Status"
  );

  detailsDirty = false;
}

// ----------------------------------------------------
// Refresh display without constant full redraws
// ----------------------------------------------------

void draw() {

  if (redrawAll) {

    tft.fillScreen(ST77XX_BLACK);
    headerDirty = true;

    if (statusScreen) {
      drawStatusDetails();
    } else {

      for (int i = 0; i < MODE_COUNT; i++) {
        drawMenuRow(i);
      }

      drawMenuFooter();
      lastSelection = selected;
    }

    redrawAll = false;

  } else if (!statusScreen) {

    if (lastSelection != selected) {

      if (lastSelection >= 0) {
        drawMenuRow(lastSelection);
      }

      drawMenuRow(selected);
      lastSelection = selected;
    }

    if (detailsDirty) {
      drawMenuFooter();
    }

  } else if (detailsDirty) {
    drawStatusDetails();
  }

  if (headerDirty) {
    drawHeader();
  }
}

// ----------------------------------------------------
// Short encoder press
// ----------------------------------------------------

void shortPress() {

  if (statusScreen) return;

  if (estopLatched) {
    lastAction = "STOP LATCHED";
    detailsDirty = true;
    return;
  }

  // Fifth menu option: Stop BB-8 and return to IDLE.
  if (selected == 4) {
    sendCommand("STOP");
  } else {
    sendCommand("START", modes[selected]);
  }

  // Only the Body Pi's reply changes reportedMode.
}

// ----------------------------------------------------
// Wi-Fi events
// ----------------------------------------------------

void onWiFiEvent(
  WiFiEvent_t event,
  WiFiEventInfo_t info
) {

  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    Serial.printf(
      "WiFi disconnected (%d)\n",
      info.wifi_sta_disconnected.reason
    );
  }
}

// ----------------------------------------------------
// Setup
// ----------------------------------------------------

void setup() {

  Serial.begin(115200);
  delay(400);

  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  pinMode(ESTOP_PIN, INPUT_PULLUP);

  previousClk = digitalRead(ENC_CLK);

  stableButton = rawButton =
    digitalRead(ENC_SW);

  stableEstop = rawEstop =
    digitalRead(ESTOP_PIN);

  SPI.begin(
    TFT_CLK, -1, TFT_MOSI, TFT_CS
  );

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.setTextWrap(false);

  draw();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);

  WiFi.onEvent(onWiFiEvent);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  lastWifiAttempt = millis();
}

// ----------------------------------------------------
// Main loop
// ----------------------------------------------------

void loop() {

  unsigned long now = millis();

  bool wifi = WiFi.status() == WL_CONNECTED;

  if (wifi != wifiWasConnected) {

    wifiWasConnected = wifi;
    headerDirty = true;
    detailsDirty = true;

    if (wifi) {

      udpStarted = udp.begin(LOCAL_PORT);

      lastStatusRx = 0;
      lastStatusRequest = 0;
      lastHeartbeat = 0;

      Serial.println(
        "WiFi connected: " +
        WiFi.localIP().toString()
      );

    } else {

      udp.stop();
      udpStarted = false;
      lastStatusRx = 0;
    }
  }

  if (!wifi &&
      now - lastWifiAttempt > 15000) {

    WiFi.reconnect();
    lastWifiAttempt = now;
  }

  if (wifi && udpStarted) {

    receiveStatus();

    if (now - lastHeartbeat >=
        HEARTBEAT_INTERVAL_MS) {

      sendCommand("HEARTBEAT");
      lastHeartbeat = now;
    }

    if (now - lastStatusRequest >=
        STATUS_INTERVAL_MS) {

      sendCommand("STATUS");
      lastStatusRequest = now;
    }
  }

  // Emergency-stop input.
  // Preserves the previous STOP + local latch behavior.
  // This is NOT the supervisor's latched ESTOP command.
  int er = digitalRead(ESTOP_PIN);

  if (er != rawEstop) {
    rawEstop = er;
    lastEstopTransition = now;
  }

  if (now - lastEstopTransition >= 35 &&
      er != stableEstop) {

    stableEstop = er;

    if (er == LOW) {

      estopLatched = true;
      sendCommand("STOP");
      detailsDirty = true;
    }
  }

  // Rotary encoder
  int clk = digitalRead(ENC_CLK);

  if (clk != previousClk) {

    if (clk == LOW &&
        now - lastEncoderEdge > 3) {

      int dt = digitalRead(ENC_DT);

      selected = (
        selected + (dt != clk ? 1 : MODE_COUNT - 1)
      ) % MODE_COUNT;

      lastEncoderEdge = now;
    }

    previousClk = clk;
  }

  // Encoder pushbutton
  int sw = digitalRead(ENC_SW);

  if (sw != rawButton) {
    rawButton = sw;
    lastButtonTransition = now;
  }

  if (now - lastButtonTransition >= 35 &&
      sw != stableButton) {

    stableButton = sw;

    if (sw == LOW) {

      pressStarted = now;

    } else if (pressStarted) {

      unsigned long held = now - pressStarted;
      pressStarted = 0;

      if (held >= LONG_PRESS_MS) {

        statusScreen = !statusScreen;
        redrawAll = true;

      } else {

        shortPress();
      }
    }
  }

  static bool lastOnline = false;

  bool online = bodyOnline(now);

  if (online != lastOnline) {

    lastOnline = online;
    headerDirty = true;
    detailsDirty = true;
  }

  if (redrawAll || headerDirty || detailsDirty ||
      (!statusScreen && lastSelection != selected)) {

    draw();
  }

  delay(2);
}
