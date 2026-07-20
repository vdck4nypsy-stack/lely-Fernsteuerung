/*
  Lely Touchscreen Fernsteuerung – ESP32-S3 "ABSOLUT + DRAG" (Bart-Methode erweitert)
  VERSION: v1.0  (funktionierende Basis + entschleunigt: delay 7ms, Homing 35x)
  ROBOTER 1  (VNC 10.4.1.xxx, Auflösung 1024x768)
  ==============================================================
  Kann jetzt zwei Dinge:
  1) KLICKEN an absoluter Position: Homing in Ecke -> 127er-Schritte zum Ziel -> Klick
  2) ZIEHEN/WISCHEN (zum Scrollen): an Startposition fahren, Taste HALTEN,
     relativ mitziehen (folgt dem Finger), am Ende loslassen.

  Wichtig in Arduino IDE:
    Board "ESP32S3 Dev Module", USB Mode "USB-OTG (TinyUSB)", USB CDC On Boot "Enabled"
  Bibliotheken: WebSockets (Markus Sattler), ArduinoJson (Benoit Blanchon v6)

  Protokoll (Web -> ESP32, Port 81):
    {"t":"click","px":512,"py":384}       -> Klick an Pixelposition
    {"t":"dragstart","px":500,"py":600}   -> an Position fahren, Taste drücken (Scroll-Beginn)
    {"t":"dragmove","dx":0,"dy":-40}      -> relativ weiterziehen (Taste bleibt gedrückt)
    {"t":"dragend"}                       -> Taste loslassen
    {"t":"wake"}                          -> Cursor sichtbar machen
*/

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "USB.h"
#include "USBHID.h"

// ---------- WLAN ----------
const char* WIFI_SSID     = "DEIN-WLAN-NAME";        // <<< HIER EINTRAGEN
const char* WIFI_PASSWORD = "DEIN-WLAN-PASSWORT";    // <<< HIER EINTRAGEN
IPAddress local_IP(192, 168, 178, 201);   // <<< feste IP fuer dieses Board -- MUSS ausserhalb des DHCP-Bereichs deines Routers liegen!
IPAddress gateway(192, 168, 178, 1);      // <<< IP deines Routers
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 178, 1);          // <<< meist gleich wie Router

const int SCREEN_W = 1024;
const int SCREEN_H = 768;

WebSocketsServer webSocket = WebSocketsServer(81);

static const uint8_t hid_descriptor[] = {
  0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x09, 0x01, 0xA1, 0x00,
  0x05, 0x09, 0x19, 0x01, 0x29, 0x03, 0x15, 0x00, 0x25, 0x01,
  0x95, 0x03, 0x75, 0x01, 0x81, 0x02, 0x95, 0x01, 0x75, 0x05, 0x81, 0x03,
  0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x15, 0x81, 0x25, 0x7F,
  0x75, 0x08, 0x95, 0x02, 0x81, 0x06,
  0xC0, 0xC0
};

class RelMouse : public USBHIDDevice {
public:
  RelMouse() { hid.addDevice(this, sizeof(hid_descriptor)); }
  void begin() { hid.begin(); }
  uint16_t _onGetDescriptor(uint8_t* buffer) override {
    memcpy(buffer, hid_descriptor, sizeof(hid_descriptor));
    return sizeof(hid_descriptor);
  }
  void report(int8_t dx, int8_t dy, uint8_t btn) {
    uint8_t r[3] = { (uint8_t)(btn & 0x07), (uint8_t)dx, (uint8_t)dy };
    hid.SendReport(0, r, sizeof(r));
    delay(7);
  }
  void moveBy(int dx, int dy, uint8_t btn) {
    while (dx != 0 || dy != 0) {
      int sx = dx; if (sx > 127) sx = 127; if (sx < -127) sx = -127;
      int sy = dy; if (sy > 127) sy = 127; if (sy < -127) sy = -127;
      report(sx, sy, btn);
      dx -= sx; dy -= sy;
    }
  }
  void home() {
    // Sicher in die obere linke Ecke fahren. Mehr Schritte als noetig + kleine Pausen,
    // damit der Roboter keine Bewegung "verschluckt" und die Ecke garantiert erreicht wird.
    for (int i = 0; i < 35; i++) { report(-127, -127, 0); delay(8); }
  }
  void wake() {
    for (int i = 0; i < 4; i++) { report(8, 0, 0); delay(15); }
    for (int i = 0; i < 4; i++) { report(-8, 0, 0); delay(15); }
  }
  uint8_t btnState = 0;
private:
  USBHID hid;
};

RelMouse mouse;

void clampPx(int &px, int &py) {
  if (px < 0) px = 0; if (px > SCREEN_W) px = SCREEN_W;
  if (py < 0) py = 0; if (py > SCREEN_H) py = SCREEN_H;
}

// Einfacher Klick an Position
void clickAt(int px, int py) {
  clampPx(px, py);
  mouse.home();
  mouse.moveBy(px, py, 0);
  delay(20);
  mouse.report(0, 0, 1);
  delay(40);
  mouse.report(0, 0, 0);
}

// Ziehen beginnen: an Position fahren, Taste drücken
void dragStart(int px, int py) {
  clampPx(px, py);
  mouse.home();
  mouse.moveBy(px, py, 0);
  delay(20);
  mouse.btnState = 1;
  mouse.report(0, 0, 1);   // Taste drücken (ohne Bewegung)
}

void onWsEvent(uint8_t clientNum, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.printf("[Roboter 1] Client %u verbunden\n", clientNum);
    mouse.wake();
  } else if (type == WStype_DISCONNECTED) {
    // Sicherheitshalber Taste loslassen, falls Verbindung mitten im Ziehen abbricht
    if (mouse.btnState) { mouse.btnState = 0; mouse.report(0, 0, 0); }
  } else if (type == WStype_TEXT) {
    StaticJsonDocument<128> doc;
    if (deserializeJson(doc, payload, length)) return;
    const char* t = doc["t"] | "";
    if (strcmp(t, "wake") == 0) {
      mouse.wake();
    } else if (strcmp(t, "click") == 0) {
      clickAt(doc["px"] | 0, doc["py"] | 0);
    } else if (strcmp(t, "dragstart") == 0) {
      dragStart(doc["px"] | 0, doc["py"] | 0);
    } else if (strcmp(t, "dragmove") == 0) {
      int dx = doc["dx"] | 0;
      int dy = doc["dy"] | 0;
      mouse.moveBy(dx, dy, mouse.btnState);   // ziehen mit gehaltener Taste
    } else if (strcmp(t, "dragend") == 0) {
      mouse.btnState = 0;
      mouse.report(0, 0, 0);   // Taste loslassen
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== Lely Bridge v1.0 ABSOLUT+DRAG: ROBOTER 1 ===");
  WiFi.config(local_IP, gateway, subnet, dns);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Verbinde mit WLAN");
  while (WiFi.status() != WL_CONNECTED) { delay(400); Serial.print("."); }
  Serial.println();
  Serial.print("Verbunden! IP-Adresse: ");
  Serial.println(WiFi.localIP());
  mouse.begin();
  USB.begin();
  webSocket.begin();
  webSocket.onEvent(onWsEvent);
  Serial.println("WebSocket-Server laeuft auf Port 81 (Absolut+Drag).");
}

void loop() {
  webSocket.loop();
}
