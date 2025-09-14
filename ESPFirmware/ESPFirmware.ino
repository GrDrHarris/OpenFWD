#include <Arduino.h>
#include "../Communicate.h"
#define RUD7_ENABLED
#define RUD8_ENABLED
const unsigned int DEVICE_TYPE = 0x00000000;

//#define BLE_MODE
#define BLE_DEVICE_NAME     "OpenFWDModel"
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define WIFI_MODE // connect GPIO4 to GND for STA mode, otherwise AP mode
#define WIFI_SSID "OpenFWDWifi"
#define WIFI_PASSWORD "OpenFWDPassword"
#define SERVER_IP 10 // 10~128, leave controllers to >128

#if defined(WIFI_MODE) == defined(BLE_MODE)
#error "must select one of wifi mode or ble mode"
#endif

static uint8_t ID_RESPOND[MSG_LEN] = {
  TYPE_RESPOND,
  DEVICE_TYPE & 0xFF,
  (DEVICE_TYPE >> 8) & 0xFF,
  (DEVICE_TYPE >> 16) & 0xFF,
  (DEVICE_TYPE >> 24) & 0xFF,
  END_BYTE
};

#ifdef BLE_MODE
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#endif

#ifdef WIFI_MODE
#include <WiFi.h>
#define WIFI_SENSE_PIN 4
IPAddress serverIP(192, 168, 1, SERVER_IP);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
#endif

#if defined(RUD7_ENABLED) || defined(RUD8_ENABLED)
#define HAS_CHILD_RUDDER
#endif

#ifdef RUD7_ENABLED
#define RUD7_S_Pin 0
#define RUD7_P_Pin 1
static uint8_t rud_new_value7;
#endif

#ifdef RUD8_ENABLED
#define RUD8_S_Pin 2
#define RUD8_P_Pin 3
static uint8_t rud_new_value8;
#endif

static inline void parse_command(uint8_t *data, int len) {
#ifdef RUD7_ENABLED
  if(*data == RUDDER_SET_TRG && *(data+1) == 7) {
    rud_new_value7 = *(data+2) + 50;
    return;
  }
#endif
#ifdef RUD8_ENABLED
  if(*data == RUDDER_SET_TRG && *(data+1) == 8) {
    rud_new_value8 = *(data+2) + 50;
    return;
  }
#endif
  if(*data == TYPE_QUERY) {
    report_error(ID_RESPOND, 6);
    return;
  }
    Serial1.write(data, len);
}

#ifdef BLE_MODE
BLEServer *pServer;
BLEService *pService;
BLECharacteristic *pCharacteristic;
class BLECallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    uint8_t *data = pCharacteristic->getData();
    uint8_t len = pCharacteristic->getLength();
    parse_command(data, len);
  }
};
static inline void report_error(uint8_t* buf_rud, int len) {
  pCharacteristic->setValue(buf_rud, len);
  pCharacteristic->notify();
}
#endif

#ifdef WIFI_MODE
#define WIFI_MODE_PIN 4
#define WIFI_PORT 11451
class TCPServer {
  private:
    WiFiServer server;
    WiFiClient client;
    uint8_t data[MSG_LEN];
    uint8_t ptr;
  public:
    TCPServer() : server(WIFI_PORT) {
      ptr = 0;
    }
    void begin() {
      server.begin();
    }
    void update() {
      if (server.hasClient()) {
        if (client && client.connected()) {
          WiFiClient newClient = server.accept();
          newClient.stop();
        } else {
          client = server.accept();
        }
      }

      if (client && client.connected()) {
        ptr += client.read(data + ptr, MSG_LEN);
        if(data[ptr - 1] == END_BYTE) {
          parse_command(data, ptr);
          ptr = 0;
        }
      }
    }
    void writeBytes(uint8_t* data, int len) {
      if (client && client.connected())
        client.write(data, len);
    }
} tcpServer;
static inline void report_error(uint8_t* buf_rud, int len) {
  tcpServer.writeBytes(buf_rud, len);
}
#endif

#ifdef HAS_CHILD_RUDDER
#define RUDDER_PEROID 2000
static uint8_t buf_rud[MSG_LEN];
void IRAM_ATTR timerCallback() {
  static uint16_t tick_count = 0;
  tick_count++;
#ifdef RUD7_ENABLED
  static uint8_t rud_value7;
  if(tick_count == rud_value7)
    digitalWrite(RUD7_S_Pin, LOW);
#endif
#ifdef RUD8_ENABLED
  static uint8_t rud_value8;
  if(tick_count == rud_value8)
    digitalWrite(RUD8_S_Pin, LOW);
#endif
  if(tick_count == RUDDER_PEROID) {
    tick_count = 0;
#ifdef RUD7_ENABLED
    digitalWrite(RUD7_S_Pin, HIGH);
    rud_value7 = rud_new_value7;
    if(digitalRead(RUD7_P_Pin)) {
      buf_rud[0] = RUDDER_PROTECT;
      buf_rud[1] = 7;
      buf_rud[2] = END_BYTE;
      report_error(buf_rud, 3);
    }
#endif
#ifdef RUD8_ENABLED
    digitalWrite(RUD8_S_Pin, HIGH);
    rud_value8 = rud_new_value8;
    if(digitalRead(RUD8_P_Pin)) {
      buf_rud[0] = RUDDER_PROTECT;
      buf_rud[1] = 8;
      buf_rud[2] = END_BYTE;
      report_error(buf_rud, 3);
    }
#endif
  }
}
#endif
void setup() {
  Serial1.begin(115200, SERIAL_8N1, 20, 21);
#ifdef RUD7_ENABLED
  pinMode(RUD7_S_Pin, OUTPUT);
  pinMode(RUD7_P_Pin, INPUT);
#endif
#ifdef RUD8_ENABLED
  pinMode(RUD8_S_Pin, OUTPUT);
  pinMode(RUD8_P_Pin, INPUT);
#endif

#ifdef BLE_MODE
  BLEDevice::init(BLE_DEVICE_NAME);
  pServer = BLEDevice::createServer();
  pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | 
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->setCallbacks(new BLECallback());
  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
#endif

#ifdef WIFI_MODE
  pinMode(WIFI_SENSE_PIN, INPUT_PULLUP);
  int wifi_active = digitalRead(WIFI_SENSE_PIN);
  if (wifi_active) {
    WiFi.softAPConfig(serverIP, gateway, subnet);
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  } else {
    WiFi.config(serverIP, gateway, subnet);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while(WiFi.status() != WL_CONNECTED)
      delay(100);
  }
  
  tcpServer.begin();
#endif

  hw_timer_t *timer = timerBegin(100000); // 100kHz
  timerAttachInterrupt(timer, timerCallback);
  timerAlarm(timer, 1, true, 0); 
}

void loop() {
  static uint8_t buffer[MSG_LEN];
  static int ptr = 0;
  if (Serial1.available()) {
    buffer[ptr] = Serial1.read();
    ptr++;
    if(buffer[ptr-1] == END_BYTE) {
#ifdef BLE_MODE
      pCharacteristic->setValue(buffer, ptr);
      pCharacteristic->notify();
#endif
#ifdef WIFI_MODE
      tcpServer.writeBytes(buffer, ptr);
#endif
      ptr = 0;
    }
  }
#ifdef WIFI_MODE
    tcpServer.update();
#endif
}
