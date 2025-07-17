#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#include "../Communicate.h"
#define RUD7_ENABLED
#define RUD8_ENABLED

#define BLE_DEVICE_NAME     "OpenFWDModel"
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
BLEServer *pServer;
BLEService *pService;
BLECharacteristic *pCharacteristic;

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

class BLECallback : public BLECharacteristicCallbacks {
  uint8_t msg_buf[MSG_LEN];
  void onWrite(BLECharacteristic *pCharacteristic) {
    uint8_t *data = pCharacteristic->getData();
    uint8_t len = pCharacteristic->getLength();
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
    if(len < MSG_LEN) {
      memcpy(msg_buf, data, len);
      Serial.write(msg_buf, MSG_LEN);
    } else if(len == MSG_LEN) {
      Serial.write(data, len);
    }
  }
};
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
      pCharacteristic->setValue(buf_rud, MSG_LEN);
      pCharacteristic->notify();
    }
#endif
#ifdef RUD8_ENABLED
    digitalWrite(RUD8_S_Pin, HIGH);
    rud_value8 = rud_new_value8;
    if(digitalRead(RUD8_P_Pin)) {
      buf_rud[0] = RUDDER_PROTECT;
      buf_rud[1] = 8;
      pCharacteristic->setValue(buf_rud, MSG_LEN);
      pCharacteristic->notify();
    }
#endif
  }
}
#endif
void setup() {
  Serial.begin(115200);
#ifdef RUD7_ENABLED
  pinMode(RUD7_S_Pin, OUTPUT);
  pinMode(RUD7_P_Pin, INPUT);
#endif
#ifdef RUD8_ENABLED
  pinMode(RUD8_S_Pin, OUTPUT);
  pinMode(RUD8_P_Pin, INPUT);
#endif

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

  hw_timer_t *timer = timerBegin(100000); // 100kHz
  timerAttachInterrupt(timer, timerCallback);
  timerAlarm(timer, 1, true, 0); 
}

void loop() {
  uint8_t buffer[MSG_LEN];
  if (Serial.available()) {
    Serial.readBytes(buffer, MSG_LEN);
    pCharacteristic->setValue(buffer, MSG_LEN);
    pCharacteristic->notify();
  }
}
