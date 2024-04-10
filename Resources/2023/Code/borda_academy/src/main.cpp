#include <Arduino.h>
#include <stdio.h>
#include <stdint.h>
#include "FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Wire.h"
#include "sdkconfig.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "soc/rtc_wdt.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include <CircularBuffer.h>

typedef struct {
  uint32_t sensor_id;
  float first_value;
  float second_value;
} sensor_t;

// bme680 0x77
// bme280 0x76
sensor_t bme680,bme280;

#define bleServerName "BORDA_ESP32"
#define RUNNING_CORE_0 0
#define RUNNING_CORE_1 1

// default values
volatile float temp, hum, pres, resis;
volatile bool status = false;

// Timer variables
volatile unsigned long lastTime = 0, lastTime1 = 0;
const unsigned long timerDelay = 30000;

CircularBuffer<int, 30> buffer1,buffer2,buffer3,buffer4;
bool deviceConnected = false;

// https://www.uuidgenerator.net/
#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42f59"

// Temperature
  BLECharacteristic bmeTemperatureCelsiusCharacteristics("cba1d466-344c-4be3-ab3f-189f80dd7518", BLECharacteristic::PROPERTY_NOTIFY);
  BLEDescriptor bmeTemperatureCelsiusDescriptor(BLEUUID((uint16_t)0x2A1F));

// Humidity
  BLECharacteristic bmeHumidityCharacteristics("ca73b3ba-39f6-4ab3-91ae-186dc9577d99", BLECharacteristic::PROPERTY_NOTIFY);
  BLEDescriptor bmeHumidityDescriptor(BLEUUID((uint16_t)0x2A6F));

// Pressure
  BLECharacteristic bmePressureCharacteristics("0167d753-8af0-46f0-963d-3da7fc2c0ad7", BLECharacteristic::PROPERTY_NOTIFY);
  BLEDescriptor bmePressureDescriptor(BLEUUID((uint16_t)0x2724));

// Resistance
  BLECharacteristic bmeResistanceCharacteristics("05c3676f-d165-4b7b-857b-b0b2c8534ccf", BLECharacteristic::PROPERTY_NOTIFY);
  BLEDescriptor bmeResistanceDescriptor(BLEUUID((uint16_t)0x2902));

// onConnect and onDisconnect
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

void TaskFirst( void *pvParameters );
void TaskSecond( void *pvParameters );

void delay_task(int ms_delay){
  vTaskDelay(ms_delay / portTICK_PERIOD_MS );  
}//if needed

void i2c_sensor_read(uint8_t device_address, sensor_t *sensor_type){
  int read_pres,read_resis,read_temp,read_hum;
  if(device_address == 0x77){
    sensor_type->sensor_id = device_address;
    Wire1.beginTransmission(device_address);
    Wire1.write(byte(0x1f));              // sends data code
    Wire1.endTransmission();    // stop transmitting
    Wire1.requestFrom(device_address, (uint8_t) 2);
    if(2 <= Wire1.available()){
      read_pres = Wire1.read();
      read_pres = read_pres << 8;
      read_pres |= Wire1.read();
    }
    sensor_type->first_value = read_pres;//pres

    Wire1.beginTransmission(device_address);
    Wire1.write(byte(0x2a));
    Wire1.endTransmission();
    Wire1.requestFrom(device_address, (uint8_t) 2);
    if(2 <= Wire1.available()){
      read_resis = Wire1.read();
      read_resis = read_resis << 8;
      read_resis |= Wire1.read();
    }
    sensor_type->second_value = read_resis;//resis
  }else if(device_address == 0x76){
    sensor_type->sensor_id = device_address;
    Wire1.beginTransmission(device_address);
    Wire1.write(byte(0xfa));
    Wire1.endTransmission();
    Wire1.requestFrom(device_address, (uint8_t) 2);
    if(2 <= Wire1.available()){
      read_temp = Wire1.read();
      read_temp = read_temp << 8;
      read_temp |= Wire1.read();
    }
    sensor_type->first_value = read_temp;//temp

    Wire1.beginTransmission(device_address);
    Wire1.write(byte(0xfd));
    Wire1.endTransmission();
    Wire1.requestFrom(device_address, (uint8_t) 2);
    if(2 <= Wire1.available()){
      read_hum = Wire1.read();
      read_hum = read_hum << 8;
      read_hum |= Wire1.read();
    }
    sensor_type->second_value = read_hum;//hum
  }
}

void filter_sensor_value(){
  pres = 0;
  resis = 0;
  temp = 0;
  hum = 0;

    int i = 0, j = 0, k = 0, l = 0;
    while(!buffer1.isEmpty()){
      pres += buffer1.pop();
      i++;
    }
    pres = pres/i;

    while(!buffer2.isEmpty()){
      resis += buffer2.pop();
      j++;
    }
    resis = resis/j;

    while(!buffer3.isEmpty()){
      temp += buffer3.pop();
      k++;
    }
    temp = temp/k;

    while(!buffer4.isEmpty()){
      hum += buffer4.pop();
      l++;
    }
    hum = hum/l;
}

void setup() {
  Serial.begin(115200);
  Wire1.setPins(14,15);
  Wire1.begin();
  
  BLEDevice::init(bleServerName);

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *bmeService = pServer->createService(SERVICE_UUID);
  
  /*BLECharacteristic *pCharacteristic = bmeService->createCharacteristic(
                        CHARACTERISTIC_UUID, 
                        BLECharacteristic::PROPERTY_READ | 
                        BLECharacteristic::PROPERTY_WRITE);
  pCharacteristic->setValue("Hello World");*/

  // Temperature
    bmeService->addCharacteristic(&bmeTemperatureCelsiusCharacteristics);
    bmeTemperatureCelsiusDescriptor.setValue("BME temperature");
    bmeTemperatureCelsiusCharacteristics.addDescriptor(&bmeTemperatureCelsiusDescriptor);
  // Humidity
    bmeService->addCharacteristic(&bmeHumidityCharacteristics);
    bmeHumidityDescriptor.setValue("BME humidity");
    bmeHumidityCharacteristics.addDescriptor(&bmeHumidityDescriptor);

  // Pressure
    bmeService->addCharacteristic(&bmePressureCharacteristics);
    bmePressureDescriptor.setValue("BME pressure");
    bmePressureCharacteristics.addDescriptor(&bmePressureDescriptor);

  // Reistance
    bmeService->addCharacteristic(&bmeResistanceCharacteristics);
    bmeResistanceDescriptor.setValue("BME resistance");
    bmeResistanceCharacteristics.addDescriptor(&bmeResistanceDescriptor);

  // Start the service
  bmeService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");

  xTaskCreatePinnedToCore(
    TaskFirst
    ,  "TaskFirst"   // Name
    ,  4096  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 2 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  RUNNING_CORE_0);

  xTaskCreatePinnedToCore(
    TaskSecond
    ,  "TaskSecond"
    ,  4096  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL 
    ,  RUNNING_CORE_1);
}

void loop() {
  //Freertos
}

void TaskFirst(void *pvParameters)  // This is first task for read raw sensor data and filtering it.
{
  (void) pvParameters;
  for (;;)
  { 
      delay(1000);
      // Read data 1hz

      i2c_sensor_read(0x77,&bme680);
      buffer1.push(bme680.first_value);
      buffer2.push(bme680.second_value);

      i2c_sensor_read(0x76,&bme280);
      buffer3.push(bme280.first_value);
      buffer4.push(bme280.second_value);

    if (buffer4.isFull()) {
      // Do filtering with moving median filter into the ring buffer data.
      filter_sensor_value();
      status = true;
    }    
  }
}

void TaskSecond(void *pvParameters)   // This is second task for send filtered data through ble.
{
  (void) pvParameters;
  for (;;)
  {
    Serial.println("Searching ble connection.\n");
    delay(2000);
    while (deviceConnected) {
      if ((millis() - lastTime) > timerDelay && status) {
        static char temperatureCTemp[6];
        dtostrf(temp, 6, 2, temperatureCTemp);
        bmeTemperatureCelsiusCharacteristics.setValue(temperatureCTemp);
        bmeTemperatureCelsiusCharacteristics.notify();
        Serial.print("Temperature Celsius: ");
        Serial.print(temp);
        Serial.print(" ºC\n");

        static char humidityTemp[6];
        dtostrf(hum, 6, 2, humidityTemp);
        bmeHumidityCharacteristics.setValue(humidityTemp);
        bmeHumidityCharacteristics.notify();   
        Serial.print("Humidity: ");
        Serial.print(hum);
        Serial.println(" %\n");
        
        static char pressureTemp[6];
        dtostrf(pres, 6, 2, pressureTemp);
        bmePressureCharacteristics.setValue(pressureTemp);
        bmePressureCharacteristics.notify();
        Serial.print("Pressure Value: ");
        Serial.print(pres);
        Serial.print(" hPa\n");

        static char resistanceTemp[6];
        dtostrf(resis, 6, 2, resistanceTemp);
        bmeResistanceCharacteristics.setValue(resistanceTemp);
        bmeResistanceCharacteristics.notify();
        Serial.print("Gas Resistance: ");
        Serial.print(resis);
        Serial.print(" KOhms\n");
        lastTime = millis();
        }
      }
  }
}