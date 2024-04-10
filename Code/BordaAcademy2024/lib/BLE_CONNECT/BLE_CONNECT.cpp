#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include "QMC5883L.cpp"

/*#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"*/

/****************************
 * Function decleration
 ***************************/
void BLE_Init();
void push_BLE();

/***************************
 * Define and variable
 **************************/
#define bleServerName "BORDA_ESP32"

// Timer variables
volatile unsigned long lastTime = 0, lastTime1 = 0;
const unsigned long timerDelay = 30000, timerDelay1 = 1000;
volatile bool deviceConnected = false;
volatile bool status = false;

double temp = 14, hum = 25, pres = 38, resis = 41;

// https://www.uuidgenerator.net/
#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42f59"
#define SERVICE_UUID1 "6ce1e6fa-1930-4252-ba31-591e0e2b7473"
#define SERVICE_UUID2 "abc23f39-1266-429a-b420-0797d410dba1"
// BME
BLECharacteristic bmeTemperatureCelsiusCharacteristics("cba1d466-344c-4be3-ab3f-189f80dd7518", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor bmeTemperatureCelsiusDescriptor(BLEUUID((uint16_t)0x1));

BLECharacteristic bmeAltitudeCharacteristics("ca73b3ba-39f6-4ab3-91ae-186dc9577d99", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor bmeAltitudeDescriptor(BLEUUID((uint16_t)0x2));

BLECharacteristic bmePressureCharacteristics("0167d753-8af0-46f0-963d-3da7fc2c0ad7", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor bmePressureDescriptor(BLEUUID((uint16_t)0x3));

// HMC
BLECharacteristic hmcXCharacteristics("05c3676f-d165-4b7b-857b-b0b2c8534ccf", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor hmcXDescriptor(BLEUUID((uint16_t)0x4));

BLECharacteristic hmcYCharacteristics("dcf9689a-74b7-48ab-982d-200d681e7f1d", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor hmcYDescriptor(BLEUUID((uint16_t)0x5));

BLECharacteristic hmcZCharacteristics("a8bc42c7-b0b0-4211-8cb2-d74bfc56057f", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor hmcZDescriptor(BLEUUID((uint16_t)0x6));

// MPU
BLECharacteristic mpuRollCharacteristics("58484acc-8f99-4221-9e0e-8dc7686bf33c", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor mpuRollDescriptor(BLEUUID((uint16_t)0x7));

BLECharacteristic mpuPitchCharacteristics("d52c5987-87d9-4637-a398-99eb26a01de7", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor mpuPitchDescriptor(BLEUUID((uint16_t)0x8));

BLECharacteristic mpuYawCharacteristics("55844b93-1178-4aba-a100-dee4ca81262f", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor mpuYawDescriptor(BLEUUID((uint16_t)0x9));

// onConnect and onDisconnect
class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
    };
    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
    }
};

void BLE_Init()
{
    BLEDevice::init(bleServerName);
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *bleService = pServer->createService(SERVICE_UUID);
    BLEService *bleService1 = pServer->createService(SERVICE_UUID1);
    BLEService *bleService2 = pServer->createService(SERVICE_UUID2);

    // Temperature
    bleService->addCharacteristic(&bmeTemperatureCelsiusCharacteristics);
    bmeTemperatureCelsiusDescriptor.setValue("BME temperature");
    bmeTemperatureCelsiusCharacteristics.addDescriptor(&bmeTemperatureCelsiusDescriptor);

    // Pressure
    bleService->addCharacteristic(&bmePressureCharacteristics);
    bmePressureDescriptor.setValue("BME pressure");
    bmePressureCharacteristics.addDescriptor(&bmePressureDescriptor);

    // Altitude
    bleService->addCharacteristic(&bmeAltitudeCharacteristics);
    bmeAltitudeDescriptor.setValue("BME altitude");
    bmeAltitudeCharacteristics.addDescriptor(&bmeAltitudeDescriptor);

    // X
    bleService1->addCharacteristic(&hmcXCharacteristics);
    hmcXDescriptor.setValue("HMC X");
    hmcXCharacteristics.addDescriptor(&hmcXDescriptor);

    // Y
    bleService1->addCharacteristic(&hmcYCharacteristics);
    hmcYDescriptor.setValue("HMC Y");
    hmcYCharacteristics.addDescriptor(&hmcYDescriptor);

    // Z
    bleService1->addCharacteristic(&hmcZCharacteristics);
    hmcZDescriptor.setValue("HMC Z");
    hmcZCharacteristics.addDescriptor(&hmcZDescriptor);

    // Roll
    bleService2->addCharacteristic(&mpuRollCharacteristics);
    mpuRollDescriptor.setValue("MPU Roll");
    mpuRollCharacteristics.addDescriptor(&mpuRollDescriptor);

    // Pitch
    bleService2->addCharacteristic(&mpuPitchCharacteristics);
    mpuPitchDescriptor.setValue("MPU Pitch");
    mpuPitchCharacteristics.addDescriptor(&mpuPitchDescriptor);

    // Yaw
    bleService2->addCharacteristic(&mpuYawCharacteristics);
    mpuYawDescriptor.setValue("MPU Yaw");
    mpuYawCharacteristics.addDescriptor(&mpuYawDescriptor);

    // Start the service
    bleService->start();
    bleService1->start();
    bleService2->start();

    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pServer->getAdvertising()->start();
}

void push_BLE()
{
    // Temperature
    if (!bufferTemp.isEmpty())
    {
        static char temperatureTemp[6];
        dtostrf(bufferTemp.pop(), 6, 2, temperatureTemp);
        bmeTemperatureCelsiusCharacteristics.setValue(temperatureTemp);
        bmeTemperatureCelsiusCharacteristics.notify();
        Serial.print("Temperature Celsius: ");
        Serial.print(temperatureTemp);
        Serial.print(" ºC\n");
    }

    // Pressure
    if (!bufferPress.isEmpty())
    {
        static char pressureTemp[6];
        dtostrf(bufferPress.pop(), 6, 2, pressureTemp);
        bmePressureCharacteristics.setValue(pressureTemp);
        bmePressureCharacteristics.notify();
        Serial.print("Pressure Value: ");
        Serial.print(pressureTemp);
        Serial.print(" hPa\n");
    }

    // Altitude
    if (!bufferAlti.isEmpty())
    {
        static char altitudeTemp[6];
        dtostrf(bufferAlti.pop(), 6, 2, altitudeTemp);
        bmeAltitudeCharacteristics.setValue(altitudeTemp);
        bmeAltitudeCharacteristics.notify();
        Serial.print("Altitude: ");
        Serial.print(altitudeTemp);
        Serial.println(" \n");
    }

    // Roll
    if (!bufferRoll.isEmpty())
    {
        static char rollTemp[6];
        dtostrf(bufferRoll.pop(), 6, 2, rollTemp);
        mpuRollCharacteristics.setValue(rollTemp);
        mpuRollCharacteristics.notify();
        Serial.print("Roll: ");
        Serial.print(rollTemp);
        Serial.println(" \n");
    }

    // Pitch
    if (!bufferPitch.isEmpty())
    {
        static char pitchTemp[6];
        dtostrf(bufferPitch.pop(), 6, 2, pitchTemp);
        mpuPitchCharacteristics.setValue(pitchTemp);
        mpuPitchCharacteristics.notify();
        Serial.print("Pitch: ");
        Serial.print(pitchTemp);
        Serial.println(" \n");
    }

    // Yaw
    if (!bufferYaw.isEmpty())
    {
        static char yawTemp[6];
        dtostrf(bufferYaw.pop(), 6, 2, yawTemp);
        mpuYawCharacteristics.setValue(yawTemp);
        mpuYawCharacteristics.notify();
        Serial.print("Yaw: ");
        Serial.print(yawTemp);
        Serial.println(" \n");
    }

    // X
    if (!bufferX.isEmpty())
    {
        static char xTemp[6];
        dtostrf(bufferX.pop(), 6, 2, xTemp);
        hmcXCharacteristics.setValue(xTemp);
        hmcXCharacteristics.notify();
        Serial.print("X: ");
        Serial.print(xTemp);
        Serial.println(" \n");
    }

    // Y
    if (!bufferY.isEmpty())
    {
        static char yTemp[6];
        dtostrf(bufferY.pop(), 6, 2, yTemp);
        hmcYCharacteristics.setValue(yTemp);
        hmcYCharacteristics.notify();
        Serial.print("Y: ");
        Serial.print(yTemp);
        Serial.println(" \n");
    }

    // Z
    if (!bufferZ.isEmpty())
    {
        static char zTemp[6];
        dtostrf(bufferZ.pop(), 6, 2, zTemp);
        hmcZCharacteristics.setValue(zTemp);
        hmcZCharacteristics.notify();
        Serial.print("Z: ");
        Serial.print(zTemp);
        Serial.println(" \n");
    }
}