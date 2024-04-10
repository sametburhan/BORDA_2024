#include "Arduino.h"
#include "Wire.h"
#include <stdio.h>

#include <MPU6050.cpp>

CircularBuffer<float, 30> bufferX, bufferY, bufferZ;
Smoothed<float> myX, myY, myZ;

#define HMC_id 0x0D

/****************************
 * Function decleration
 ***************************/
void HMC_Init(sensor_t *HMC);
void read_HMC(sensor_t *HMC);
void decToBinary(int n);

void HMC_Init(sensor_t *HMC)
{
    HMC->sensor_id = HMC_id;
    Wire.beginTransmission(HMC_id); // open communication with HMC5883
    Wire.write(0x0B);
    Wire.write(0x01);
    Wire.endTransmission();

    Wire.beginTransmission(HMC_id);
    Wire.write(0x09);
    Wire.write(0x01 | 0x0C | 0x10 | 0X00); // OSR_RNG_ODR_MODE
    Wire.endTransmission();

    /**
     * Filter init
     */
    myX.begin(SMOOTHED_AVERAGE, window_size);
    myY.begin(SMOOTHED_AVERAGE, window_size);
    myZ.begin(SMOOTHED_AVERAGE, window_size);
    myX.clear();
    myY.clear();
    myZ.clear();
    delay(10);
}

/**
 * Ham veriler alınmaktadır. Kalibrasyon için gerekli aşamalar bu uygulamada yapılmamaktadır.
 */
void read_HMC(sensor_t *HMC)
{
    int16_t DOR_OVL_DRDY, temp, OSR_RNG_ODR_MODE;

    Wire.beginTransmission(HMC_id);
    Wire.write(0x00);
    int16_t err = Wire.endTransmission();
    if (!err)
    {
        Wire.requestFrom(HMC_id, 10);
        HMC->sensor_value1 = (int16_t)(Wire.read() | Wire.read() << 8); // triple axis 16bit sign data -32768/+32767
        HMC->sensor_value2 = (int16_t)(Wire.read() | Wire.read() << 8); // triple axis 16bit sign data -32768/+32767
        HMC->sensor_value3 = (int16_t)(Wire.read() | Wire.read() << 8); // triple axis 16bit sign data -32768/+32767

        DOR_OVL_DRDY = (int16_t)Wire.read();

        temp = (int16_t)(Wire.read() | Wire.read() << 8); // relative temperature
        /*decToBinary(temp);*/

        OSR_RNG_ODR_MODE = (uint16_t)(Wire.read());
    }
}

/**
 * Test amaçlı kullanılmıştır
 */
void decToBinary(int n)
{
    // array to store binary number
    int binaryNum[32];

    // counter for binary array
    int i = 0;
    while (n > 0)
    {
        // storing remainder in binary array
        binaryNum[i] = n % 2;
        n = n / 2;
        i++;
    }

    // printing binary array in reverse order
    for (int j = i - 1; j >= 0; j--)
        Serial.print(binaryNum[j]);
}
