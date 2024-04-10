#include "Arduino.h"
#include "Wire.h"
#include <MadgwickAHRS.h>

#include <BME280.cpp>

Madgwick MadgwickFilter;
CircularBuffer<float, 30> bufferRoll, bufferPitch, bufferYaw;
Smoothed<float> myRoll, myPitch, myYaw;

/****************************
 * Variables
 ***************************/
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ, MgX, MgY, MgZ;
float SF_Acc = 16384;
float SF_Gy = 131;
float SF_Mg = 500;
float SF_Tmp = 333.87;
float g2mpss = 9.80665;
int i;
// float ROLL, PITCH, YAW;

volatile int timeCounter1;
hw_timer_t *timer1 = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

#define MPU_id 0x68

/****************************
 * Function decleration
 ***************************/
void MPU_Init(sensor_t *MPU);
void read_MPU(sensor_t *MPU);
void CalMPU(sensor_t *MPU);

void output();

void IRAM_ATTR onTimer1()
{

    portENTER_CRITICAL_ISR(&timerMux);
    timeCounter1++;
    portEXIT_CRITICAL_ISR(&timerMux);
}

void MPU_Init(sensor_t *MPU)
{
    MPU->sensor_id = MPU_id;
    Wire.begin();
    Wire.beginTransmission(MPU_id);
    Wire.write(0x6B);       // Send the requested starting register
    Wire.write(0x00);       // Set the requested starting register
    Wire.endTransmission(); // End the transmission
    // Configure the accelerometer (+/-8g)
    Wire.beginTransmission(MPU_id); // Start communicating with the MPU-6050
    Wire.write(0x1C);               // Send the requested starting register
    Wire.write(0x10);               // Set the requested starting register
    Wire.endTransmission();         // End the transmission
    // Configure the gyro (500dps full scale)
    Wire.beginTransmission(MPU_id); // Start communicating with the MPU-6050
    Wire.write(0x1B);               // Send the requested starting register
    Wire.write(0x08);               // Set the requested starting register
    Wire.endTransmission();         // End the transmission

    Wire.beginTransmission(MPU_id); // Start communication with the address found during search
    Wire.write(0x1A);               // We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);               // Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();         // End the transmission with the gyro
    Serial.println("MPU6050 Hazır.");

    // Orientation MadgwickFilter
    MadgwickFilter.begin(10); // filtering frequency 100Hz
    timer1 = timerBegin(0, 80, true);
    timerAttachInterrupt(timer1, &onTimer1, true);
    timerAlarmWrite(timer1, 100000, true);
    timerAlarmEnable(timer1);

    // Filter init
    myRoll.begin(SMOOTHED_AVERAGE, window_size);
    myPitch.begin(SMOOTHED_AVERAGE, window_size);
    myYaw.begin(SMOOTHED_AVERAGE, window_size);
    myRoll.clear();
    myPitch.clear();
    myYaw.clear();
    delay(10);
}

void read_MPU(sensor_t *MPU)
{
    //  Subroutine for reading the raw gyro and accelerometer data
    Wire.beginTransmission(MPU_id); // Start communicating with the MPU-6050
    Wire.write(0x3B);               // Send the requested starting register
    Wire.endTransmission();         // End the transmission
    Wire.requestFrom(MPU_id, 14);   // Request 14 bytes from the MPU-6050
    while (Wire.available() < 14)
        ;                                 // Wait until all the bytes are received
    AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void CalMPU(sensor_t *MPU)
{
    unsigned long time;
    time = millis();
    if (timeCounter1 > 0)
    {
        portENTER_CRITICAL(&timerMux);
        timeCounter1--;
        portEXIT_CRITICAL(&timerMux);

        read_MPU(MPU); // Read the raw acc and gyro data from the MPU-6050
        MadgwickFilter.update(GyX / SF_Gy, GyY / SF_Gy, GyZ / SF_Gy, AcX / SF_Acc, AcY / SF_Acc, AcZ / SF_Acc, MgX / SF_Mg, MgY / SF_Mg, MgZ / SF_Mg);
        /**
         * Kalman filtre benzeri bir sensör füzyon filtresi olan madgwick filtre ile oryantasyon datası ile edilmektedir.
         */
        MPU->sensor_value1 = MadgwickFilter.getRoll();
        MPU->sensor_value2 = MadgwickFilter.getPitch();
        MPU->sensor_value3 = MadgwickFilter.getYaw();
    }
}
