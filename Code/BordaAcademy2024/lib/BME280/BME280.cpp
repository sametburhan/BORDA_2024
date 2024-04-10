#include "Arduino.h"
#include "Wire.h"
#include "SparkFunBME280.h"

#include <CircularBuffer.hpp>
#include <Smoothed.h>

CircularBuffer<float, 30> bufferTemp, bufferPress, bufferAlti;
Smoothed<float> myTemp, myPress, myAlti;

#define window_size 4
#define BME_id 0x76

/***************************
 * Define and variable
 **************************/
BME280 mySensor;

typedef struct
{
    uint32_t sensor_id;
    float sensor_value1 = 0; // raw
    float sensor_value2 = 0; // raw
    float sensor_value3 = 0; // raw
} sensor_t;

sensor_t MPU, HMC, BME;

/****************************
 * Function decleration
 ***************************/
void BME_Init(sensor_t *BME);
void read_BME(sensor_t *BME);

void BME_Init(sensor_t *BME)
{
    BME->sensor_id = BME_id;
    mySensor.setI2CAddress(BME_id);
    mySensor.beginI2C();
    mySensor.setReferencePressure(101200); // Adjust the sea level pressure used for altitude calculations

    /**
     * Filter init
     */
    myTemp.begin(SMOOTHED_AVERAGE, window_size);
    myPress.begin(SMOOTHED_AVERAGE, window_size);
    myAlti.begin(SMOOTHED_AVERAGE, window_size);
    myTemp.clear();
    myPress.clear();
    myAlti.clear();
    delay(10);
}

void read_BME(sensor_t *BME)
{

    BME->sensor_value1 = mySensor.readTempC(); // mySensor.readFloatAltitudeMeters();

    BME->sensor_value2 = mySensor.readFloatPressure();
    // humidity = mySensor.readFloatHumidity(); // always reads 0% sensör hatalı
    BME->sensor_value3 = mySensor.readFloatAltitudeMeters();
}