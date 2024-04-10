#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "BLE_CONNECT.cpp"

#include "SearchI2C.cpp"

/***************************
 * Define and variable
 **************************/
#define RUNNING_CORE_0 0
#define RUNNING_CORE_1 1

/****************************
 * Tasks decleration
 ***************************/
TaskHandle_t Task1;
TaskHandle_t Task2;
void Task2code(void *pvParameters);
void Task1code(void *pvParameters);
void filter_sensor_value();

void setup(void)
{
  Wire.begin();
  Wire.setClock(400000); // Increase to fast I2C speed!
  Serial.begin(115200);
  Serial.println("Uart and I2C started!");

  xTaskCreatePinnedToCore(
      Task1code, /* Task function. */
      "Task1",   /* name of task. */
      10000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      1,         /* priority of the task */
      &Task1,    /* Task handle to keep track of created task */
      RUNNING_CORE_0);        /* pin task to core 0 */
  delay(200);

  // create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
      Task2code, /* Task function. */
      "Task2",   /* name of task. */
      10000,     /* Stack size of task */
      NULL,      /* parameter of the task */
      1,         /* priority of the task */
      &Task2,    /* Task handle to keep track of created task */
      RUNNING_CORE_1);        /* pin task to core 1 */
  delay(200);
}

void loop(void)
{
}

/**
 * Filtre aşamasını farklı yorumladım.
 */
void filter_sensor_value()
{
  // for BME
  myTemp.add(BME.sensor_value1);
  bufferTemp.push(myTemp.get());

  myPress.add(BME.sensor_value2);
  bufferPress.push(myPress.get());

  myAlti.add(BME.sensor_value3);
  bufferAlti.push(myAlti.get());

  // for MPU
  myRoll.add(MPU.sensor_value1);
  bufferRoll.push(myRoll.get());

  myPitch.add(MPU.sensor_value2);
  bufferPitch.push(myPitch.get());

  myYaw.add(MPU.sensor_value3);
  bufferYaw.push(myYaw.get());

  // for HMC
  myX.add(HMC.sensor_value1);
  bufferX.push(myX.get());

  myY.add(HMC.sensor_value2);
  bufferY.push(myY.get());

  myZ.add(HMC.sensor_value3);
  bufferZ.push(myZ.get());
}

/****************************
 * Task 1
 ***************************/
void Task1code(void *pvParameters)
{
  Serial.print("Task 1: ");
  Serial.println(xPortGetCoreID());

  MPU_Init(&MPU);
  BME_Init(&BME);
  HMC_Init(&HMC);

  for (;;)
  {
    if ((millis() - lastTime1) > timerDelay1) // 1hz
    {
      read_BME(&BME);
      CalMPU(&MPU);
      read_HMC(&HMC);
      filter_sensor_value();
      lastTime1 = millis();
      status = true;
    }
    delay(100);
  }
}

/****************************
 * Task 2
 ***************************/
void Task2code(void *pvParameters)
{
  Serial.print("Task 2: ");
  Serial.println(xPortGetCoreID());

  BLE_Init();

  for (;;)
  {
    while (deviceConnected)
    {
      if ((millis() - lastTime) > timerDelay && status) // 30 sec
      {
        push_BLE();
        lastTime = millis();
      }
    }
    delay(100);
  }
}
