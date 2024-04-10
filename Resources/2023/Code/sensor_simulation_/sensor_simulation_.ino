#include <Arduino.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define DHTPIN 22
#define DHTTYPE    DHT11
DHT_Unified dht(DHTPIN, DHTTYPE);
uint8_t SLAVE_ADDR2 = 0x76;
uint8_t SLAVE_ADDR1 = 0x77;
volatile int8_t hum_msb,hum_lsb,temp_msb,temp_lsb,pres_msb,pres_lsb,resis_msb,resis_lsb;

volatile float temp, hum, resis = 28, pres = 83;
volatile int receive_data1, receive_data2;

void Request_first(){
  if(receive_data1 == 0x1f){//press
      Wire.write(byte(pres_msb));
      Wire.write(byte(pres_lsb));
  }else if(receive_data1 == 0x2a){//resis
      Wire.write(byte(resis_msb));
      Wire.write(byte(resis_lsb));
  }
}

void Receive_first(int a){//0x77
  receive_data1 = Wire.read();
}

void Request_second(){
  if(receive_data2 == 0xfa){//temp
      Wire1.write(byte(temp_msb));
      Wire1.write(byte(temp_lsb));
  }else if(receive_data2 == 0xfd){//hum
      Wire1.write(byte(hum_msb));
      Wire1.write(byte(hum_lsb));
  }
}

void Receive_second(int b){//0x76
  receive_data2 = Wire1.read();
}

void bitwise(int value, int type){
  switch (type){
    case 0:
      //temp shifting
      temp_lsb = (unsigned)value & 0xff;
      temp_msb = (unsigned)value >> 8;
    break;
    case 1:
      //hum shifting
      hum_lsb = (unsigned)value & 0xff;
      hum_msb = (unsigned)value >> 8;
    break;
    case 2:
      //pres shifting
      pres_lsb = (unsigned)value & 0xff;
      pres_msb = (unsigned)value >> 8;
    break;
    case 3:
      //resis shifting
      resis_lsb = (unsigned)value & 0xff;
      resis_msb = (unsigned)value >> 8;
    break;
    default:
    break;
  }
}

void setup(){
  Serial.begin(115200);
  Wire.setSDA(20);
  Wire.setSCL(21);
  Wire.begin(SLAVE_ADDR1);
  Wire.onReceive(Receive_first);
  Wire.onRequest(Request_first);
  
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire1.begin(SLAVE_ADDR2);
  Wire1.onReceive(Receive_second);
  Wire1.onRequest(Request_second);
  
  dht.begin();
}

void loop(){
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  temp = event.temperature;
  bitwise((int)temp, 0);
  
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  hum = event.relative_humidity;
  bitwise((int)hum, 1);
  Serial.println(hum);
  Serial.println(temp);

  bitwise((int)pres,2);
  bitwise((int)resis,3);
}
