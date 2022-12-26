#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <BMP280_DEV.h>
#include "Wire.h"
#include <SPI.h>
#include <SD.h>

MPU6050 mpu;
BMP280_DEV bmp280;
File file;

#define led_state (int[]){6,7,8,9}
#define sd_pin 10

uint8_t state;
uint8_t calibration_time;
bool parachute_flag, calibration_flag;
float temperature, pressure, altitude;

uint8_t fifoBuffer[64];
Quaternion q;           
VectorInt16 aa;         
VectorInt16 aaReal;     
VectorInt16 aaWorld;    
VectorFloat gravity;    
float ypr[3];   

void setup() {
pinMode(led_state[0],OUTPUT);
pinMode(led_state[1],OUTPUT);
pinMode(led_state[2],OUTPUT);
pinMode(led_state[3],OUTPUT);
Serial.begin(115200);
Wire.begin();
Wire.setClock(400000);
sensors_innit();
SD.begin(sd_pin);
SD.mkdir("/data");

state = 1;

}

void loop() {

  switch (state) {
    case 1:
      calibration_state(aaReal ,ypr,temperature, pressure, altitude);
      break;
    case 2: 
      flying_state(aaReal, ypr, temperature, pressure, altitude);
      break;
    case 3: 
      recovery_state();
      break;
  }
}

void calibration_state(VectorInt16 aaReal ,float ypr[], float temperature, float pressure, float altitude){

  digitalWrite(led_state[1],1);
  digitalWrite(led_state[2],0);
  digitalWrite(led_state[3],0);

  get_sensors_data(aaReal,ypr, temperature, pressure, altitude);
  print_data(aaReal,ypr, temperature, pressure, altitude);

  if(millis() - calibration_time > 20000 && abs(ypr[1] * 180/M_PI) < 0.5 && abs(ypr[2] * 180/M_PI) < 0.5){
    calibration_flag = true;
    state = 2;
  }
}
void flying_state(VectorInt16 aaReal,float ypr[], float temperature, float pressure, float altitude){

  digitalWrite(led_state[1],0);
  digitalWrite(led_state[2],1);
  digitalWrite(led_state[3],0);
  
  get_sensors_data(aaReal,ypr, temperature, pressure, altitude);
  write_data(aaReal,ypr, temperature, pressure, altitude);
  print_data(aaReal,ypr, temperature, pressure, altitude);
  
  if(!parachute_flag){
    parachute_deployd(ypr);
  }
  if(parachute_flag){
        
      state = 3;
    }
} 

void recovery_state(){
  
  if (file){
    file.close();
  }
}

bool sensors_test(){
  if(!mpu.testConnection()){                            
    return false; 
  }
  return true;
}

void get_sensors_data(VectorInt16 aaReal,float ypr[], float temperature, float pressure, float altitude){

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      }
  if(bmp280.getMeasurements(temperature, pressure, altitude));
}

bool parachute_deployd(float ypr[]){
  
  if(abs(ypr[1] * 180/M_PI) > 89.0||abs(ypr[2] * 180/M_PI) > 89.0){
    parachute_flag = true;
  }
  return parachute_flag;
}

void write_data(VectorInt16 aaReal,float ypr[],float temperature, float pressure, float altitude){
  String data = " ";
  file = SD.open("/data/data.csv", FILE_WRITE);
  data += String(aaReal.x)+","+String(aaReal.y)+","+String(aaReal.z)+","+String(ypr[0])+","+String(ypr[1])+","+String(ypr[2])+","+String(temperature)+","+String(pressure)+","+String(altitude);
  
  if (file){
    file.println(data);
    file.close();
  }
}

void sensors_innit(){

  bool dmpReady = false; 
  uint8_t mpuIntStatus;   
  uint8_t devStatus;      
  uint16_t packetSize;    
  uint16_t fifoCount;     

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 

    if (devStatus == 0) {

        mpu.CalibrateAccel(15);
        mpu.CalibrateGyro(15);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);


        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }


    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  bmp280.begin(BMP280_I2C_ALT_ADDR);             
  bmp280.setPresOversampling(OVERSAMPLING_X16);    
  bmp280.setTempOversampling(OVERSAMPLING_X16);    
  bmp280.setIIRFilter(IIR_FILTER_16);              
  bmp280.setTimeStandby(TIME_STANDBY_05MS);     
  bmp280.startNormalConversion();                 

}

void print_data(VectorInt16 aaReal ,float ypr[], float temperature, float pressure, float altitude){
   Serial.print(state);

    Serial.print(aaReal.x);
    Serial.print(F("\t"));
    Serial.print(aaReal.y);
    Serial.print(F("\t"));
    Serial.print(aaReal.z);
    Serial.print(F("\t"));

    Serial.print(ypr[1] * 180/M_PI);
    Serial.print(F("\t"));
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print(F("\t"));
    Serial.print(ypr[2] * 180/M_PI);
    Serial.print(F("\t"));

    Serial.print(temperature);
    Serial.print("\t");  
    Serial.print(F("*C"));
    Serial.print(F("\t"));
    Serial.print(pressure);    
    Serial.print(F("\t"));
    Serial.print(altitude);
    Serial.print(F("\t"));
    Serial.print(F("\n"));
    
} 



