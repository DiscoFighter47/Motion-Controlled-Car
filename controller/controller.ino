#include <SoftwareSerial.h>

#define BT_VCC 8
#define BT_TX  10
#define BT_RX  11

SoftwareSerial mySerial(BT_TX, BT_RX); //tx rx

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

#define INTERRUPT_PIN 2

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize; 
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity; 
float degree[3];

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(BT_VCC, OUTPUT);
  digitalWrite(BT_VCC, HIGH);
  
  mySerial.begin(9600);

  Wire.begin();
  Wire.setClock(400000);

  pinMode(INTERRUPT_PIN, INPUT);
  Serial.begin(115200);
    
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
    
  if (devStatus == 0) {
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
        
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  /*mySerial.write(1);
  delay(2000);
  mySerial.write(2);
  delay(2000);

  mySerial.write(1);
  delay(2000);
  mySerial.write(3);
  delay(2000);*/

  if (!dmpReady) {
    Serial.println("Error!");
    return;
  }
    
  while (!mpuInterrupt) {}

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
  } else if (mpuIntStatus & 0x02) {
    fifoCount = mpu.getFIFOCount();
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
        
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(degree, &q, &gravity);

    double val = degree[2] * 180/M_PI;
        
    Serial.println("angle\t" + String(val));

    if (val>=25) {
       mySerial.write(3);
    } else if (val <=-25) {
      mySerial.write(2);
    } else {
      mySerial.write(1);
    }
  }
}
