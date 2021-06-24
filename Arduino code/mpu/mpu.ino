#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <rosuno.h>
#include <geometry_msgs/Quaternion.h>

//Ros Node
ros::NodeHandle  nh;

geometry_msgs::Quaternion quaternion;

//Publisher
ros::Publisher quatPub("quaternion", &quaternion);

MPU6050 mpu;
Quaternion q;

uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

volatile bool mpuInterrupt = false;
void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup()
{
  nh.initNode();
  nh.advertise(quatPub);
  Wire.begin();
  TWBR = 24;
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setXAccelOffset(-1956);
  mpu.setYAccelOffset(-862);
  mpu.setZAccelOffset(5250);
  mpu.setXGyroOffset(54);
  mpu.setYGyroOffset(3);
  mpu.setZGyroOffset(36);
  mpu.setDMPEnabled(true);
  attachInterrupt(0, dmpDataReady, RISING);
  packetSize = mpu.dmpGetFIFOPacketSize();
}

void loop()
{
  while (!mpuInterrupt);
  mpuInterrupt = false;
  fifoCount = mpu.getFIFOCount();
  if (fifoCount == 1024)
  {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }
  else
  {
    if ((fifoCount % packetSize != 0) || (fifoCount < packetSize) )
    {
      mpu.resetFIFO();
      Serial.println("stopped in reset fifo");
    }
    else
    {
      while (fifoCount >= packetSize)
      {
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
      }
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      quaternion.x = q.x;
      quaternion.y = q.y;
      quaternion.z = q.z;
      quaternion.w = q.w;
      quatPub.publish(&quaternion);
      nh.spinOnce();
      delay(1);
    }
  }
}
