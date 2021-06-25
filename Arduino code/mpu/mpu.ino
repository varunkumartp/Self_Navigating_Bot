#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <rosuno.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32.h>

// Ultrasonic sensor pins
#define trigl 4
#define echol 5
#define trigc 6
#define echoc 7
#define trigr 8
#define echor 9

//Ros Node
ros::NodeHandle  nh;

geometry_msgs::Quaternion quaternion;
std_msgs::Float32 USSL;
std_msgs::Float32 USSC;
std_msgs::Float32 USSR;

//Publisher
ros::Publisher quatPub("quaternion", &quaternion);
ros::Publisher leftPub("left", &USSL);
ros::Publisher centerPub("center", &USSC);
ros::Publisher rightPub("right", &USSR);

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

float checkDistance(int echo, int trig)
{
  digitalWrite(trig, HIGH);
  delayMicroseconds(2);
  digitalWrite(trig, LOW);
  delayMicroseconds(10);
  digitalWrite(trig, HIGH);
  long duration = pulseIn(echo, HIGH);
  float distance = (duration * 0.034 / 2)/100;
  if (distance > 1.00)
    return 1.00;
  return distance;  
}

void setup()
{
  nh.initNode();
  nh.advertise(quatPub);
  nh.advertise(leftPub);
  nh.advertise(centerPub);
  nh.advertise(rightPub);
  pinMode(trigl, OUTPUT);
  pinMode(trigc, OUTPUT);
  pinMode(trigr, OUTPUT);
  pinMode(echol, INPUT);
  pinMode(echoc, INPUT);
  pinMode(echor, INPUT);
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
      USSL.data = checkDistance(echol, trigl);
      delay(30);
      USSC.data = checkDistance(echoc, trigc);
      delay(30);
      USSR.data = checkDistance(echor, trigr);
      delay(30);
      leftPub.publish(&USSL);
      centerPub.publish(&USSC);
      rightPub.publish(&USSR);
      nh.spinOnce();
      delay(1);
    }
  }
}
