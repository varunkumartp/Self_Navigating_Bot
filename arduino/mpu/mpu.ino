#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <MechaQMC5883.h>
#include <rosuno.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32.h>

//Ros Node
ros::NodeHandle  nh;

geometry_msgs::Quaternion quaternion;
std_msgs::Float32 heading;
std_msgs::Float32 latitude;
std_msgs::Float32 longitude;

//Publisher
ros::Publisher quatPub("imu/quat", &quaternion);
ros::Publisher headPub("imu/heading", &heading);
ros::Publisher latPub("coordinates/lat", &latitude);
ros::Publisher lonPub("coordinates/lon", &longitude);

SoftwareSerial gpsSerial(10, 11);
TinyGPS gps;
MPU6050 mpu;
MechaQMC5883 qmc;
Quaternion q;

float x;
float y;
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
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(quatPub);
  nh.advertise(headPub);
  nh.advertise(lonPub);
  nh.advertise(latPub);
  Wire.begin();
  TWBR = 24;
  qmc.init();
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
  gpsSerial.begin(9600);
}

void getQuat()
{
  while (!mpuInterrupt);
  mpuInterrupt = false;
  fifoCount = mpu.getFIFOCount();
  if (fifoCount == 1024)
  {
    mpu.resetFIFO();
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
      delay(1);
      quatPub.publish(&quaternion);
    }
  }
}

void getHeading()
{
  int x, y, z;
  qmc.read(&x, &y, &z);
  heading.data = atan2(x, y);
  delay(1);
  headPub.publish(&heading);
}

void getCoordinates()
{
  if (gpsSerial.available())
  {
    if (gps.encode(gpsSerial.read()))
    {
      gps.f_get_position(&x, &y);
      latitude.data = x;
      longitude.data = y;
      delay(1);
      latPub.publish(&latitude);
      delay(1);
      lonPub.publish(&longitude);
    }
  }
}

void loop()
{
  getQuat();
  getHeading();
  getCoordinates();
  nh.spinOnce();
  delay(1);
}
