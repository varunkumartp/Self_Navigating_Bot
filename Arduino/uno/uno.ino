// disconnect gps while running rosserial_python
// Final uno
// An arduino mega can be used and both the uno and nano can be combined to form a single code 
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <rosuno.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Quaternion.h>

#define echor 4
#define trigr 5
#define echoc 6
#define trigc 7
#define echol 8
#define trigl 9

SoftwareSerial gpsSerial(10, 11);
TinyGPS gps;
float x, y;

// ros node
ros::NodeHandle nh;

std_msgs::Float32 latitude;
std_msgs::Float32 longitude;
std_msgs::Float32 rangeL;
std_msgs::Float32 rangeR;
std_msgs::Float32 rangeC;
geometry_msgs::Quaternion quaternion;

//Publisher buffer size 300 bytes
ros::Publisher quatPub("imu/quat", &quaternion);
ros::Publisher latPub("coordinates/lat", &latitude);
ros::Publisher lonPub("coordinates/lon", &longitude);
ros::Publisher rangeCPub("sonar/center", &rangeC);
ros::Publisher rangeRPub("sonar/right", &rangeR);
ros::Publisher rangeLPub("sonar/left", &rangeL);

MPU6050 mpu;
Quaternion q;

uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

volatile bool mpuInterrupt = false;

void getCoordinates()
{
  gpsSerial.begin(9600);
  
  unsigned long start = millis();
  do
  {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < 150);
  
  gps.f_get_position(&x, &y);
  
  latitude.data = x;
  longitude.data = y;
  
  gpsSerial.end();
  
  latPub.publish(&latitude);
  lonPub.publish(&longitude);
}

void dmpDataReady()
{
  mpuInterrupt = true;
}


float checkDistance(int echo, int trig)
{
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH);
  float distance = (duration * 0.0343 / 2) / 100;
  delay(32);
  if (distance > 1.00)
    return 1.00;
  return distance;
}

void checkSonar()
{
  rangeC.data = checkDistance(echoc, trigc);
  rangeL.data = checkDistance(echol, trigl);
  rangeR.data = checkDistance(echor, trigr);
  rangeCPub.publish(&rangeC);
  rangeLPub.publish(&rangeL);
  rangeRPub.publish(&rangeR);
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
    if ((fifoCount % packetSize != 0) || (fifoCount < packetSize))
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
      delay(1);
    }
  }
}

void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(lonPub);
  nh.advertise(latPub);
  nh.advertise(rangeLPub);
  nh.advertise(rangeRPub);
  nh.advertise(rangeCPub);
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
  pinMode(echol, INPUT);
  pinMode(echoc, INPUT);
  pinMode(echor, INPUT);
  pinMode(trigl, OUTPUT);
  pinMode(trigc, OUTPUT);
  pinMode(trigr, OUTPUT);
}

void loop()
{
  getCoordinates();
  getQuat();
  checkSonar();
  nh.spinOnce();
  delay(1);
}
