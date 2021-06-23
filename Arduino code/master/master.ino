// in navigator_bringup
#include "Simple_MPU6050.h"
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <ros.h>
#include <std_msgs/Float32.h>

Simple_MPU6050 mpu;
ENABLE_MPU_OVERFLOW_PROTECTION();

//Ros Node
ros::NodeHandle  nh;

std_msgs::Float32 lat;
std_msgs::Float32 lon;
std_msgs::Float32 qx;
std_msgs::Float32 qy;
std_msgs::Float32 qz;
std_msgs::Float32 qw;

//Publisher
ros::Publisher latPub("lat", &lat);
ros::Publisher lonPub("lon", &lon);
ros::Publisher qxPub("qx", &qx);
ros::Publisher qyPub("qy", &qy);
ros::Publisher qzPub("qz", &qz);
ros::Publisher qwPub("qw", &qw);

// Soft Serial for gps
SoftwareSerial gpsSerial(3, 4); // RX TX

// GPS object
TinyGPS gps;

void print_Values (int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp)
{
  Quaternion q;
  mpu.GetQuaternion(&q, quat);
  qx.data = q.x;
  qy.data = q.y;
  qz.data = q.z;
  qw.data = q.w;
}

void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(latPub);
  nh.advertise(lonPub);
  nh.advertise(qxPub);
  nh.advertise(qyPub);
  nh.advertise(qzPub);
  nh.advertise(qwPub);
  gpsSerial.begin(9600);
  mpu.SetAddress(0x68).CalibrateMPU().load_DMP_Image();// Does it all for you with Calibration
  mpu.on_FIFO(print_Values);
}

void loop()
{
  mpu.dmp_read_fifo();
  if (gpsSerial.available()) // check for gps data
  {
    if (gps.encode(gpsSerial.read()))
      gps.f_get_position(&lat.data, &lon.data); // get latitude and longitude
  }
  qxPub.publish(&qx);
  qyPub.publish(&qy);
  qzPub.publish(&qz);
  qwPub.publish(&qw);
  latPub.publish(&lat);
  lonPub.publish(&lon);
  nh.spinOnce();
  delay(100);
}
