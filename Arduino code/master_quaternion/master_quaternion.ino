//in navigator_bringup
#include "Simple_MPU6050.h"
#include <rosuno.h>
#include <geometry_msgs/Quaternion.h>

Simple_MPU6050 mpu;
ENABLE_MPU_OVERFLOW_PROTECTION();

//Ros Node
ros::NodeHandle  nh;

geometry_msgs::Quaternion quaternion;

//Publisher
ros::Publisher quatPub("quaternion", &quaternion);

void print_Values (int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp)
{
  Quaternion q;
  mpu.GetQuaternion(&q, quat);
  quaternion.x = q.x;
  quaternion.y = q.y;
  quaternion.z = q.z;
  quaternion.w = q.w;
  quatPub.publish(&quaternion);
  nh.spinOnce();
  delay(10);
}

void setup()
{
  nh.initNode();
  nh.advertise(quatPub);
  mpu.SetAddress(0x68).CalibrateMPU().load_DMP_Image();// Does it all for you with Calibration
  mpu.on_FIFO(print_Values);
}

void loop()
{
  mpu.dmp_read_fifo();
}
