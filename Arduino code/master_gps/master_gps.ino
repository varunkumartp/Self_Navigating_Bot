#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <ros.h>
#include <std_msgs/Float32.h>

//Ros Node
ros::NodeHandle  nh;

std_msgs::Float32 lat;
std_msgs::Float32 lon;

//Publisher
ros::Publisher latPub("lat", &lat);
ros::Publisher lonPub("lon", &lon);

// Soft Serial for gps
SoftwareSerial gpsSerial(8, 9); // RX TX

// GPS object
TinyGPS gps;

void setup()
{
  nh.initNode();
  nh.advertise(latPub);
  nh.advertise(lonPub);
  gpsSerial.begin(9600);
}

void loop()
{
  while (gpsSerial.available()) // check for gps data
  {
    if (gps.encode(gpsSerial.read()))
    {
      gps.f_get_position(&lat.data, &lon.data); // get latitude and longitude
      latPub.publish(&lat);
      delay(1);
      lonPub.publish(&lon);
      delay(1);
      nh.spinOnce();
      delay(1);
    }
  }
  nh.spinOnce();
  delay(1);
}
