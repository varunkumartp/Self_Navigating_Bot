#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <ros.h>
#include <std_msgs/Float64.h>

float latitude;
float longitude;

//Ros Node
ros::NodeHandle  nh;

std_msgs::Float64 lat;
std_msgs::Float64 lon;

//Publisher
ros::Publisher latPub("lat", &lat);
ros::Publisher lonPub("lon", &lon);

// Soft Serial for gps
SoftwareSerial gpsSerial(8, 9); // RX TX

// GPS object
TinyGPS gps;

void setup()
{
  Serial.begin(57600);
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
      //      Serial.print(lat.data);
      //      Serial.print(" -- ");
      //      Serial.println(lon.data);

      latPub.publish(&lat);
      delay(1);
      lonPub.publish(&lon);
      delay(1);
    }
    nh.spinOnce();
    delay(1);

  }
  nh.spinOnce();
  delay(1);
}
