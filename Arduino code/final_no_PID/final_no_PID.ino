//in navigator bringup
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <rosnano.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

void motor_control(int in1, int in2, int pin, int m, bool dir);  // motor control
void lwheel_callback(const std_msgs::Float32 &lvel);
void rwheel_callback(const std_msgs::Float32 &rvel);
void control_callback(const std_msgs::Int16 &character);
void Lencoder();
void Rencoder();
void gpsCheck();

//Ros Node
ros::NodeHandle  nh;

std_msgs::Int16 lwheelMsg;
std_msgs::Int16 rwheelMsg;
std_msgs::Float32 lat;
std_msgs::Float32 lon;

//Publisher
ros::Publisher lwheelPub("lwheel", &lwheelMsg);
ros::Publisher rwheelPub("rwheel", &rwheelMsg);
ros::Publisher latPub("lat", &lat);
ros::Publisher lonPub("lon", &lon);

// Subscribers
ros::Subscriber<std_msgs::Float32> lwheel_vel("lwheel_vtarget", &lwheel_callback );
ros::Subscriber<std_msgs::Float32> rwheel_vel("rwheel_vtarget", &rwheel_callback );
ros::Subscriber<std_msgs::Int16> intake("intake", &control_callback );

// Soft Serial for gps
SoftwareSerial gpsSerial(8, 9); // RX TX

// GPS object
TinyGPS gps;

// motor pins
#define m1 A2
#define m2 A3
#define m3 A1
#define m4 A0
#define e1 5
#define e2 6

// Encoder pins
#define lencoder 3
#define rencoder 2

// Direction of wheels (true = forward && false = backward)
bool lstat = true;
bool rstat = true;

// motor speeds
uint8_t l = 128;
uint8_t r = 128;

// Keep track of the number of encoder ticks
volatile int32_t lwheel_encoder = 0;
volatile int32_t rwheel_encoder = 0;
int lprev = LOW;
int rprev = LOW;
int lcur = LOW;
int rcur = LOW;

void motor_control(int in1, int in2, int pin, int m, bool dir)  // motor control
{
  digitalWrite(in1, !dir);
  digitalWrite(in2, dir);
  analogWrite(pin, m);
}

void lwheel_callback(const std_msgs::Float32 &lvel) // left wheel Subscriber callback
{
  Serial.println(lvel.data);
  lstat = lvel.data >= 0;
  motor_control(m1, m2, e1, lvel.data, lstat);
}

void rwheel_callback(const std_msgs::Float32 &rvel) // right wheel Subscriber callback
{
  Serial.println(rvel.data);
  rstat = rvel.data >= 0;
  motor_control(m3, m4, e2, rvel.data, rstat);
}

void Lencoder()
{
  lcur = digitalRead(lencoder);
  if (lprev == LOW and lcur == HIGH)
  {
    lwheelMsg.data = lstat ? ++lwheel_encoder : --lwheel_encoder;
    lwheelPub.publish(&lwheelMsg);
  }
  lprev = lcur;
}

void Rencoder()
{
  rcur = digitalRead(rencoder);
  if (rprev == LOW and rcur == HIGH)
  {
    rwheelMsg.data = rstat ? ++rwheel_encoder : --rwheel_encoder;
    rwheelPub.publish(&rwheelMsg);
  }
  rprev = rcur;
}

void control_callback(const std_msgs::Int16 &character)
{
  switch (character.data)
  {
    case 0:
      digitalWrite(m1, LOW);
      digitalWrite(m2, LOW);
      digitalWrite(m3, LOW);
      digitalWrite(m4, LOW);
      break;
    case 1: // front
      lstat = true;
      rstat = true;
      l = 128;
      r = 128;
      digitalWrite(m1, LOW);
      digitalWrite(m2, HIGH);
      digitalWrite(m3, LOW);
      digitalWrite(m4, HIGH);
      break;
    case 2: //bac
      lstat = false;
      rstat = false;
      l = 128;
      r = 128;
      digitalWrite(m1, HIGH);
      digitalWrite(m2, LOW);
      digitalWrite(m3, HIGH);
      digitalWrite(m4, LOW);
      break;
    case 3: // right
      lstat = true;
      rstat = false;
      l = 95;
      r = 95;
      digitalWrite(m1, LOW);
      digitalWrite(m2, HIGH);
      digitalWrite(m3, HIGH);
      digitalWrite(m4, LOW);
      break;
    case 4: //left
      lstat = false;
      rstat = true;
      l = 95;
      r = 95;
      digitalWrite(m1, HIGH);
      digitalWrite(m2, LOW);
      digitalWrite(m3, LOW);
      digitalWrite(m4, HIGH);
      break;
  }
  analogWrite(e1, l);
  analogWrite(e2, r);
}

void gpsCheck()
{
  if (gpsSerial.available()) // check for gps data
  {
    if (gps.encode(gpsSerial.read()))
    {
      gps.f_get_position(&lat.data, &lon.data); // get latitude and longitude
      latPub.publish(&lat);
      delay(1);
      lonPub.publish(&lon);
      delay(1);
    }
  }
}

void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(lwheelPub);
  nh.advertise(rwheelPub);
  nh.advertise(latPub);
  nh.advertise(lonPub);
  nh.subscribe(lwheel_vel);
  nh.subscribe(rwheel_vel);
  nh.subscribe(intake);
  gpsSerial.begin(9600);
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(m3, OUTPUT);
  pinMode(m4, OUTPUT);
  pinMode(e1, OUTPUT);
  pinMode(e2, OUTPUT);
  pinMode(lencoder, INPUT);
  pinMode(rencoder, INPUT);
}

void loop()
{
  Rencoder();
  Lencoder();
  gpsCheck();
  nh.spinOnce();
  delay(1);
}
