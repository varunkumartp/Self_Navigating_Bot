//in navigator bringup
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <rosnano.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

void lwheel_callback(const std_msgs::Float32 &lvel);
void rwheel_callback(const std_msgs::Float32 &rvel);
void control_callback(const std_msgs::Int16 &character);
void Lencoder();
void Rencoder();
void motor_control(int in1, int in2, bool m, bool dir);
void sendPulse(int pin, int pwm_pulse);
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

// PID constants
#define kp 1.55
#define kd 0.05
#define ki 1.5

// Timer interrupt delay
#define DELAY1 1

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

// motors status
bool stat = false;

// motor speeds
uint8_t l = 128;
uint8_t r = 128;

// Keep track of the number of encoder ticks
volatile long lwheel = 0;
volatile long rwheel = 0;
volatile uint32_t lwheel_encoder = 0;
volatile uint32_t rwheel_encoder = 0;
int lprev = LOW;
int rprev = LOW;
int lcur = LOW;
int rcur = LOW;

// RPM of wheels
double lpv_speed = 0;
double rpv_speed = 0;

//PID variables
double le_speed = 0; //error of speed = set_speed - pv_speed
double re_speed = 0; //error of speed = set_speed - pv_speed
double le_speed_pre = 0;  //last error of speed
double re_speed_pre = 0;  //last error of speed
double le_speed_sum = 0;  //sum error of speed
double re_speed_sum = 0;  //sum error of speed
double lpwm_pulse = 0;
double rpwm_pulse = 0;
double lset_speed = 50;
double rset_speed = 50;

void motor_control(int in1, int in2, bool m, bool dir)  // motor control
{
  if (m)
  {
    if (dir)
    {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }
    else
    {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    }
    stat = true;
  }
  else
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    stat = false;
  }
}

void sendPulse(int pin, int pwm_pulse)  // pwm function
{
  if (pwm_pulse < 255 and pwm_pulse > 0)
    analogWrite(pin, pwm_pulse);
  else if (pwm_pulse > 255)
    analogWrite(pin, 255);
  else
    analogWrite(pin, 128);
}

void lwheel_callback(const std_msgs::Float32 &lvel) // left wheel Subscriber callback
{
  lset_speed = lvel.data;
  lstat = lvel.data >= 0 ? true : false;
  motor_control(m1, m2, lset_speed != 0 ? true : false, lstat);
}

void rwheel_callback(const std_msgs::Float32 &rvel) // right wheel Subscriber callback
{
  rset_speed = rvel.data;
  rstat = rvel.data >= 0 ? true : false;
  motor_control(m3, m4, rset_speed != 0 ? true : false, rstat);
}

void Lencoder() // Interrupt Service Routine left wheel
{
  ++lwheel;
  lwheelMsg.data = lstat ? ++lwheel_encoder : --lwheel_encoder;
  lwheelPub.publish(&lwheelMsg);
}

void Rencoder() // Interrupt Service Routine right wheel
{
  ++rwheel;
  rwheelMsg.data = rstat ? ++rwheel_encoder : --rwheel_encoder;
  rwheelPub.publish(&rwheelMsg);
}

ISR(TIMER1_COMPA_vect)  // timer interrupt
{
  lpv_speed = lwheel * 3 / DELAY1;
  rpv_speed = rwheel * 3 / DELAY1;
  lwheel = 0;
  rwheel = 0;
  if (stat)
  {
    le_speed = lset_speed - lpv_speed;
    re_speed = rset_speed - rpv_speed;

    lpwm_pulse = le_speed * kp + le_speed_sum * ki + (le_speed - le_speed_pre) * kd;
    rpwm_pulse = re_speed * kp + re_speed_sum * ki + (re_speed - re_speed_pre) * kd;

    le_speed_pre = le_speed;  //save last (previous) error
    re_speed_pre = re_speed;  //save last (previous) error

    le_speed_sum += le_speed; //sum of error
    re_speed_sum += re_speed; //sum of error

    if (le_speed_sum > 400)
      le_speed_sum = 400;
    if (le_speed_sum < -400)
      le_speed_sum = -400;
    if (re_speed_sum > 400)
      re_speed_sum = 400;
    if (re_speed_sum < -400)
      re_speed_sum = -400;
  }
  else
  {
    le_speed = 0;
    re_speed = 0;
    le_speed_pre = 0;
    re_speed_pre = 0;
    le_speed_sum = 0;
    re_speed_sum = 0;
    lpwm_pulse = 0;
    rpwm_pulse = 0;
  }
  sendPulse(e2, lpwm_pulse);
  sendPulse(e1, rpwm_pulse);
}

void control_callback(const std_msgs::Int16 &character)
{
  switch (character.data)
  {
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
    case 0:
      digitalWrite(m1, LOW);
      digitalWrite(m2, LOW);
      digitalWrite(m3, LOW);
      digitalWrite(m4, LOW);
      break;
  }
  analogWrite(e1, l);
  analogWrite(e2, r);
}


void setup()
{
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
  noInterrupts();           // disable all interrupts                 //--------------------------timer setup
  // timer 1 - 1Hz
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; //initialize counter value to 0
  OCR1A = ((long)(16e6) * DELAY1 / 1024) - 1;  // = (16*10^6) / (1*1024) - 1 (must be <65536)
  TCCR1B |= (1 << WGM12); // turn on CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10);  // Set CS12 and CS10 bits for 1024 prescaler
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();           // enable all interrupts                 //--------------------------timer setup
}

void loop()
{
  analogWrite(e1, l);
  analogWrite(e2, r);
  if (gpsSerial.available()) // check for gps data
  {
    if (gps.encode(gpsSerial.read()))
    {
      gps.f_get_position(&lat.data, &lon.data); // get latitude and longitude
    }
  }
  latPub.publish(&lat);
  delay(1);
  lonPub.publish(&lon);
  delay(1);
  rcur = digitalRead(rencoder);
  if (rprev == LOW and rcur == HIGH)
  {
    Rencoder();
  }
  rprev = rcur;
  lcur = digitalRead(lencoder);
  if (lprev == LOW and lcur == HIGH)
  {
    Lencoder();
  }
  lprev = lcur;
  nh.spinOnce();
  delay(1);
}
