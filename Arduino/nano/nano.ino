// final nano
// An arduino mega can be used and both the uno and nano can be combined to form a single code 
#include <Wire.h>
#include <MechaQMC5883.h>
#include <rosnano.h>
#include <std_msgs/Int16.h>

void lwheel_callback(const std_msgs::Int16 &lvel);
void rwheel_callback(const std_msgs::Int16&rvel);
void lwheel_pwm_callback(const std_msgs::Int16 &lpwm);
void rwheel_pwm_callback(const std_msgs::Int16 &rpwm);
void Lencoder();
void Rencoder();
void sendPulse(int pin, int pwm_pulse);


//Ros Node
ros::NodeHandle  nh;

std_msgs::Int16 lwheelMsg;
std_msgs::Int16 rwheelMsg;
std_msgs::Float32 yaw;

//Publisher buffer 200 bytes
ros::Publisher lwheelPub("encoder_ticks/lwheel", &lwheelMsg);
ros::Publisher rwheelPub("encoder_ticks/rwheel", &rwheelMsg);
ros::Publisher headingPub("imu/yaw", &yaw);

// Subscribers buffer 200 bytes
ros::Subscriber<std_msgs::Int16> lwheel_vel("motors/vtarget/lwheel", &lwheel_callback );
ros::Subscriber<std_msgs::Int16> rwheel_vel("motors/vtarget/rwheel", &rwheel_callback );
ros::Subscriber<std_msgs::Int16> lwheel_pwm("motors/pwm/lwheel", &lwheel_pwm_callback );
ros::Subscriber<std_msgs::Int16> rwheel_pwm("motors/pwm/rwheel", &rwheel_pwm_callback );

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

// magnetometer object
MechaQMC5883  qmc;

// Direction of wheels (true = forward && false = backward)
bool lstat = true;
bool rstat = true;

// motors status
bool stat = false;

// Keep track of the number of encoder ticks
volatile long lwheel = 0;
volatile long rwheel = 0;
volatile uint32_t lwheel_encoder = 0;
volatile uint32_t rwheel_encoder = 0;

// status of encoder
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
uint8_t lset_speed = 50;
uint8_t rset_speed = 50;

void sendPulse(int pin, int pwm_pulse)  // pwm function
{
  if (pwm_pulse < 128 and pwm_pulse > 0)
    analogWrite(pin, pwm_pulse);
  else if (pwm_pulse > 128)
    analogWrite(pin, 128);
  else
    analogWrite(pin, 0);
}

void lwheel_callback(const std_msgs::Int16 &lvel)
{
  lset_speed = lvel.data;
  lstat = lvel.data >= 0;
  stat = lset_speed != 0;
  digitalWrite(m1, stat && !lstat);
  digitalWrite(m2, stat && lstat);
}

void rwheel_callback(const std_msgs::Int16  &rvel)
{
  rset_speed = rvel.data;
  rstat = rvel.data >= 0;
  stat = rset_speed != 0;
  digitalWrite(m3, stat && !rstat);
  digitalWrite(m4, stat && rstat);
}


void lwheel_pwm_callback(const std_msgs::Int16 &lpwm)
{
  lstat = lpwm.data >= 0;
  digitalWrite(m1, !lstat);
  digitalWrite(m2, lstat);
  analogWrite(e2, lpwm.data);
  stat = false;
}

void rwheel_pwm_callback(const std_msgs::Int16 &rpwm)
{
  rstat = rpwm.data >= 0;
  digitalWrite(m3, !rstat);
  digitalWrite(m4, rstat);
  analogWrite(e1, rpwm.data);
  stat = false;
}

void Lencoder()
{
  ++lwheel;
  lwheelMsg.data = lstat ? ++lwheel_encoder : --lwheel_encoder;
  lwheelPub.publish(&lwheelMsg);
}

void Rencoder()
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

    sendPulse(e2, le_speed * kp + le_speed_sum * ki + (le_speed - le_speed_pre) * kd);
    sendPulse(e1, re_speed * kp + re_speed_sum * ki + (re_speed - re_speed_pre) * kd);

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
  }
}

void getHeading()
{
  int x, y, z;
  qmc.read(&x, &y, &z);
  yaw.data = atan2(x,y);
  headingPub.publish(&yaw);
}

void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(lwheelPub);
  nh.advertise(rwheelPub);
  nh.subscribe(lwheel_vel);
  nh.subscribe(rwheel_vel);
  nh.subscribe(lwheel_pwm);
  nh.subscribe(rwheel_pwm);
  Wire.begin();
  qmc.init();
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
  lcur = digitalRead(lencoder);
  if (lprev == LOW and lcur == HIGH)
  {
    Lencoder();
  }
  lprev = lcur;
  rcur = digitalRead(rencoder);
  if (rprev == LOW and rcur == HIGH)
  {
    Rencoder();
  }
  rprev = rcur;
  getHeading();
  nh.spinOnce();
  delay(1);
}
