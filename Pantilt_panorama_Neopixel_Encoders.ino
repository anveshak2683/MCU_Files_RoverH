#include <ros.h>
#include<std_msgs/Int32.h>
#include<std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Joy.h>
#include <ESP32Servo.h>

ros::NodeHandle nh;

#define pan_factor 13
#define tilt_factor 15
#define inc_tilt_down_by 5
#define img_count 12

float pan_speed=90;
float tilt_speed=90;
int state=0;
int count=0;
int flag=0;
bool panaroma_turning=false;
bool panaroma_start=false;
Servo servo_pan;
Servo servo_tilt;

float map_speed(float val,float factor)
{
  float loc_speed = 90;
  if(val<0) loc_speed = 90+factor*(val); 
  else
  {
    if (factor==tilt_factor) loc_speed = 90+(factor+inc_tilt_down_by)*val;   //man wanted tilt upwards to be with 22 velocity, but tilt down to be with 17 velocity
    else loc_speed=90+factor*val;
  }
  return loc_speed;
}

void turn(float pan_speed,float tilt_speed)
{
  servo_pan.write(pan_speed);
  servo_tilt.write(tilt_speed);
  delay(10);
}

void panaroma(int imgcount)
{
  if (count<=imgcount)
  {
    if (flag%2==1)
    {
      servo_pan.write(110);
      delay(100);
      flag=0;
    }
    else
    {
      flag=1;
      servo_pan.write(90);
      delay(2000);
    }
    count+=flag;
    panaroma_turning=true;
  }
  else panaroma_turning=false;
}

void cb(const sensor_msgs::Joy& msg)
{
  pan_speed=map_speed(msg.axes[6],pan_factor);
  tilt_speed=map_speed(msg.axes[7],tilt_factor);
  
  if (msg.buttons[11]>0) panaroma_start=true;  //button for starting panorama cam
  else panaroma_start=false;
}

#include <Adafruit_NeoPixel.h>
#define PIN 27
#define NUMPIXELS 48
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#define num_enc 4
int encA[num_enc] = {7, 10, 42, 47};
int encB[num_enc] = {6, 11, 41, 48};
float enc_feed[num_enc] = {0};
volatile bool A_set[num_enc] = {false};
volatile bool B_set[num_enc] = {false};
volatile long enc_pos[num_enc] = {0};

void messageCb(const std_msgs::Int32 & lightstate)
{
  int f = lightstate.data;
  
  if(f==0)
  {
    for(int i=0; i<NUMPIXELS; i++) pixels.setPixelColor(i, pixels.Color(255, 0, 0));
    pixels.show();
    return;
  }
  
  else if(f==1)
  {
    for(int i=0; i<NUMPIXELS; i++) pixels.setPixelColor(i, pixels.Color(0, 255, 0));
    pixels.show();
    return;
  }
  
  else if(f==2)
  {
    for(int i=0; i<NUMPIXELS; i++) pixels.setPixelColor(i, pixels.Color(255, 255, 0));
    pixels.show();
    return;
  }
}

void callback_0(){
    int i=0;
    A_set[i]=digitalRead(encA[i]);
    B_set[i]=digitalRead(encB[i]);
    if(A_set[i]==B_set[i]) enc_pos[i]++;
    else enc_pos[i]--;
}
void callback_1(){
    int i=1;
    A_set[i]=digitalRead(encA[i]);
    B_set[i]=digitalRead(encB[i]);
    if(A_set[i]==B_set[i]) enc_pos[i]++;
    else enc_pos[i]--;
}
 
void callback_2(){
    int i=2;
    A_set[i]=digitalRead(encA[i]);
    B_set[i]=digitalRead(encB[i]);
    if(A_set[i]==B_set[i]) enc_pos[i]++;
    else enc_pos[i]--;
}
 
void callback_3(){
    int i=3;
    A_set[i]=digitalRead(encA[i]);
    B_set[i]=digitalRead(encB[i]);
    if(A_set[i]==B_set[i]) enc_pos[i]++;
    else enc_pos[i]--;
}

ros::Subscriber<std_msgs::Int32> sub("rover_state", messageCb);
std_msgs::Float32MultiArray enc_msg;
std_msgs::MultiArrayLayout enc_layout;
ros::Publisher enc_pub("enc_ls", &enc_msg);
ros::Subscriber<sensor_msgs::Joy> joycontrol("/joy_arm",cb);

void setup() 
{
  pixels.begin();

  for(int i=0; i<num_enc; i++)
  {
    pinMode(encA[i],INPUT_PULLUP);
    pinMode(encB[i],INPUT_PULLUP);
  }

  attachInterrupt(digitalPinToInterrupt(encA[0]),callback_0,RISING);
  attachInterrupt(digitalPinToInterrupt(encA[1]),callback_1,RISING);
  attachInterrupt(digitalPinToInterrupt(encA[2]),callback_2,RISING);
  attachInterrupt(digitalPinToInterrupt(encA[3]),callback_3,RISING);

  servo_pan.setPeriodHertz(50);
  servo_tilt.setPeriodHertz(50);
  servo_tilt.attach(48,0,4095); 
  servo_pan.attach(41,0,4095);
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(joycontrol);
  nh.advertise(enc_pub);
  enc_msg.data_length = num_enc;
  enc_msg.layout = enc_layout;

  delay(5000);
}

void loop() 
{
  nh.spinOnce();

  if (panaroma_turning or panaroma_start) panaroma(img_count);
  
  else turn(pan_speed,tilt_speed);
  
  for(int i=0; i<num_enc; i++) enc_feed[i] = (enc_pos[i]*360/4800);
  enc_msg.data = enc_feed;
  enc_pub.publish(&enc_msg);
  delay(100);
}
