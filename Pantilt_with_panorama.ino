#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <ESP32Servo.h>
#include <std_msgs/Int16.h>

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
    if (factor==tilt_factor) loc_speed = 90+(factor+inc_tilt_down_by)*val; //man wanted tilt upwards to be with 22 velocity, but tilt down to be with 17 velocity
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
    if (flag==0)
    {
      servo_pan.write(110);
      delay(100);
      flag=1;
    }
    else
    {
      servo_pan.write(90);
      delay(500);
      flag=0;
    }
    count+=flag;
    panaroma_turning=true;
  }
  else if (count==(imgcount+1)){
    servo_pan.write(80);
    delay(1300);
    flag=2;
    count+=1;
  }
  else{
    flag=0;
    count=0;
    panaroma_turning=false;
  }
}

void cb(const sensor_msgs::Joy& msg)
{
  pan_speed=map_speed(msg.axes[6],pan_factor);
  tilt_speed=map_speed(msg.axes[7],tilt_factor);
  
  if (msg.buttons[11]>0) panaroma_start=true; //button for starting panorama cam
  else panaroma_start=false;
}

ros::NodeHandle nh;
ros::Subscriber<sensor_msgs::Joy> joycontrol("/joy",cb);
std_msgs::Int16 angle;
ros::Publisher servo_control("Servo_Angle",&angle);
void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(servo_control);
  nh.subscribe(joycontrol);
  servo_pan.setPeriodHertz(50);
  servo_tilt.setPeriodHertz(50);
  servo_tilt.attach(48,0,4095); 
  servo_pan.attach(47,0,4095);
  
  delay(5000);
}

void loop()
{
  angle.data=flag;
  if (panaroma_turning or panaroma_start) panaroma(img_count);
  
  else turn(pan_speed,tilt_speed);
  servo_control.publish(&angle);
  nh.spinOnce();
}
