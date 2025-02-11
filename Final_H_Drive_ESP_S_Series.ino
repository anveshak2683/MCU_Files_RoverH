//Final as of 19.9.24
//Things that may need to be changed in this github (only with hardware changes) - ENC_CALIB and defaultdir[NUM_Motor]
//In the new PCB, 24V is given to steering, which has to be capped using the variable in the python drive codes

#include<ros.h>
#include<std_msgs/Float32MultiArray.h>
#include<std_msgs/Int32MultiArray.h>
#include<std_msgs/Int32.h>

#define NUM_ENC 6
#define NUM_Motor 8
#define ENC_Calib 360/562000

//out of the 8 indices, 1st 4 are drive, other 4 are steering. In the 4, it is front two, followed by back two, LRLR
//depening on the PCB and the ESP (S2/S3), uncomment one of the below 4 and use

//old PCB, ESP S3
//int PWMpin[NUM_Motor] = {18,37,9,35,13,20,11,47};   //47 becomes 33 for S2 and 48 becomes 34
//int dirpin[NUM_Motor] = {8,36,10,48,14,19,12,21};
//old PCB, ESP S2
//int PWMpin[NUM_Motor] = {18,37,9,35,13,20,11,33};
//int dirpin[NUM_Motor] = {8,36,10,34,14,19,12,21};
//new PCB, ESP S3
//int PWMpin[NUM_Motor] = {40,38,36,48,9,18,16,7};
//int dirpin[NUM_Motor] = {41,39,37,35,10,8,17,15};
//new PCB, ESP S2
//int PWMpin[NUM_Motor] = {40,38,36,34,9,18,16,7};
//int dirpin[NUM_Motor] = {41,39,37,35,10,8,17,15};

int defaultdir[NUM_Motor] = {0,1,0,1,0,1,0,1};  // change only when hardware changes are made to rover or to standardize python code
int enc_dir[NUM_ENC] = {1,1,1,1,1,1};  //basically not used

ros::NodeHandle nh;

int drive_buf[NUM_Motor] = {0};
float enc_feed[NUM_ENC] = {0};

void messageCb(const std_msgs::Int32MultiArray & drive_inp)
{
  for(int i=0;i<NUM_Motor;i++)
  {
    drive_buf[i] = drive_inp.data[i];
  }
}

void resetmessageCb(const std_msgs::Int32 & drive_inp)
{
  if(drive_inp.data == 1) ESP.restart();
}

std_msgs::Float32MultiArray enc_msg;
std_msgs::MultiArrayDimension enc_dim;
std_msgs::MultiArrayLayout enc_layout;
ros::Publisher enc_pub("enc_auto", &enc_msg);

ros::Subscriber<std_msgs::Int32MultiArray> sub("motor_pwm", messageCb);
ros::Subscriber<std_msgs::Int32> sub2("esp_reset", resetmessageCb);

volatile long enc_pos[NUM_ENC]={0};
volatile int lastEncoded[NUM_ENC] ={0};

int encA[NUM_ENC] = {5, 7,16, 2,41,39};
int encB[NUM_ENC] = {6,15,17,42,40,38};

void updateEncoder0()
{
  int MSB = digitalRead(encA[0]); //MSB = most significant bit
  int LSB = digitalRead(encB[0]); //LSB = least significant bit
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded[0] << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) enc_pos[0] ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) enc_pos[0] --;
  
  lastEncoded[0] = encoded; //store this value for next time
}

void updateEncoder1()
{
  int MSB = digitalRead(encA[1]); //MSB = most significant bit
  int LSB = digitalRead(encB[1]); //LSB = least significant bit
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded[1] << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) enc_pos[1] --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) enc_pos[1] ++;
  
  lastEncoded[1] = encoded; //store this value for next time
}

void updateEncoder2()
{
  int MSB = digitalRead(encA[2]); //MSB = most significant bit
  int LSB = digitalRead(encB[2]); //LSB = least significant bit
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded[2] << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) enc_pos[2] ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) enc_pos[2] --;
  
  lastEncoded[2] = encoded; //store this value for next time
}

void updateEncoder3()
{
  int MSB = digitalRead(encA[3]); //MSB = most significant bit
  int LSB = digitalRead(encB[3]); //LSB = least significant bit
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded[3] << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) enc_pos[3] --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) enc_pos[3] ++;
  
  lastEncoded[3] = encoded; //store this value for next time
}

void updateEncoder4()
{
  int MSB = digitalRead(encA[4]); //MSB = most significant bit
  int LSB = digitalRead(encB[4]); //LSB = least significant bit
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded[4] << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) enc_pos[4] --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) enc_pos[4] ++;
  
  lastEncoded[4] = encoded; //store this value for next time
}

void updateEncoder5()
{
  int MSB = digitalRead(encA[5]); //MSB = most significant bit
  int LSB = digitalRead(encB[5]); //LSB = least significant bit
  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded[5] << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) enc_pos[5] --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) enc_pos[5] ++;
  
  lastEncoded[5] = encoded; //store this value for next time
}

void setup()
{
  for(int i=0;i<NUM_ENC;i++)
  {
    pinMode(encA[i],INPUT_PULLUP);
    pinMode(encB[i],INPUT_PULLUP);
  }
  for(int i=0;i<NUM_Motor;i++)
  {
    pinMode(PWMpin[i], OUTPUT);
    pinMode(dirpin[i], OUTPUT);
  }
  attachInterrupt(digitalPinToInterrupt(encA[0]),updateEncoder0,CHANGE);    
  attachInterrupt(digitalPinToInterrupt(encA[1]),updateEncoder1,CHANGE);    
  attachInterrupt(digitalPinToInterrupt(encA[2]),updateEncoder2,CHANGE);    
  attachInterrupt(digitalPinToInterrupt(encA[3]),updateEncoder3,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA[4]),updateEncoder4,CHANGE);    
  attachInterrupt(digitalPinToInterrupt(encA[5]),updateEncoder5,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB[0]),updateEncoder0,CHANGE);    
  attachInterrupt(digitalPinToInterrupt(encB[1]),updateEncoder1,CHANGE);    
  attachInterrupt(digitalPinToInterrupt(encB[2]),updateEncoder2,CHANGE);    
  attachInterrupt(digitalPinToInterrupt(encB[3]),updateEncoder3,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB[4]),updateEncoder4,CHANGE);    
  attachInterrupt(digitalPinToInterrupt(encB[5]),updateEncoder5,CHANGE);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(enc_pub);
  nh.subscribe(sub);
  nh.subscribe(sub2);
  enc_msg.data_length = NUM_ENC;
  enc_msg.layout = enc_layout;
}

void loop()
{
  nh.spinOnce();
  
  for(int i=0;i<NUM_Motor;i++)
  {
    if(drive_buf[i]>=0)
      {
        analogWrite(PWMpin[i],min(255,drive_buf[i]));
        digitalWrite(dirpin[i], defaultdir[i]);
      }

    else if(drive_buf[i]<0)
      {
        analogWrite(PWMpin[i],min(255,-drive_buf[i]));
        digitalWrite(dirpin[i], 1-defaultdir[i]);
      }
  }
  
  for(int i=0;i<NUM_ENC;i++) enc_feed[i] = enc_pos[i]*ENC_Calib*enc_dir[i];
  enc_msg.data = enc_feed;
  enc_pub.publish(&enc_msg);
  
  delay(100); 
}
