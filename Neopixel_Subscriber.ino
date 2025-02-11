#include <ros.h>
#include<std_msgs/Int32.h>
#include <Adafruit_NeoPixel.h>

#define ls_ks 37
int ks_state = 0;
String colour = "None";
bool shown = false;
bool ks_int = true;
#define PIN 20
#define NUMPIXELS 48

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void neo_cb(const std_msgs::Int32 & lightstate)
{
  int f = lightstate.data;
  
  if (f==3) colour = "None";
  else if (f==1) colour = "Green";
  else if (f==2) colour = "Yellow";
  else if (f==0) colour = "Red";
  shown = false;
}

void ks_callback()
{
  ks_int = true;
  shown = false;
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int32> sub("rover_state", neo_cb);

void setup() 
{
  delay(100);
  
  pixels.begin();

  pinMode(ls_ks,INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(ls_ks),ks_callback,CHANGE);
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() 
{
  nh.spinOnce();
  if (shown == false)
  {
    if (ks_int == true)
    {
      delay(100);
      ks_state = digitalRead(ls_ks);
      if (ks_state == 1)
      {
        for(int i=0; i<NUMPIXELS; i++) pixels.setPixelColor(i, pixels.Color(255, 0, 0));  //red   //ks_int not set to false, because I don't want Pratham to talk to neopixel when kill switch is pressed
      }
      else 
      {
        for(int i=0; i<NUMPIXELS; i++) pixels.setPixelColor(i, pixels.Color(0, 255, 0));  //green
        ks_int = false;
      }
    }
    
    else
    {
      if (colour == "None")
      {
        for(int i=0; i<NUMPIXELS; i++) pixels.setPixelColor(i, pixels.Color(0, 0, 0));
      }
      else if (colour == "Green")
      {
        for(int i=0; i<NUMPIXELS; i++) pixels.setPixelColor(i, pixels.Color(0, 255, 0));
      }
      else if (colour == "Yellow")
      {
        for(int i=0; i<NUMPIXELS; i++) pixels.setPixelColor(i, pixels.Color(255, 255, 0));
      }
      else if (colour == "Red")
      {
        for(int i=0; i<NUMPIXELS; i++) pixels.setPixelColor(i, pixels.Color(255, 0, 0));
      }
    }
    
    pixels.show();
    shown = true;
  }
}
