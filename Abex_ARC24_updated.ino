#include <ros.h>
#include<std_msgs/Float32MultiArray.h>
#include<std_msgs/Int32MultiArray.h>

#include <AccelStepper.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>
#include <Adafruit_BMP085_U.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Define pins for sensors
#define DHTPIN 34     // Digital pin connected to the DHT sensor
#define SOIL_MOISTURE_PIN_A 14 // Analog pin connected to the soil moisture sensor
#define SOIL_MOISTURE_PIN_B 27 // Analog pin connected to the soil moisture sensor-2
#define SDA_PIN 32 // I2C SDA pin for BMP180
#define SCL_PIN 33 // I2C SCL pin for BMP180
const int oneWireBus = 13; // GPIO where the DS18B20 is connected to

//pins for motors
#define ENCA 25
#define ENCB 26

#define DR11 23
#define DR12 22
#define DR22 19
#define DR21 18

#define John1 16
#define John2 4

#define STEP 2
#define DIR 15

#define DHTTYPE DHT11

// Create sensor instances
DHT_Unified dht(DHTPIN, DHTTYPE);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
AccelStepper stepper(AccelStepper::DRIVER, STEP, DIR);

// Variables for sensor readings
uint32_t dhtDelayMS;
int soilMoisture_a;
int soilMoisture_b;

//variables for motors
volatile long encoderCount = 0;
const int stepsPerRevolution = 25;
const int gearRatio = 1620;
const long stepsPerOutputRevolution = stepsPerRevolution * gearRatio;

float init_list[9] = {-1,25,-1,-1,-1,-1,-1,-1,-1};
volatile float sciencemotors[3]={0};

void messageCb(const std_msgs::Int32MultiArray & gui_inp)
{
  for(int i=0;i<4;i++) sciencemotors[i] = gui_inp.data[i];
}

ros::NodeHandle nh;

std_msgs::Float32MultiArray sensor_data;
ros::Publisher sensor_pub("sensor_data", &sensor_data);
ros::Subscriber<std_msgs::Int32MultiArray> sub("motor_data_science", messageCb);

void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(sensor_pub);

  dht.begin();

  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dhtDelayMS = sensor.min_delay / 1000;

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!bmp.begin())
  {
    for (int i=0;i<50;i++) Serial.println("Could not find a valid BMP180 sensor, check wiring!");
  }

  // Initialize DS18B20 sensor
  sensors.begin();

  // Reserve space for sensor data array
  sensor_data.data_length = 9; // Number of sensor readings
  sensor_data.data = init_list;

  pinMode(SOIL_MOISTURE_PIN_A,INPUT);
  pinMode(SOIL_MOISTURE_PIN_B,INPUT);

  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  
  pinMode(DR11, OUTPUT);
  pinMode(DR12, OUTPUT);
  pinMode(DR21, OUTPUT);
  pinMode(DR22, OUTPUT);
  
  pinMode(John1, OUTPUT);
  pinMode(John2, OUTPUT);
  
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  digitalWrite(DIR, HIGH); // Set initial direction
  stepper.setMaxSpeed(20); // Adjust as needed
  stepper.setAcceleration(10); // Adjust as needed

  attachInterrupt(digitalPinToInterrupt(ENCA), encoderISR, CHANGE);
}

void loop()
{
  nh.spinOnce();

  if (sciencemotors[0]!=0)
  {
    rotateMotor(-90);
    sciencemotors[0] = 0;
  }

  if (sciencemotors[1]!=0) 
  {
    runpumps(sciencemotors[1]);
    sciencemotors[1] = 0;
  }
  
  if (sciencemotors[2] == 1) 
  {
    movestepper(90);
    sciencemotors[2] = 0;
  }
  else if (sciencemotors[2] == 2)
  {
    movestepper(-90);
    sciencemotors[2] = 0;
  }

  if (sciencemotors[3] == 1)
  {
    delay(dhtDelayMS);
    sensors_event_t dhtEvent;
    dht.temperature().getEvent(&dhtEvent);
    if (!isnan(dhtEvent.temperature))
    {
      sensor_data.data[0] = dhtEvent.temperature; // 0--dht_temperature
    }
    
    dht.humidity().getEvent(&dhtEvent);
    if (!isnan(dhtEvent.relative_humidity))
    {
      sensor_data.data[1] = dhtEvent.relative_humidity; // 1--dht_humidity
    }
  
    // Read Soil Moisture sensor data
    soilMoisture_a = analogRead(SOIL_MOISTURE_PIN_A);
    int moisturePercent_a = 100 - (soilMoisture_a / 4095.0 * 100);
    sensor_data.data[2] = moisturePercent_a;  // 2-- moisture_percent--LM393A
  
    soilMoisture_b = analogRead(SOIL_MOISTURE_PIN_B);
    int moisturePercent_b = 100 - (soilMoisture_b / 4095.0 * 100);
    sensor_data.data[3] = moisturePercent_b;  // 8-- moisture_percent_2--LM393B
  
    // Read BMP180 sensor data
    sensors_event_t bmpEvent;
    bmp.getEvent(&bmpEvent);
    if (bmpEvent.pressure)
    {
      sensor_data.data[4] = bmpEvent.pressure; // 3--bmp_pressure
  
      float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
      sensor_data.data[5] = bmp.pressureToAltitude(seaLevelPressure, bmpEvent.pressure); //4 -- bmp_altitude
  
      float temperature;
      bmp.getTemperature(&temperature);
      sensor_data.data[6] = temperature; //5 --bmp_temperature
    }
  
    // Read DS18B20 sensor data
    sensors.requestTemperatures();
    float temperatureC1 = sensors.getTempCByIndex(0);
    float temperatureC2 = sensors.getTempCByIndex(1);
  
    sensor_data.data[7] = temperatureC1; //6 -- DS18B20_temp1
    sensor_data.data[8] = temperatureC2; //7 -- DS18B20_temp2
  
    // Publish sensor data
    sensor_pub.publish(&sensor_data);

    sciencemotors[3] = 0;
  }

  delay(50); 
}

void movestepper(int angle)
{
  long steps = (200 / 360.0) * abs(angle); // Assuming 200 steps per revolution and 1.8 degrees per step
  stepper.moveTo(stepper.currentPosition() + steps);
  stepper.runToPosition();
}

void runpumps(int a)
{
  if(a==1)
  {
    digitalWrite(DR12,HIGH);
    delay(8000);
    digitalWrite(DR12,LOW);
    return;
  }
  else if(a==2)
  {
    digitalWrite(DR21,HIGH);
    delay(8000);
    digitalWrite(DR21,LOW);
    return;
  }
  else if(a==3)
  {
    digitalWrite(DR12,HIGH);
    delay(2000);
    digitalWrite(DR12,LOW);
    return;
  }
  else if(a==4)
  {
    digitalWrite(DR21,HIGH);
    delay(2000);
    digitalWrite(DR21,LOW);
    return;
  }
}

void rotateMotor(int angle)
{
  long steps = (stepsPerOutputRevolution / 360) * abs(angle);
  encoderCount = 0;

  if (angle > 0) 
  {
    digitalWrite(John1, HIGH);
    digitalWrite(John2, LOW);
  } 
  else 
  {
    digitalWrite(John1, LOW);
    digitalWrite(John2, HIGH);
  }

  while (abs(encoderCount) < steps) delay(10); // Adjust this delay to control the motor speed    //The motor will be rotating even in this while loop, because the pin will be high, and it even will go into the ISR, because it interrupts this

  // Stop the motor
  digitalWrite(John1, LOW);
  digitalWrite(John2, LOW);
}

void encoderISR()
{
  int stateA = digitalRead(ENCA);
  int stateB = digitalRead(ENCB);

  if (stateA == stateB) encoderCount++;
  else encoderCount--;
}
