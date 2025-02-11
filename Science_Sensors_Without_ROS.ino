#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>
#include <Adafruit_BMP085_U.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Define pins for sensors
#define DHTPIN 21     // Digital pin connected to the DHT sensor
#define SOIL_MOISTURE_PIN_A 14 // Analog pin connected to the soil moisture sensor
#define SOIL_MOISTURE_PIN_B 27 // Analog pin connected to the soil moisture sensor-2
#define SDA_PIN 32 // I2C SDA pin for BMP180
#define SCL_PIN 33 // I2C SCL pin for BMP180
const int oneWireBus = 35; // GPIO where the DS18B20 is connected to

// Uncomment the type of DHT sensor in use:
#define DHTTYPE DHT11 // DHT 11

// Create sensor instances
DHT_Unified dht(DHTPIN, DHTTYPE);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// Variables for sensor readings
uint32_t dhtDelayMS;
int soilMoisture_a;
int soilMoisture_b;

float sensor_data[9] = {-1,-1,-1,-1,-1,-1,-1,-1,-1};

void setup()
{
  Serial.begin(115200);

  // Initialize DHT sensor
  dht.begin();
  Serial.println("DHT has begun");
  
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dhtDelayMS = sensor.min_delay / 1000;

  // Initialize BMP180 sensor
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("I2C has begun");
  
  if (!bmp.begin())
  {
    for (int i=0;i<50;i++) Serial.println("Could not find a valid BMP180 sensor, check wiring!");
  }

  // Initialize DS18B20 sensor
  sensors.begin();
  Serial.println("DS18B20 has begun");

  pinMode(SOIL_MOISTURE_PIN_A,INPUT);
  pinMode(SOIL_MOISTURE_PIN_B,INPUT);
}

void loop()
{
  // Read DHT sensor data
  delay(dhtDelayMS);
  sensors_event_t dhtEvent;
  
  dht.temperature().getEvent(&dhtEvent);
  if (!isnan(dhtEvent.temperature)) sensor_data[0] = dhtEvent.temperature; // 0--dht_temperature
  else Serial.println("DHT temperature is nan");
  
  dht.humidity().getEvent(&dhtEvent);
  if (!isnan(dhtEvent.relative_humidity)) sensor_data[1] = dhtEvent.relative_humidity; // 1--dht_humidity
  else Serial.println("DHT humidity is nan");

  // Read Soil Moisture sensor data
  soilMoisture_a = analogRead(SOIL_MOISTURE_PIN_A);
  int moisturePercent_a = 100 - (soilMoisture_a / 4095.0 * 100);
  sensor_data[2] = moisturePercent_a;  // 2-- moisture_percent--LM393A

  soilMoisture_b = analogRead(SOIL_MOISTURE_PIN_B);
  int moisturePercent_b = 100 - (soilMoisture_b / 4095.0 * 100);
  sensor_data[3] = moisturePercent_b;  // 8-- moisture_percent_2--LM393B

  // Read BMP180 sensor data
  sensors_event_t bmpEvent;
  bmp.getEvent(&bmpEvent);
  if (bmpEvent.pressure)
  {
    sensor_data[4] = bmpEvent.pressure; // 3--bmp_pressure

    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    sensor_data[5] = bmp.pressureToAltitude(seaLevelPressure, bmpEvent.pressure); //4 -- bmp_altitude

    float temperature;
    bmp.getTemperature(&temperature);
    sensor_data[6] = temperature; //5 --bmp_temperature
  }
  else Serial.println("bpm pressure is wrong");

  // Read DS18B20 sensor data
  sensors.requestTemperatures();
  float temperatureC1 = sensors.getTempCByIndex(0);
  float temperatureC2 = sensors.getTempCByIndex(1);

  sensor_data[7] = temperatureC1; //6 -- DS18B20_temp1
  sensor_data[8] = temperatureC2; //7 -- DS18B20_temp2

  // Delay before next loop
  delay(500);

  for (int i=0;i<9;i++) Serial.println(sensor_data[i]);
  Serial.println();
}
