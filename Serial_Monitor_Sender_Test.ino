#include <esp_now.h>
#include <WiFi.h>

#define pwm_len 8

int dir[pwm_len] = {0};
int dire = 0;
int spee = 0;
int tim = 0;

uint8_t broadcastAddress[] = {0xDC, 0xDA, 0x0C, 0x48, 0x97, 0x8C};    //receiver's MAC address    //DC:DA:0C:4B:35:70 for my test S3    //A0:76:4E:93:06:2E for arm S2    //DC:DA:0C:48:97:8C for drive S3

typedef struct struct_message
{
  int pwm[pwm_len];
  int tim_ms;
} struct_message;

struct_message myData;  //Structure's object

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  if (status == ESP_NOW_SEND_SUCCESS) Serial.println("Sent successfully.");
  else Serial.println("Sending failed.");

  Serial.print("[");
  for (int i =0; i<pwm_len-1; i++)
  {
    Serial.print(myData.pwm[i]);
    Serial.print(", ");
  }
  Serial.print(myData.pwm[pwm_len-1]);
  Serial.print("] for time ");
  Serial.println(myData.tim_ms);
}

void setup()
{
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);  // Set device as a Wi-Fi Station

  while (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    delay(1000);
  }
  esp_now_register_send_cb(OnDataSent);   //declaring the callback for when the message is sent
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);    // Register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  while (esp_now_add_peer(&peerInfo) != ESP_OK)  // Add peer
  {
    Serial.println("Failed to add peer");
    delay(1000);
  }
}

void loop()
{
  Serial.println();
  Serial.print("Enter direction (1 for Front, 2 for Back, 3 for Left or 4 for Right):- ");
  while (Serial.available()==0) {};
  dire = Serial.parseInt();
  Serial.parseInt();  //a stray 0 was being read after every Serial.parseInt()
  if (dire == 1)
  {
    dir[0]=1; dir[1]=1; dir[2]=1; dir[3]=1;
  }
  else if (dire == 2)
  {
    dir[0]=-1; dir[1]=-1; dir[2]=-1; dir[3]=-1;
  }
  else if (dire == 3)
  {
    dir[0]=-1; dir[1]=1; dir[2]=-1; dir[3]=1;
  }
  else if (dire == 4)
  {
    dir[0]=1; dir[1]=-1; dir[2]=1; dir[3]=-1;
  }
  else
  {
    dir[0]=0; dir[1]=0; dir[2]=0; dir[3]=0;
  }
  Serial.println(dire);

  Serial.print("Enter speed (from 0 to 255):- ");
  while (Serial.available()==0) {};
  spee = Serial.parseInt();
  Serial.parseInt();
  if (spee<0 || spee>255) spee = 0;
  Serial.println(spee);

  Serial.print("Enter time in ms (from 0 to 5000):- ");
  while (Serial.available()==0) {};
  tim = Serial.parseInt();
  Serial.parseInt();
  if (tim<0 || tim>5000) tim = 0;
  Serial.println(tim);
  
  for (int i=0;i<pwm_len;i++) myData.pwm[i] = dir[i]*spee;
  myData.tim_ms = tim;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));     // Send message via ESP-NOW

  if (result == ESP_OK) Serial.println("Sent with success");
  else Serial.println("Error sending the data");

  delay(2000);
}
