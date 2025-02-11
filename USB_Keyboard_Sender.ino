//D+ is 20, D- is 19

#include "EspUsbHostKeybord.h"
#include <esp_now.h>
#include <WiFi.h>

#define num_motors 8
#define speed_quantization 28
#define str_spee 255
#define how_slow_print 3
#define q_size 5      //for moving avg filter for smoothened motion of rover, set to 1 to remove this feature

uint8_t before_first = 0;
uint8_t before_second = 0;
uint8_t prefix = 0;
uint8_t first = 0;
uint8_t second = 0;

int que[q_size] = {0};
int dir[num_motors] = {0,0,0,0,0,0,0,0};
int spee = speed_quantization;

int str_ind = -1;
int print_counter = 0;

uint8_t broadcastAddress[] = {0xDC, 0xDA, 0x0C, 0x48, 0x97, 0x8C};    //receiver's MAC address    //DC:DA:0C:4B:35:70 for my test S3    //A0:76:4E:93:06:2E for arm S2    //DC:DA:0C:48:97:8C for drive S3
typedef struct struct_message
{
  int pwm[num_motors];
} struct_message;
struct_message myData;  //Structure's object
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  //if (status != ESP_NOW_SEND_SUCCESS) Serial.println("Sending failed.");
  //else Serial.println("Sent successfully.");
  //It prints "Sent successfully" in the callback only if the receiver ESP (with the correct Mac address) is powered, irrespective of the code that the receiver ESP has.
}

class MyEspUsbHostKeybord : public EspUsbHostKeybord 
{
public:
  void onKey(usb_transfer_t *transfer)    //2 messages in one second get printed if print is put here
  {
    uint8_t *const p = transfer->data_buffer;
    prefix = p[0];
    first = p[2];
    second = p[3];
  };
};

MyEspUsbHostKeybord usbHost;

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
  
  usbHost.begin();
}

void loop() 
{
  usbHost.task();

  if (first != 0)
  {
    //drive -> classic WASD
    if (first == 26)
    {
      dir[0] = 1; dir[1] = 1; dir[2] = 1; dir[3] = 1;
    }
    else if (first == 22)
    {
      dir[0] = -1; dir[1] = -1; dir[2] = -1; dir[3] = -1;
    }
    else if (first == 4)
    {
      dir[0] = -1; dir[1] = 1; dir[2] = -1; dir[3] = 1;
    }
    else if (first == 7)
    {
      dir[0] = 1; dir[1] = -1; dir[2] = 1; dir[3] = -1;
    }

    //mode control -> hotkeys from 1 to 9 for proportional speeds upto 252, + and - increase and decrease modes
    if (first>=30 && first <=38) spee = speed_quantization * (first - 29);
    else if (first == 45 && before_first != 45) spee = max(speed_quantization, spee-speed_quantization);
    else if (first == 46 && before_first != 46) spee = min(252, spee+speed_quantization);

    //indiv str -> the extereme 4 keys of extended keyboard is for selecting wheel, right/left arrow keys for direction
    if (first == 95) str_ind = 4;
    else if (first == 97) str_ind = 5;
    else if (first == 89) str_ind = 6;
    else if (first == 91) str_ind = 7;
    if (str_ind != -1) 
    {
      if (second == 80) dir[str_ind] = -1;
      else if (second == 79) dir[str_ind] = 1;

      str_ind = -1;
    }

    //rot in place -> space is rot, shift + space is come back from rot
    if (first == 44)
    {
      if (prefix == 2)
      {
        dir[4] = -1; dir[5] = 1; dir[6] = 1; dir[7] = -1;
      }
      else 
      {
        dir[4] = 1; dir[5] = -1; dir[6] = -1; dir[7] = 1;
      }
    }

    //same and opposite for all steering
    if (first == 80)
    {
      if (prefix == 2)
      {
        dir[4] = -1; dir[5] = -1; dir[6] = 1; dir[7] = 1;
      }
      else
      {
        dir[4] = -1; dir[5] = -1; dir[6] = -1; dir[7] = -1;
      }
    }
    else if (first == 79)
    {
      if (prefix == 2)
      {
        dir[4] = 1; dir[5] = 1; dir[6] = -1; dir[7] = -1;
      }
      else
      {
        dir[4] = 1; dir[5] = 1; dir[6] = 1; dir[7] = 1;
      }
    }
    
  }
  
  if (second != 0)
  {
    //2 drive given at once
    if (second == 26)
    {
      dir[0] += 1; dir[1] += 1; dir[2] += 1; dir[3] += 1;
    }
    else if (second == 22)
    {
      dir[0] += -1; dir[1] += -1; dir[2] += -1; dir[3] += -1;
    }
    else if (second == 4)
    {
      dir[0] += -1; dir[1] += 1; dir[2] += -1; dir[3] += 1;
    }
    else if (second == 7)
    {
      dir[0] += 1; dir[1] += -1; dir[2] += 1; dir[3] += -1;
    }
    else
    {
      dir[0] += 0; dir[1] += 0; dir[2] += 0; dir[3] += 0;
    }

    //mode changing when driving (only 1 drive direction has been allowed)
    if (second>=30 && second <=38) spee = speed_quantization * (second - 29);
    else if (second == 45 && before_second != 45) spee = max(speed_quantization, spee-speed_quantization);
    else if (second == 46 && before_second != 46) spee = min(252, spee+speed_quantization);
  }
  
  before_first = first;
  before_second = second;

  for (int i=0; i<q_size-1; i++) que[i] = que[i+1];     //remove 1st value (left shift all q elements)
  que[q_size] = spee;     //append to q

  spee = 0;     //to find avg pwm
  for (int i=0; i<q_size; i++) spee+=que[i];
  spee = spee/q_size;

  for (int i=0; i<num_motors/2; i++) myData.pwm[i] = dir[i]*spee;
  for (int i=num_motors/2; i<num_motors; i++) myData.pwm[i] = dir[i]*str_spee;

  for (int i=0; i<num_motors; i++) dir[i] = 0;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));     // Send message via ESP-NOW
  
  //Serial.println(spee);
  if (print_counter == 0)
  {
    Serial.print("[");
    for (int i =0; i<num_motors-1; i++)
    {
      Serial.print(myData.pwm[i]);
      Serial.print(", ");
    }
    Serial.print(myData.pwm[num_motors-1]);
    Serial.print("]");
    Serial.println();
  }

  print_counter = (print_counter + 1) % how_slow_print;    //pranav's idea to put % here and not in the if statement to not encounter overflow
  delay(100);
}
