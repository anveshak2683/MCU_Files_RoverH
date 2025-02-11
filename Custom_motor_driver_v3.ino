#include "driver/twai.h"
#include "driver/pcnt.h"
#include "math.h"

#define PCNT_UNIT PCNT_UNIT_0
#define PCNT_H_LIM 32767
#define PCNT_L_LIM -32768

#define ENC_DIR 1
#define ENC_Calib 360/float(562000)

#define RX_PIN 25
#define TX_PIN 26
#define ENCODER_GPIO_A 32
#define ENCODER_GPIO_B 33

#define TRANSMIT_RATE_MS 500
#define POLLING_RATE_MS 500

#define MA 14
#define MB 27
#define PWM 17
#define INA 5
#define INB 4

// Defines for the DIPswitch pins
#define dP0 19
#define dP1 21
#define dP2 22
#define dP3 23

int dipPins[] = {19, 21, 22, 23}; // The pins connected to the DIPswitches in the same order

// Directions of rotation of the wheels depending on dip switch values
// Assumed that there are only 8 motor drivers and  0 <= dipValues <= 7

int16_t count = 0;
volatile int accumulator = 0;
float angles=0;

static bool driver_installed = false;
uint32_t alerts_triggered;
twai_status_info_t twaistatus;
unsigned long previousMillis=0; 
unsigned long currentMillis=0;
int pwm_value=0;

int dipValue = 0; // Variable to hold the state of the DIPswitches
int switch_toggled = 0; // Variable showing that switch was triggered

typedef struct {
  int16_t encoder_angles;
  bool ls1;
  bool ls2;
  float current;
} feedback_t;

feedback_t feedback={0};

void PCNT_Init();
float Get_Angles(volatile int enc_accumulator,int16_t enc_count);
static void IRAM_ATTR pcnt_intr_handler(void *arg);
static void IRAM_ATTR MA_handler();
static void IRAM_ATTR MB_handler();

static void IRAM_ATTR dP_handler();

bool CAN_Init();
static void send_message();
static void handle_rx_message(twai_message_t &message);
void Handle_Errors(uint32_t alerts_triggered,twai_status_info_t twaistatus);

void setup() {
  Serial.begin(115200);
  Serial.println("code started");
  pinMode(INA, OUTPUT);
  pinMode(INB,OUTPUT);
  pinMode(PWM,OUTPUT);
  pinMode(MA,INPUT_PULLDOWN);
  pinMode(MB,INPUT_PULLDOWN);
  
  // Setting up all the DIP switches as INPUT pins
  for(int i = 0; i < 4; i++){
    pinMode(dipPins[i], INPUT_PULLDOWN);
  }
  
  // Attaching interrupts to all the dipPins to monitor the state of the switches
  attachInterrupt(digitalPinToInterrupt(dipPins[0]), dP_handler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(dipPins[1]), dP_handler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(dipPins[2]), dP_handler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(dipPins[3]), dP_handler, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(MA),MA_handler,CHANGE);
  attachInterrupt(digitalPinToInterrupt(MB),MB_handler,CHANGE);

  PCNT_Init();

  driver_installed = CAN_Init();

  Serial.println("setup done");
}

void loop() {

  if (!driver_installed) {
    Serial.println("driver not installed");
    delay(1000);
    return;
  }

  pcnt_get_counter_value(PCNT_UNIT, &count);
  angles=Get_Angles(accumulator,count);
  Serial.printf("Count: %d, Accumulator: %d, Angle: %.2f \n", count, accumulator, angles);

  if (switch_toggled == 1) {
      delay(100);
      dipValue = (digitalRead(dP0) * 1) + (digitalRead(dP1) * 2) + (digitalRead(dP2) * 4) + (digitalRead(dP3) * 8);
  }
    
  Serial.printf("DIP switch state is: %d \n", dipValue);
  
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
  twai_get_status_info(&twaistatus);
  Handle_Errors(alerts_triggered,twaistatus);

  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    twai_message_t message;
    
    while (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK) { 
      handle_rx_message(message);
        
      if (pwm_value>0){
      Serial.println("inside positive loop");
      digitalWrite(INB,LOW);
      digitalWrite(INA,HIGH);
      // analogWrite(PWM,pwm_value);
      analogWrite(PWM,50);
      }
      else if(pwm_value<0){
      Serial.println("inside negative loop");
      digitalWrite(INB,HIGH);
      digitalWrite(INA,LOW);
      // analogWrite(PWM,pwm_value);
      analogWrite(PWM,50);
      }
      else{
      Serial.println("rest");
      digitalWrite(INB,LOW);
      digitalWrite(INA,LOW);
      // analogWrite(PWM,pwm_value);
      analogWrite(PWM,0);
      }
    }
  }
  if (currentMillis - previousMillis >= TRANSMIT_RATE_MS) {
    previousMillis = currentMillis;
    send_message();
  }
  vTaskDelay(pdMS_TO_TICKS(100));
}

void PCNT_Init(){

  pcnt_config_t pcnt_config = {
  .pulse_gpio_num = ENCODER_GPIO_A,  
  .ctrl_gpio_num = ENCODER_GPIO_B, 
  .lctrl_mode = PCNT_MODE_REVERSE,  
  .hctrl_mode = PCNT_MODE_KEEP,     
  .pos_mode = PCNT_COUNT_INC,       
  .neg_mode = PCNT_COUNT_DEC,       
  .counter_h_lim = 32767,           
  .counter_l_lim = -32768,          
  .unit = PCNT_UNIT,                
  .channel = PCNT_CHANNEL_0         
  };

  pcnt_unit_config(&pcnt_config);

  pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
  pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_L_LIM);

  pcnt_isr_service_install(0);
  pcnt_isr_handler_add(PCNT_UNIT, pcnt_intr_handler , NULL);

  pcnt_counter_pause(PCNT_UNIT);
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);

}

float Get_Angles(volatile int enc_accumulator,int16_t enc_count){

    float angle=( enc_accumulator * PCNT_H_LIM + enc_count) * ENC_Calib * ENC_DIR;
    return angle;

}

static void IRAM_ATTR pcnt_intr_handler(void *arg) {
    uint32_t evt_status;
    pcnt_get_event_status(PCNT_UNIT, &evt_status);

    if (evt_status & PCNT_EVT_H_LIM) {
        accumulator ++;  
    }
    if (evt_status & PCNT_EVT_L_LIM) {
        accumulator --;  
    }

}



bool CAN_Init(){

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("Driver installed");
  } 
  else {
    Serial.println("Failed to install driver");
    return false;
  }

    if (twai_start() == ESP_OK) {
      Serial.println("Driver started");
  } else {
      Serial.println("Failed to start driver");
      return false;
    }

  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_TX_FAILED | TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS  ; //last 2 not used
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
      Serial.println("CAN Alerts reconfigured");
  } else {
      Serial.println("Failed to reconfigure alerts");
      return false;
  }

  return true;

}

static void send_message() {
  // Send message

  // Configure message to transmit
  twai_message_t message={0};
  message.extd=0;
  message.identifier = 0x17;
  message.data_length_code = 4;
  feedback.encoder_angles=angles;
  feedback.current=random(0,10);
  feedback.ls1=random(0,2);
  feedback.ls2=random(0,2);
  memcpy(&message.data,&feedback,sizeof(feedback_t));

  // Queue message for transmission
  if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    printf("Message queued for transmission\n");
  } else {
    printf("Failed to queue message for transmission\n");
  }

}

static void handle_rx_message(twai_message_t &message) {
  Serial.printf("ID: %lx\nByte:", message.identifier);
  for (int i = 0; i < message.data_length_code; i++) {
    pwm_value=(message.data[i]-127.5)*2;
    Serial.print("Pwm: ");
    Serial.println(pwm_value);
    }
    Serial.println();
}

void Handle_Errors(uint32_t alerts_triggered,twai_status_info_t twaistatus){

  if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
      Serial.println("Alert: TWAI controller has become error passive.");
  }
  
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
      Serial.println("Alert: A bus error has occurred.");
      Serial.printf("Bus error count: %lu\n", twaistatus.bus_error_count);
  }
  
  if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
      Serial.println("Alert: The RX queue is full causing a received frame to be lost.");
      Serial.printf("RX buffered: %lu\t", twaistatus.msgs_to_rx);
      Serial.printf("RX missed: %lu\t", twaistatus.rx_missed_count);
      Serial.printf("RX overrun %lu\n", twaistatus.rx_overrun_count);
  }

  if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
    Serial.println("Alert: The Transmission failed.");
    Serial.printf("TX buffered: %lu\t", twaistatus.msgs_to_tx);
    Serial.printf("TX error: %lu\t", twaistatus.tx_error_counter);
    Serial.printf("TX failed: %lu\n", twaistatus.tx_failed_count);
  }

}

static void IRAM_ATTR MA_handler(){

  if ((digitalRead(MA)==HIGH) && dipValue == 0){
    digitalWrite(INA,HIGH);
    digitalWrite(INB,LOW);
    analogWrite(PWM,150);
  }
  else if ((digitalRead(MA)==HIGH) && dipValue == 1){
    digitalWrite(INA,LOW);
    digitalWrite(INB,HIGH);
    analogWrite(PWM,150);
  }
  else {
    digitalWrite(INA,LOW);
    digitalWrite(INB,LOW);
    analogWrite(PWM,0);
  }
  
}

static void IRAM_ATTR MB_handler(){

  if ((digitalRead(MB)==HIGH) && dipValue == 0){
    digitalWrite(INA,LOW);
    digitalWrite(INB,HIGH);
    analogWrite(PWM,150);
  }
  else if ((digitalRead(MB)==HIGH) && dipValue == 1){
    digitalWrite(INA,HIGH);
    digitalWrite(INB,LOW);
    analogWrite(PWM,150);
  }
  else{
    digitalWrite(INA,LOW);
    digitalWrite(INB,LOW);
    analogWrite(PWM,0);
  }
  
}

// Interrupt routine reads all the pin values and sets the dip switch state
static void IRAM_ATTR dP_handler(){
  switch_toggled = 1;
}
