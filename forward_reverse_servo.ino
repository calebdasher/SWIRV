#include <driver/ledc.h>
#include <esp_now.h>
#include <WiFi.h>

// receiver mac address 40:4C:CA:4A:AA:F4
uint8_t des_addr[] = {0x40, 0x4C, 0xCA, 0x4A, 0xAA, 0xF4};
// sender mac address 40:4C:CA:4A:AA:F0
uint8_t src_addr[] = {0x40, 0x4C, 0xCA, 0x4A, 0xAA, 0xF0};
struct esp_now_recv_info recv_info;
// Structure example to receive data
// Must match the sender structure
typedef struct recv_move_command{
  int left;
  int right;
} recv_move_command;
recv_move_command message;
// Create a struct_message called myData
recv_move_command myData;

#define left_motor  3  // pin 3
#define right_motor 4  // pin 4
#define PWM_Res     16 // 16 bit, 65536 values
#define PWM_Freq    50 // 50Hz

int L_sig, R_sig;
int time_delta, old_time;
int test_l, test_r;
// callback function that will be executed when data is received
void OnDataRecv(const esp_now_recv_info_t * rcv_info, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  //Serial.print("\nBytes received: \n");     
  //Serial.print("\nLeft: ");
  //Serial.print(myData.left);
  //Serial.print("\tRight: ");
  //Serial.print(myData.right);
  L_sig = map(myData.left, -999, 1000, 3277, 6554);
  R_sig = map(myData.right, -999, 1000, 3277, 6554);
  test_l = ledcRead(3);
  test_r = ledcRead(4);
  Serial.printf("Left: %d, Right: %d\n", test_l, test_r);
  ledcWrite(left_motor, L_sig);
  ledcWrite(right_motor, R_sig);
  time_delta += 1;
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Set up the PWM Control for the ESC's
  ledcAttach(left_motor, PWM_Freq, PWM_Res);
  ledcAttach(right_motor, PWM_Freq, PWM_Res);

  time_delta = 0;
  old_time = 0;
  // Init ESP-NOW
  recv_info.src_addr = src_addr;
  recv_info.des_addr = des_addr;
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}
void loop() {
  if (time_delta == old_time){
    ledcWrite(left_motor, 4915); // 75% duty cycle for 11 bit number to idle
    ledcWrite(right_motor, 4915);
    Serial.println("Idling");
  }
  old_time = time_delta;
  delay(500);
}
