#include <WiFi.h>
#include <esp_now.h>
// receiver mac address 40:4C:CA:4A:AA:F4
uint8_t des_addr[] = {0x40, 0x4C, 0xCA, 0x4A, 0xAA, 0xF4};
// sender mac address 40:4C:CA:4A:AA:F0
uint8_t src_addr[] = {0x40, 0x4C, 0xCA, 0x4A, 0xAA, 0xF0};
struct esp_now_recv_info recv_info;

typedef struct send_move_command{
  int left;
  int right;
  int wifi_status;
  int control_status;
} send_move_command;
send_move_command myData;
esp_now_peer_info_t peerInfo;


void OnDataSent(const uint8_t * des_addr, esp_now_send_status_t status){
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Deliver Failure");
}

void setup() {
  Serial.begin(115200);
  Serial.println("creating AP");
  recv_info.src_addr = src_addr;
  recv_info.des_addr = des_addr;
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK){
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, des_addr, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
// Reads in 14 chars, creates buffer for each char
char buffer[13];
char left[6];
char right[6];
char wifi_mode[1];
char control_mode[1];

void loop() {

  if (Serial.available() > 0){
      for (int i = 0; i < 13; i++){
      buffer[i] = Serial.read();
    }
    if (buffer[0] == 's') {
      // copy each segment of input buffer to char for each variable
      memcpy(&left, &buffer[1], 5);
      memcpy(&right, &buffer[6], 5);
      memcpy(&wifi_mode, &buffer[11], 1);
      memcpy(&control_mode, &buffer[12], 1);
      // converts each char buffer into interger and updates tranmission data structure
      myData.left = atoi(left);
      myData.right = atoi(right);
      myData.wifi_status = atoi(wifi_mode);
      myData.control_status = atoi(control_mode);
    }
  }
  esp_err_t result = esp_now_send(des_addr, (uint8_t *) &myData, sizeof(myData));
  if (result == ESP_OK){
    Serial.println("Send with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}
