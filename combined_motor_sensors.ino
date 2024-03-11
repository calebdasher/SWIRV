#include <driver/ledc.h>
#include <esp_now.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>

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

// GPIO where the DS18B20 is connected to
const int oneWireBus = 4;     

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

#define left_motor  3  // pin 3
#define right_motor 4  // pin 4
#define PWM_Res     16 // 16 bit, 65536 values
#define PWM_Freq    50 // 50Hz
#define TdsSensorPin 1        // insert data wire into gpio1 / pin1
#define VREF 3.3              // analog reference voltage(Volt) of the ADC
#define SCOUNT  30            // sum of sample point

int L_sig, R_sig;
int time_delta, old_time;
int test_l, test_r;
// callback function that will be executed when data is received

int analogBuffer[SCOUNT];     // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen){
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen; i++)
  bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0){
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}


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

    pinMode(TdsSensorPin,INPUT);
    
      // Start the DS18B20 sensor
  sensors.begin();
    
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

  static unsigned long analogSampleTimepoint = millis();
  if(millis()-analogSampleTimepoint > 40U){     //every 40 milliseconds,read the analog value from the ADC
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if(analogBufferIndex == SCOUNT){ 
      analogBufferIndex = 0;
    }
  }   
  
  static unsigned long printTimepoint = millis();
  if(millis()-printTimepoint > 800U){
    printTimepoint = millis();
    for(copyIndex=0; copyIndex<SCOUNT; copyIndex++){
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      
      // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 4096.0;
      
      //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0)); 
      float compensationCoefficient = 1.0+0.02*(temperature-25.0);
      //temperature compensation
      float compensationVoltage=averageVoltage/compensationCoefficient;
      
      //convert voltage value to tds value
      tdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5;
      //Serial.print("voltage:");
      //Serial.print(averageVoltage,2);
      //Serial.print("V   ");
      Serial.print("TDS Value:");
      Serial.print(tdsValue,0);
      Serial.println("ppm");
      delay(2000);
    }
      for sensors.requestTemperatures(){
  float temperatureC = sensors.getTempCByIndex(0);
  float temperatureF = sensors.getTempFByIndex(0);
  Serial.print(temperatureC);
  Serial.println(" - Celsius");
  Serial.print(temperatureF);
  Serial.println("  - Fahrenheit");
  delay(5000);
      }
  }



  
}
