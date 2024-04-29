#include <driver/ledc.h>
#include <esp_now.h>
#include <WiFi.h>

#include <OneWire.h>

// ESP Now uses mac addresses to establish a link between two Esp32 devices
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
  int wifi_status;
  int control_status;
} recv_move_command;
recv_move_command message;

// Create a struct_message called myData
recv_move_command myData;
// buffer for checking control bit
recv_move_command newData;

#define wifi_status_pin 9 // toggle on to switch jetson into access point
#define control_status_pin 23 // manual or ROS control
// Motor control setup
#define left_motor  3  // pin 3
#define right_motor 4  // pin 4
#define PWM_Res     16 // 16 bit, 65536 values
#define PWM_Freq    50 // 50Hz
// Temperature Sensor
OneWire  ds(10);  // on pin 10 (a 4.7K resistor is necessary)
// TDS Sensor Values
#define TdsSensorPin 1        // insert data wire into gpio1 / pin1
#define VREF 3.3              // analog reference voltage(Volt) of the ADC
#define SCOUNT  30            // sum of sample point

// motor signals and time period between recieved signals
int L_sig, R_sig;
int time_delta, old_time;
int test_l, test_r;
// tds sensor
int analogBuffer[SCOUNT];     // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

float averageVoltage = 0;
float tdsValue = 0;
float temperature = 25;       // current temperature for compensation
float temperatureCelsius = 0;

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
// callback function that will be executed when data is received
void OnDataRecv(const esp_now_recv_info_t * rcv_info, const uint8_t *incomingData, int len) {
  memcpy(&newData, incomingData, sizeof(myData));
  if (newData.control_status == 0)  { // run if bit is set for manual control by copying transmitted data into control structure
    memcpy(&myData, incomingData, sizeof(myData));
  }
  // Troubleshooting messages
  /*
  Serial.print("\nBytes received: \n");     
  Serial.print("\nLeft: ");
  Serial.print(myData.left);
  Serial.print("\tRight: ");
  Serial.print(myData.right);
  Serial.print("\tWifi Status: ");
  Serial.print(myData.wifi_status);
  Serial.print("\tControl Status: ");
  Serial.print(myData.control_status);
  */
  // switch wifi mode
  if (myData.wifi_status == 1){ digitalWrite(wifi_status_pin, HIGH); }
  if (myData.wifi_status == 0){ digitalWrite(wifi_status_pin, LOW); }
  if (myData.control_status == 1){ digitalWrite(control_status_pin, HIGH); }
  if (myData.control_status == 0){ digitalWrite(control_status_pin, LOW); }

  L_sig = map(myData.left, -999, 1000, 3277, 6554);
  R_sig = map(myData.right, -999, 1000, 3277, 6554);
  test_l = ledcRead(3);
  test_r = ledcRead(4);
  //Serial.printf("Left: %d, Right: %d\n", test_l, test_r);
  ledcWrite(left_motor, L_sig);
  ledcWrite(right_motor, R_sig);
  time_delta += 1;
  delay(1);
}
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Set up the PWM Control for the ESC's
  ledcAttach(left_motor, PWM_Freq, PWM_Res);
  ledcAttach(right_motor, PWM_Freq, PWM_Res);
  // These timing variables are used to determine if the signal is no longer connected
  int time_delta = 0;
  int old_time = 0;
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
  pinMode(wifi_status_pin, OUTPUT);
  pinMode(control_status_pin, OUTPUT);
  digitalWrite(wifi_status_pin, LOW); // set initial state to 0
  // TDS Setup
  pinMode(TdsSensorPin,INPUT);
}

char buffer[14];
char left[6];
char right[6];
char wifi_mode[1];
char control_mode[1];
float temp_send;
float tds_send;
int wifi_status;
int control_status;

void loop() {
  // reads for data download from jetson nano
  if (Serial.available() > 0){
    for (int i=0; i<13; i++){
      buffer[i]=Serial.read();
    }
    if (buffer[0] == 's'){
      memcpy(&left, &buffer[1], 5);
      memcpy(&right, &buffer[6], 5);
      //memcpy(&wifi_mode, &buffer[11], 1); // uneeded as these bits are controlled by the transmitter not the jetson nano
      //memcpy(&control_mode, &buffer[12], 1);
      if (myData.control_status == 1){ // use jetson input if control_status is set to 1
        myData.left = atoi(left);
        myData.right = atoi(right);
        L_sig = map(myData.left, -999, 1000, 3277, 6554);
        R_sig = map(myData.right, -999, 1000, 3277, 6554);
        //Serial.printf("Left: %d, Right: %d\n", test_l, test_r);
        ledcWrite(left_motor, L_sig);
        ledcWrite(right_motor, R_sig);
      }
    }
  }
  if (time_delta == old_time){
    // ESP32 Servo might be better, it didn't work for us because it wasn't updated for ESP32-C6 yet
    // This code essentially sets the duty cycle half way between 50% and 100%
    // This is '0' velocity for the speed controller.
    // less then 75% is reverse, more than 75% is forwards
    ledcWrite(left_motor, 4915); // 75% duty cycle for 11 bit number to idle
    ledcWrite(right_motor, 4915);
    //Serial.println("Idling");
  }
  // This sets up the system count the timing between current and previous message
  old_time = time_delta;
  delay(10); // 100 frames per second

  // Temperature Sensing Code
  byte i;
  byte present = 0;
  byte type_s;
  byte data[9];
  byte addr[8];
  float celsius, fahrenheit;
  if ( !ds.search(addr)) {
    //Serial.println("No more addresses.");
    //Serial.println();
    ds.reset_search();
    delay(250);
  }
  //Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    //Serial.write(' ');
    //Serial.print(addr[i], HEX);
  }
  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
  }
  //Serial.println();
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      //Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      //Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      //Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      //Serial.println("Device is not a DS18x20 family device.");
      break;
  }
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad
  //Serial.print("  Data = ");
  //Serial.print(present, HEX);
  //Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    //Serial.print(data[i], HEX);
    //Serial.print(" ");
  }
  //Serial.print(" CRC=");
  //Serial.print(OneWire::crc8(data, 8), HEX);
  //Serial.println();
  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;

  // TDS Code
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
  if(millis()-printTimepoint > 800U)  {
    printTimepoint = millis();
    for(copyIndex=0; copyIndex<SCOUNT; copyIndex++){
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      
      // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 4096.0;
      
      //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0)); 
      // used our own measured temperature
      float compensationCoefficient = 1.0+0.02*(temperature-25.0);
      //temperature compensation
      float compensationVoltage=averageVoltage/compensationCoefficient;
      
      //convert voltage value to tds value
      tdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5;
      /*
      Serial.print("voltage:");
      Serial.print(averageVoltage,2);
      Serial.print("V   ");
      Serial.print("TDS Value:");
      Serial.print(tdsValue,0);
      Serial.println("ppm");
      */
    }
  }
  /*
  Serial.print("  Temperature = ");
  Serial.print(celsius);
  Serial.print(" Celsius, ");
  Serial.print(fahrenheit);
  Serial.println(" Fahrenheit");
  */
  Serial.print(tdsValue,0);
  Serial.print(" ");
  Serial.print(celsius);
  Serial.print("\n");
}
