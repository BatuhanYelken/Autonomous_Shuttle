#define REMOTEXY_MODE__ESP32CORE_WIFI_POINT
#include <WiFi.h>
#include <RemoteXY.h>

// RemoteXY connection settings
#define REMOTEXY_WIFI_SSID "ESP32"
#define REMOTEXY_WIFI_PASSWORD "pppppppp"
#define REMOTEXY_SERVER_PORT 6377
#define ENC_COUNT_REV 800 //Motor encoder output pulses per 360 degree revolution (measured manually)


// RemoteXY configurate  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =   // 147 bytes
  { 255,4,0,33,0,140,0,16,24,5,1,10,26,62,12,12,0,6,31,94,
  0,1,10,12,76,12,12,0,6,31,60,0,1,10,40,76,12,12,0,6,
  31,62,0,1,10,26,76,12,12,0,1,31,83,84,79,80,0,129,0,9,
  10,19,5,0,16,76,101,102,116,32,80,73,68,0,129,0,35,10,22,5,
  0,16,82,105,103,104,116,32,80,73,68,0,131,1,11,33,43,8,1,2,
  31,86,101,108,111,99,105,116,121,32,40,107,109,47,104,41,0,67,4,9,
  17,20,7,0,133,16,11,67,4,35,17,20,7,0,133,16,11,67,4,23,
  44,20,7,0,133,16,11 };

// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  uint8_t Forward; // =1 if button pressed, else =0 
  uint8_t Left; // =1 if button pressed, else =0 
  uint8_t Right; // =1 if button pressed, else =0 
  uint8_t Stop; // =1 if button pressed, else =0 
    // output variables
  char Left_PID[11];  // string UTF8 end zero 
  char Right_PID[11];  // string UTF8 end zero 
  char Velocity[11];  // string UTF8 end zero 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

const int motor1PWM = 14; // Motor 1 PWM pin (Speed control)
const int motor1Dir = 27;  // Motor 1 Direction pin
const int motor2PWM = 26; // Motor 2 PWM pin (Speed control)
const int motor2Dir = 25; // Motor 2 Direction pin
const int motor1ENCA = 22;     // Motor 1 Encoder Pin (Yellow)
const int motor1ENCB = 18;    // Motor 1 Encoder Pin (White)
const int motor2ENCA = 19;    // Motor 2 Encoder Pin (Yellow)
const int motor2ENCB = 21;    // Motor 2 Encoder Pin (White)
const int interval = 1000;    //One-second interval for measurements
const float wheelCircumference = 0.00028274333;    // Wheel circumference in km

// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;

long previousPos = 0;
long currentPos = 0;

int16_t EncPosition1 = 0;
int16_t EncPosition2 = 0;
float v = 0.0;
float rpm_right = 0.0;
float rpm_left = 0.0;

// LEDC settings for both motors
const int ledChannel1 = 0;
const int ledChannel2 = 1;
const int freq = 5000;  // Frequency in Hz
const int resolution = 8;  // 8-bit resolution
//char buffer[11];  // Temporary buffer to hold the string representation
void setup() 
{
  Serial.begin(115200);
  RemoteXY_Init (); 
  // PWM setup for motor 1 speed control
  ledcSetup(ledChannel1, freq, resolution);
  ledcAttachPin(motor1PWM, ledChannel1);

  // PWM setup for motor 2 speed control
  ledcSetup(ledChannel2, freq, resolution);
  ledcAttachPin(motor2PWM, ledChannel2);

  // Set motor direction pins as output
  pinMode(motor1Dir, OUTPUT);
  pinMode(motor2Dir, OUTPUT);

  // Set encoder pins as inputs
  pinMode(motor1ENCA, INPUT);
  pinMode(motor1ENCB, INPUT);
  pinMode(motor2ENCA, INPUT);
  pinMode(motor2ENCB, INPUT);

  attachInterrupt(digitalPinToInterrupt(motor1ENCA),readEncoder1,RISING);
  attachInterrupt(digitalPinToInterrupt(motor2ENCA),readEncoder2,RISING);
}
void loop() {

  
  RemoteXY_Handler(); // Handle RemoteXY communication


  Serial.print(EncPosition1);
  Serial.print("  ");
  Serial.print(EncPosition2);
  Serial.print("  ");
  Serial.print("Speed:  ");
  Serial.print(v);
  Serial.println();

  // Handle button presses from RemoteXY interface

  if (RemoteXY.Forward) {
    // Move forward
    ledcWrite(ledChannel1, 255);
    digitalWrite(motor1Dir, HIGH);
    ledcWrite(ledChannel2, 255);
    digitalWrite(motor2Dir, HIGH);

  }
  else if (RemoteXY.Left) {
    // Turn right
    ledcWrite(ledChannel1, 50);
    digitalWrite(motor1Dir, HIGH);
    ledcWrite(ledChannel2, 0);
    digitalWrite(motor2Dir, LOW);
  }
  else if (RemoteXY.Right) {
    // Turn left
    ledcWrite(ledChannel1, 0);
    digitalWrite(motor1Dir, LOW);
    ledcWrite(ledChannel2, 50);
    digitalWrite(motor2Dir, HIGH);
  }
  else if (RemoteXY.Stop) {
    // Stop motors
    ledcWrite(ledChannel1, LOW);
    digitalWrite(motor1Dir, 0);
    ledcWrite(ledChannel2, LOW);
    digitalWrite(motor2Dir, 0);
  } 
  else {
    // No button is pressed, stop motors
    ledcWrite(ledChannel1, 0);
    digitalWrite(motor1Dir, LOW);
    ledcWrite(ledChannel2, 0);
    digitalWrite(motor2Dir, LOW);
  }
    currentMillis = millis();   //record time

    if (currentMillis - previousMillis > interval){
      previousMillis = currentMillis;
      currentPos = EncPosition1;

      rpm_right = (float)((currentPos - previousPos) * 60 / ENC_COUNT_REV);
      v = rpm_right * wheelCircumference * 60;   //km/h

      previousPos = currentPos;

    }  

    snprintf(RemoteXY.Left_PID, sizeof(RemoteXY.Left_PID), "%d", EncPosition2);
    snprintf(RemoteXY.Right_PID, sizeof(RemoteXY.Right_PID), "%d", EncPosition1);
    snprintf(RemoteXY.Velocity, sizeof(RemoteXY.Velocity), "%f", v);
  


  }

void readEncoder1(){

  int encData = digitalRead(motor1ENCB);

  if(encData>0){
    EncPosition1++;
  }
  else EncPosition1--;

}

void readEncoder2(){

  int encData = digitalRead(motor2ENCB);

  if(encData>0){
    EncPosition2--;
  }
  else EncPosition2++;
}

