#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C Lcd(0x27,16,2);
#include <Servo.h>
#include <NewPing.h>

#define trigPin 33  // Define the trigger pin
#define echoPin 35     // Define the echo pin
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define S0 8
#define S1 11
#define S2 12
#define S3 13
#define sensorOut 43

#define IR1 5
#define IR2 4
#define IR3 3
#define IR4 2
#define IR5 24
#define IR6 22
#define IR7 25
#define IR8 23

#define LMotorA 32
#define LMotorB 30
#define LMotorPWM 10

#define RMotorA 28
#define RMotorB 26
#define RMotorPWM 9

#define MAX_SPEED 240

Servo servo1;
Servo servo2;

int angle1 = 0;
int angle2 = 85;

int IR_val[8] = {0,0,0,0,0,0,0,0};
int pre_IR_val[8] = {0,0,0,0,0,0,0,0};
int IR_weights[8] = {-60,-15,-10,-5,5,10,15,60};

int MotorBasespeed = 120;


int LMotorSpeed = 0;
int RMotorSpeed = 0;
int speedAdjust = 0;

float P,I,D;
float error;
float previousError =0;
float Kp = 4;
float Kd = 4;
float Ki = 0;

NewPing sonar(trigPin, echoPin, MAX_DISTANCE);

int redMin = 17;
int redMax = 156;
int greenMin = 18;
int greenMax = 176;
int blueMin = 16;
int blueMax = 159;

int redPW = 0;
int greenPW = 0;
int bluePW = 0;

int redValue;
int greenValue;
int blueValue;

int distance;
////////////////////////////////////////
#define IRB1 45
#define IRB2 47
#define IRB3 49
#define IRB4 51
#define IRB5 52

float PB,IB,DB;
float errorB;
float previousErrorB =0;
float KpB = 4;
float KdB = 4;
float KiB = 0;
bool junction_detected = false;
int speedAdjustBackward = 0;

int BIR_val[5] = {0,0,0,0,0};
int BIR_weights[4] = {-10,-5,5,10};

bool gate_opened = false;
bool first_check_point_passed = false;
bool all_check_point_passed = false;

bool all_checkpoint_passed = false;
bool first_checkpoint_passed = false;
bool just_started = true;
bool near_to_box = false;
////////////////////////////////////////
void setup() {
  Lcd.begin();
  Lcd.backlight();

  Lcd.setCursor(0,0);
  Lcd.print(" The Titans ");
  servo1.attach(6);
//  servo1.write(angle1);
  servo2.attach(7);
//  servo2.write(angle2);
  Serial.begin(9600); // Initialize serial communication

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  pinMode(LMotorA, OUTPUT);
  pinMode(LMotorB, OUTPUT);
  pinMode(LMotorPWM, OUTPUT);

  pinMode(RMotorA, OUTPUT);
  pinMode(RMotorB, OUTPUT);
  pinMode(RMotorPWM, OUTPUT);
}

void moveServo(Servo servo, int targetAngle, int delayTime) {
  int currentAngle = servo.read();
  if (currentAngle != targetAngle) {
    if (targetAngle > currentAngle) {
      for (int i = currentAngle; i <= targetAngle; i++) {
        servo.write(i);
        delay(delayTime);
      }
    } else {
      for (int i = currentAngle; i >= targetAngle; i--) {
        servo.write(i);
        delay(delayTime);
      }
    }
  }
}

int getRedPW() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  int PW = pulseIn(sensorOut, LOW);
  return PW;
}

int getGreenPW() {
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  int PW = pulseIn(sensorOut, LOW);
  return PW;
}

int getBluePW() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  int PW = pulseIn(sensorOut, LOW);
  return PW;
}

String color(){
  redPW = getRedPW();
  redValue = map(redPW, redMin, redMax, 255, 0);
  delay(200);

  greenPW = getGreenPW();
  greenValue = map(greenPW, greenMin, greenMax, 255, 0);
  delay(200);

  bluePW = getBluePW();
  blueValue = map(bluePW, blueMin, blueMax, 255, 0);
  delay(200);

  Serial.print("Red = ");
  Serial.print(redValue);
  Serial.print(" --- Green = ");
  Serial.print(greenValue);
  Serial.print(" --- Blue = ");
  Serial.println(blueValue);
  Lcd.clear();
  if (redValue > greenValue && redValue > blueValue) {
    Serial.println("Color is Red");
    Lcd.print("Red");
    return "Red";
  } 
  else if (greenValue > redValue && greenValue > blueValue) {
    Serial.println("Color is Green");
    Lcd.print("Green");
    return "Green";
  }
  else if (blueValue > greenValue && blueValue > redValue) {
    Serial.println("Color is Blue");
    Lcd.print("Blue");
    return "Blue";
  } 
}

int getDistance() {
  delay(50); // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  unsigned int distance = sonar.ping_cm(); // Send a ping and get the distance in centimeters.
  return distance;
}
void read_IR(){
  IR_val[0] = !digitalRead(IR1);
  IR_val[1] = !digitalRead(IR2);
  IR_val[2] = !digitalRead(IR3);
  IR_val[3] = !digitalRead(IR4);
  IR_val[4] = !digitalRead(IR5);
  IR_val[5] = !digitalRead(IR6);
  IR_val[6] = !digitalRead(IR7);
  IR_val[7] = !digitalRead(IR8);

   Serial.print(IR_val[0]);
   Serial.print(" ");
   Serial.print(IR_val[1]);
   Serial.print(" ");
   Serial.print(IR_val[2]);
   Serial.print(" ");
   Serial.print(IR_val[3]);
   Serial.print(" ");
   Serial.print(IR_val[4]);
   Serial.print(" ");
   Serial.print(IR_val[5]);
   Serial.print(" ");
   Serial.print(IR_val[6]);
   Serial.print(" ");
   Serial.println(IR_val[7]);
}

void PID_control(){

  error = 0;
  
  for (int i=0; i<8; i++){
    error += IR_weights[i] * IR_val[i];
    }

  P = error;
  I = I + error;
  D = error - previousError;

  previousError = error;

  speedAdjust = Kp * P + Ki * I + Kd * D;

  
}

void mdrive(int ml, int mr){
  if (ml > 0) {
    if (ml > 255) {
      ml = 255;
    }
    digitalWrite(LMotorA, HIGH);
    digitalWrite(LMotorB, LOW);
    analogWrite(LMotorPWM, ml);
  }
  else {
    if (ml < -255) {
      ml = -255;
    }
    digitalWrite(LMotorA, LOW);
    digitalWrite(LMotorB, HIGH);
    analogWrite(LMotorPWM, -1*ml);

  }
  if (mr > 0) {
    if (mr > 255) {
      mr = 255;
    }
    digitalWrite(RMotorA, HIGH);
    digitalWrite(RMotorB, LOW);
    analogWrite(RMotorPWM, mr);
  }
  else {
    if (mr < -255) {
      mr = -255;
    }
    digitalWrite(RMotorA, LOW);
    digitalWrite(RMotorB, HIGH);
    analogWrite(RMotorPWM, -1*mr);

  }
}

void Go_backward(){
  Lcd.clear();
  Lcd.print("G");
  while (!junction_detected){
    read_IRB();
    PID_control_backward();
    int ml = -100;
    int mr = -100;

    mdrive(ml-speedAdjustBackward, mr+speedAdjustBackward);

    if (BIR_val[4] == 1){
      junction_detected = true;
      }
    
    }
    }

void PID_control_backward(){
  errorB = 0;
    
  for (int i=0; i<4; i++){
    errorB += BIR_weights[i] * BIR_val[i];
    }

  PB = errorB;
  IB = IB + errorB;
  DB = errorB - previousErrorB;

  previousErrorB = errorB;

  speedAdjustBackward = KpB * PB + KiB * IB + KdB * DB;
  }

void read_IRB(){
  BIR_val[0] = !digitalRead(IRB1);
  BIR_val[1] = !digitalRead(IRB2);
  BIR_val[2] = !digitalRead(IRB3);
  BIR_val[3] = !digitalRead(IRB4);
  BIR_val[4] = !digitalRead(IRB5);
  
  }


void loop() {
  moveServo(servo1, 90, 20); // Move servo1 to 90 degrees
  moveServo(servo2, 130, 30); // Move servo2 to 130 degrees

  if (!gate_opened){distance = getDistance();
  }else{distance = 0;}

  if ((distance < 6 ) && (distance > 0) && !(gate_opened)) {
    Lcd.clear();
    Lcd.print("Detected");
    mdrive(0, 0);
    moveServo(servo1, 0, 30); // Move servo1 to 0 degrees
    delay(100);
    //String detected_color = color();
    moveServo(servo2, 85, 30); // Move servo2 to 85 degrees
    //moveServo(servo1, 90, 30); // Move servo1 back to 90 degrees
    Go_backward();
    near_to_box = false;
    mdrive(0,0);
    delay(100);
    moveServo(servo2,130,30);
    moveServo(servo1,90,30);
    read_IR();
    while (!(IR_val[0] == 1 || IR_val[7] == 1)){
      mdrive(-200,-200);
      read_IR();
      }
    mdrive(180,-180);
    delay(1600);
    gate_opened = true;
    
  }
  else{
    read_IR();
    PID_control();
    int ml = 200;
    int mr = 200;

    mdrive(ml+speedAdjust, mr-speedAdjust);

    if ( IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1 ){
      if ((all_checkpoint_passed || just_started) && !near_to_box){
        mdrive(100, 100);
        delay(200);
        Lcd.clear();
        Lcd.print("A");
        read_IR();
        if ( IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1 ){
          delay(200);
          read_IR();
          Lcd.clear();
          Lcd.print("A2");
          if ( IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1 ){
            delay(200);
            read_IR();
            Lcd.clear();
            Lcd.print("A3");
            if ( IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1 ){
              Lcd.print("A4");
              while(1){
                delay(200);
                mdrive(0,0);
              }
            }
            else{
              mdrive(200,200);
              delay(100);
              mdrive(180, -180);
              delay(1600);
              just_started = false;
              near_to_box = true;
            }
          }else{
              mdrive(200,200);
              delay(100);
              mdrive(180, -180);
              delay(1600);
              just_started = false;
              near_to_box = true;
        }
        }else{
              mdrive(200,200);
              delay(100);
              mdrive(180, -180);
              delay(1600);
              just_started = false;
              near_to_box = true;
            }
      
    }else{
          mdrive(100,100);
          delay(200);
          mdrive(0,0);
          delay(200);
          Lcd.clear();
          Lcd.print("D");
          if (first_checkpoint_passed){
            mdrive(200,200);
            delay(800);
            mdrive(180,-180);
            delay(1600);
            all_checkpoint_passed = true;
            Lcd.clear();
            Lcd.print("D2");
            }else if (!first_checkpoint_passed && !near_to_box){
              mdrive(200,200);
              delay(800);
              mdrive(-180,180);
              delay(1600);
              Lcd.clear();
              Lcd.print("D3");
              }
          mdrive(100,100);
          delay(200);
          }
    
  }
  }
  }
  
