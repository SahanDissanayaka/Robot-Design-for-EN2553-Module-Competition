#include <LiquidCrystal_I2C.h>

#include <Wire.h>


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

#define AIR1 A0
#define AIR2 A1
#define AIR3 A2
#define AIR4 A3
#define AIR5 A4
#define AIR6 A5
#define AIR7 A6
#define AIR8 A7

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

int Analog = 0;
int box = 0;

int IR_val[8] = {0,0,0,0,0,0,0,0};
int IR_Analog[8] = {0,0,0,0,0,0,0,0};
int AIR_val[8] = {0,0,0,0,0,0,0,0};
int T_val[8] = {906, 846, 863, 875, 855, 842, 875, 794 };
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

String detected_color = "AA";
int count = 0;
int distance;

void setup() {
  Lcd.begin(16,2);
  Lcd.backlight();

  Lcd.setCursor(0,0);
  Lcd.print(" The Titans ");
  servo1.attach(6);
  // servo1.write(angle1);
  servo2.attach(7);
  // servo2.write(angle2);
  moveServo(servo1, 90, 20); // Move servo1 to 90 degrees
  moveServo(servo2, 130, 30); // Move servo2 to 130 degrees
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

void go_little(){
  mdrive(200,200);
  delay(900);
  mdrive(0,0);
}

void Analog_Go(){
  while (true) {
  Lcd.clear();
  Lcd.print("Analog Go");
  read_AIR();
  PID_control();
  int ml = 200;
  int mr = 200;
  // mdrive(ml, mr);
  mdrive(ml+speedAdjust, mr-speedAdjust); 

  if (AIR_val[0] == 1 && AIR_val[1] == 1 && AIR_val[2] == 1 && AIR_val[3] == 1 && AIR_val[4] == 1 && AIR_val[5] == 1 && AIR_val[7] == 0 ){
    go_little();
    mdrive(-180,180);
    delay(1600); 
    
  }
  else if( AIR_val[0] == 1 && AIR_val[1] == 1 && AIR_val[2] == 1 && AIR_val[3] == 1 && AIR_val[4] == 1 && AIR_val[5] == 1 && AIR_val[6] == 1 && AIR_val[7] == 1){
    go_little();
    read_AIR();
    if (AIR_val[0] == 0 && AIR_val[7] == 0 ){
      mdrive(-180,180);
      delay(1600);
    }
    else if (AIR_val[0] == 1 && AIR_val[1] == 1 && AIR_val[2] == 1 && AIR_val[3] == 1 && AIR_val[4] == 1 && AIR_val[5] == 1 && AIR_val[6] == 1 && AIR_val[7] == 1){
      // while(1){
      //   mdrive(0,0);
      // }
    }
  }
  else if(AIR_val[0] == 0 && AIR_val[1] == 0 && AIR_val[2] == 0 && AIR_val[3] == 0 && AIR_val[4] == 0 && AIR_val[5] == 0 && AIR_val[6] == 0 && AIR_val[7] == 0){
    go_little();
    read_AIR();
    if (AIR_val[0] == 0 && AIR_val[1] == 0 && AIR_val[2] == 0 && AIR_val[3] == 0 && AIR_val[4] == 0 && AIR_val[5] == 0 && AIR_val[6] == 0 && AIR_val[7] == 0 ){
      mdrive(-180,180);
      delay(3100);  
    }
  }
  else if (AIR_val[7] == 1 && AIR_val[6] == 1 && AIR_val[5] == 1 && AIR_val[4] == 1 && AIR_val[3] == 1 && AIR_val[2] == 1 && AIR_val[0] == 0 ){
    go_little();
    mdrive(180,-180);
    delay(1600); 
  }  
  }
  
}

void read_AIR(){
  AIR_val[0] = analogRead(AIR1);
  AIR_val[1] = analogRead(AIR2);
  AIR_val[2] = analogRead(AIR3);
  AIR_val[3] = analogRead(AIR4);
  AIR_val[4] = analogRead(AIR5);
  AIR_val[5] = analogRead(AIR6);
  AIR_val[6] = analogRead(AIR7);
  AIR_val[7] = analogRead(AIR8);
    
  for (int i = 0; i < 8; i++) {
    if (IR_Analog[i] <= T_val[i]){
      AIR_val[i] = 1;      
      // Serial.print("Blue");
      // Serial.print("  ");

    }
    else{
      // Serial.print("Black");
      // Serial.print("  ");
      AIR_val[i] = 0; 
    }
  }
  Serial.println("  ");
  Serial.print(AIR_val[0]);
  Serial.print(" ");
  Serial.print(AIR_val[1]);
  Serial.print(" ");
  Serial.print(AIR_val[2]);
  Serial.print(" ");
  Serial.print(AIR_val[3]);
  Serial.print(" ");
  Serial.print(AIR_val[4]);
  Serial.print(" ");
  Serial.print(AIR_val[5]);
  Serial.print(" ");
  Serial.print(AIR_val[6]);
  Serial.print(" ");
  Serial.println(AIR_val[7]);
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

  // Serial.print(IR_val[0]);
  // Serial.print(" ");
  // Serial.print(IR_val[1]);
  // Serial.print(" ");
  // Serial.print(IR_val[2]);
  // Serial.print(" ");
  // Serial.print(IR_val[3]);
  // Serial.print(" ");
  // Serial.print(IR_val[4]);
  // Serial.print(" ");
  // Serial.print(IR_val[5]);
  // Serial.print(" ");
  // Serial.print(IR_val[6]);
  // Serial.print(" ");
  // Serial.println(IR_val[7]);
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


void loop() {
  if(box == 1) {
    read_IR();
    PID_control();
    int ml = 200;
    int mr = 200;

    mdrive(ml+speedAdjust, mr-speedAdjust);

    if ((IR_val[0] == 1 || IR_val[7] == 1) && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && (detected_color == "Blue" || detected_color == "Green" )){
      mdrive(180,180);
      delay(1200);
      mdrive(180,-180);
      delay(1600);
      mdrive(180,190);
      delay(1000);
      detected_color = "Bl";
      read_IR();
      PID_control();
      int ml = 200;
      int mr = 200;
      mdrive(ml+speedAdjust, mr-speedAdjust);
    }  
    else if ((IR_val[0] == 1 || IR_val[7] == 1) && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && detected_color == "Red" ){
      mdrive(180,180);
      delay(1200);
      mdrive(-180,180);
      delay(1600);
      mdrive(180,190);
      delay(1000);
      detected_color = "Re";
      read_IR();
      PID_control();
      int ml = 200;
      int mr = 200;
      mdrive(ml+speedAdjust, mr-speedAdjust);
    }

    else if( IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 ){
      go_little();
      read_IR();
      if (IR_val[0] == 0 || IR_val[7] == 0 ){
        mdrive(-180,180);
        delay(1600);
        read_IR();
        PID_control();
        int ml = 200;
        int mr = 200;
        mdrive(ml+speedAdjust, mr-speedAdjust);
      }  
      else if (IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1){
        mdrive(0,0);
        moveServo(servo1, 0, 30); // Move servo1 to 85 degrees
        moveServo(servo2, 130, 30); // Move servo2 back to 90 degrees
        moveServo(servo1, 90, 30);
        delay(500);
        mdrive(-180,-180);
        delay(1000);
        box = 0; 
        while(1){
          mdrive(0,0);
        } 
        
      }
    }
    else if(  IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1 ){
      go_little();
      read_IR();
      if (IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1){
        mdrive(180,-180);
        delay(1600);
        read_IR();
        PID_control();
        int ml = 200;
        int mr = 200;
        mdrive(ml+speedAdjust, mr-speedAdjust);
      }
      else{
        mdrive(180,190);
        delay(1000);
        read_IR();
        PID_control();
        int ml = 200;
        int mr = 200;
        mdrive(ml+speedAdjust, mr-speedAdjust);
      }
    }

  }

  else if (box == 0){
    
    read_IR();
    PID_control();
    int ml = 200;
    int mr = 200;
    mdrive(ml+speedAdjust, mr-speedAdjust);
    if( (IR_val[0] == 1 || IR_val[7] == 1) && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 ){
      if (count == 0){
        mdrive(180,190);
        delay(1000);
        read_IR();
        PID_control();
        int ml = 200;
        int mr = 200;
        mdrive(ml+speedAdjust, mr-speedAdjust);
        count +=1;
      }
      else if (count == 1){
        distance = getDistance();
        if ((distance < 20 ) && (distance > 0) ){
          while((distance > 6 ) || (distance == 0)){
            mdrive(180,180);
            delay(10);
            distance = getDistance();
          }
          mdrive(0, 0);
          moveServo(servo1, 0, 30); // Move servo1 to 0 degrees
          delay(500);
          detected_color = color();
          delay(100);
          moveServo(servo2, 85, 30); // Move servo2 to 85 degrees
          moveServo(servo1, 90, 30); // Move servo1 back to 90 degrees
          box = 1;
          mdrive(-180,180);
          delay(3100);
          mdrive(200,200);
          delay(1000);
          read_IR();
          PID_control();
          int ml = 200;
          int mr = 200;
          mdrive(ml+speedAdjust, mr-speedAdjust);
        }
        
        else{
          long long T = millis();
          while((distance == 0 || distance > 30) && (millis() < T+1600) ){
            mdrive(-150,150);
            delay(10);
            distance = getDistance();
          }
          if (distance <=30){
            T = millis()-T;
            while((distance > 6 ) || (distance == 0)){
              mdrive(180,180);
              delay(10);
              distance = getDistance();
            }
            mdrive(0, 0);
            moveServo(servo1, 0, 30); // Move servo1 to 0 degrees
            delay(500);
            detected_color = color();
            delay(100);
            moveServo(servo2, 85, 30); // Move servo2 to 85 degrees
            moveServo(servo1, 90, 30); // Move servo1 back to 90 degrees
            box = 1;
            mdrive(-180,180);
            delay(3200-T);
            mdrive(200,200);
            delay(1000);
            read_IR();
            PID_control();
            int ml = 200;
            int mr = 200;
            mdrive(ml+speedAdjust, mr-speedAdjust);
          }
          
          else{
            mdrive(0,0);
            delay(100);
            mdrive(180,-180);
            delay(1600);
            mdrive(0,0);
            delay(100);
            T = millis();
            while((distance == 0 || distance > 30) && (millis() < T+1600) ){
              mdrive(150,-150);
              delay(10);
              distance = getDistance();
            }
            T = millis() -T;
            while((distance > 6 ) || (distance == 0)){
              mdrive(180,180);
              delay(10);
              distance = getDistance();
            }
            mdrive(0, 0);
            moveServo(servo1, 0, 30); // Move servo1 to 0 degrees
            delay(500);
            detected_color = color();
            delay(100);
            moveServo(servo2, 85, 30); // Move servo2 to 85 degrees
            moveServo(servo1, 90, 30); // Move servo1 back to 90 degrees
            box = 1;
            mdrive(180,-180);
            delay(3100-T);
            mdrive(200,200);
            delay(1000);
            read_IR();
            PID_control();
            int ml = 200;
            int mr = 200;
            mdrive(ml+speedAdjust, mr-speedAdjust);
          }
        }
      }
    }
  }
}
