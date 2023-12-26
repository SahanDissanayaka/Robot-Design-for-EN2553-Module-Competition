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

#define IRB1 44
#define IRB2 46
#define IRB3 48
#define IRB4 50
#define IRB5 52

#define USR1_Trig 38
#define USR1_Echo 39
#define USR2_Trig 25
#define USR2_Echo 53
#define USL1_Trig 37
#define USL1_Echo 31
#define USL2_Trig 50
#define USL2_Echo 48
#define USF_Trig 33
#define USF_Echo 35
#define max_distance 19

NewPing SonarL1(USL1_Trig,USL1_Echo,max_distance);
NewPing SonarL2(USL2_Trig,USL2_Echo,max_distance);
NewPing SonarR1(USR1_Trig,USR1_Echo,max_distance);
NewPing SonarR2(USR2_Trig,USR2_Echo,max_distance);

Servo servo1;
Servo servo2;

int angle1 = 0;
int angle2 = 85;

int Analog = 0;
int box = 0;
int white = 0;

int IR_val[8] = {0,0,0,0,0,0,0,0};
int IR_Analog[8] = {0,0,0,0,0,0,0,0};
int AIR_val[8] = {0,0,0,0,0,0,0,0};
int T_val[8] = {906, 846, 863, 875, 855, 842, 875, 794 };
int IR_weights[8] = {-60,-15,-10,-5,5,10,15,60};
int BIR_val[5] = {0,0,0,0,0};
int BIR_weights[4] = {-10,-5,5,10};

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

float PB,IB,DB;
float errorB;
float previousErrorB =0;
float KpB = 4;
float KdB = 4;
float KiB = 0;
bool junction_detected = false;
int speedAdjustBackward = 0;

int distance;


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
String detected_color;

bool gate_opened = false;
bool first_check_point_passed = false;
bool all_check_point_passed = false;

bool all_checkpoint_passed = false;
bool first_checkpoint_passed = false;
bool just_started = true;
bool near_to_box = false;

int count = 0;
int Ultra = 0;

int start;

#define select1 27
#define select2 53
#define select3 29

#define max_distance_guard 80

NewPing SonarLG1(USL1_Trig,USL1_Echo,max_distance_guard);
NewPing SonarRG1(USR1_Trig,USR1_Echo,max_distance_guard);
NewPing SonarFG(USF_Trig,USF_Echo,max_distance_guard);

int Us_val_guard[3] = {0,0,0};

bool guard_robot_detected = false;

int check1 = 0;
int check2 = 0;
bool going_left = false;
bool going_right = false;

#define sound_digital 45
const int detectionThreshold = 600; 
const int durationThreshold = 2000; 

unsigned long soundStartTime = 0; 
bool soundDetected = false; 
int Us_val[2] = {0,0};
bool path_is_free = false;
int USF_dis;
int USR_dis;

void setup() {
  Serial.begin(9600);
  pinMode(select1,INPUT_PULLUP);
  pinMode(select2,INPUT_PULLUP);
  pinMode(select3,INPUT_PULLUP);
  Lcd.begin();
  Lcd.setCursor(0,0);
  Lcd.print("Give me a option");
  delay(5000);
  int a = digitalRead(select1);
  Serial.print(a);
  int b = digitalRead(select2);
  Serial.print(b);
  int c = digitalRead(select3);
  Serial.print(c);
  Lcd.clear();
  Lcd.print("Time Out");
  if (a == 0 && b == 0 && c == 0 ){
    start = 0;}
  else if (a == 0 && b == 0 && c == 1 ){
    start = 1;}
  else if (a == 0 && b == 1 && c == 0 ){
    start = 2;}
  else if (a == 0 && b == 1 && c == 1 ){
    start = 3;}
  else if (a == 1 && b == 0 && c == 0 ){
    start = 4;}
  else if (a == 1 && b == 0 && c == 1 ){
    start = 5;}
  else if (a == 1 && b == 1 && c == 0 ){
    start = 6;}
  else {
    start = 7;}
  
  pinMode(LMotorA, OUTPUT);
  pinMode(LMotorB, OUTPUT);
  pinMode(LMotorPWM, OUTPUT);

  pinMode(RMotorA, OUTPUT);
  pinMode(RMotorB, OUTPUT);
  pinMode(RMotorPWM, OUTPUT);

  pinMode(USL1_Trig,OUTPUT);
  pinMode(USL1_Echo,INPUT);
  pinMode(USL2_Trig,OUTPUT);
  pinMode(USL2_Echo,INPUT);
  pinMode(USR1_Trig,OUTPUT);
  pinMode(USR1_Echo,INPUT);
  pinMode(USR2_Trig,OUTPUT);
  pinMode(USR2_Echo,INPUT);
  
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  pinMode(sound_digital,INPUT);

  servo1.attach(6);
  //  servo1.write(angle1);
  servo2.attach(7);
  //  servo2.write(angle2);
  moveServo(servo1, 90, 20); // Move servo1 to 90 degrees
  moveServo(servo2, 130, 30); // Move servo2 to 130 degrees

  

  
}

// Line
void Task1(){
  Lcd.clear();
  Lcd.print("Task 1");
  read_IR();
  PID_control();
  int ml = 200;
  int mr = 200;
  mdrive(ml+speedAdjust, mr-speedAdjust);
  
  if ( IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1 ){
    mdrive(100, 100);
    delay(300);
    read_IR();
    if ( IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1 ){
      delay(300);
      read_IR();
      if ( IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1 ){
        delay(300);
        read_IR();
        if ( IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1 ){
          delay(3000);
          mdrive(0,0);
          start = 1;
        }  
        else{
          mdrive(-120, 120);
          delay(1800);
        }
      }
    }
  }
  else{
    mdrive(ml+speedAdjust, mr-speedAdjust);
  }
}

//Wall
void Task2(){
  Lcd.clear();
  Lcd.print("Task 2");
  Lcd.print(Ultra);
  int ml = 200;
  int mr = 200;
  read_IR();
  read_US();
  PID_control();
  mdrive(ml+speedAdjust, mr-speedAdjust);
  if (Ultra == 0){
    if (Us_val[0] != 0 || Us_val[1] != 0 ){
      mdrive(-180,180);
      delay(1200);
      mdrive(0,0);
      delay(100);
      Ultra = 1;
      read_IR();
      while ( IR_val[2] == 0 && IR_val[3] == 0 && IR_val[4] == 0 && IR_val[5] == 0 ){
        mdrive(100,200);
        read_IR();
        }
      mdrive(150,150);
      delay(900);  
      mdrive(-150,150);
      delay(1300);
      mdrive(0,0);
      delay(100);  
    }
  }
  else if (Ultra == 1){
    if (Us_val[0] != 0 || Us_val[1] != 0 ){
      mdrive(180,-180);
      delay(1200);
      mdrive(0,0);
      delay(100);
      Ultra = 2;
      read_IR();
      while (IR_val[2] == 0 && IR_val[3] == 0 && IR_val[4] == 0 && IR_val[5] == 0 ){
        mdrive(200,100);
        read_IR();
        }
      mdrive(150,150);
      delay(900);   
      mdrive(150,-150);
      delay(1300);
      mdrive(0,0);
      delay(100);  
    }
  }
  else if (IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1){
    mdrive(150,150);
    delay(1600);
    mdrive(0, 0);
    delay(300);
    mdrive(180, -180);
    delay(1600);
    mdrive(200,200);
    delay(1200);
    start = 2;
  }
  else{
    read_IR();
    PID_control();
    mdrive(ml+speedAdjust, mr-speedAdjust);
  }
}

//Ramp
void Task3(){
  Lcd.clear();
  Lcd.print("Task 3");
  read_IR();
  PID_control();
  int ml = 200;
  int mr = 200;
  
  if ( IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1 ){
    mdrive(0, 0);
    delay(300);
    mdrive(200, 200);
    delay(3000);
    start = 3;
    
  }
  else{
    mdrive(ml+speedAdjust, mr-speedAdjust);
  }
}

//Gate
void Task4(){
  Lcd.clear();
  Lcd.print("Task 4");
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
                delay(3000);
                mdrive(0,0);
                start = 4;
    
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


//Color
void Task5(){
  Lcd.clear();
  Lcd.print("Task 1");
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
        mdrive(180,180);
        delay(2500);
        moveServo(servo1, 0, 30); // Move servo1 to 85 degrees
        moveServo(servo2, 130, 30); // Move servo2 back to 90 degrees
        moveServo(servo1, 90, 30);
        delay(500);
        mdrive(-180,-180);
        delay(1000);
        box = 0; 
        if (detected_color == "Bl"){
          mdrive(180,-180);
          delay(1600);
          mdrive(180,180);
          delay(150);
          mdrive(0,0);
          start = 5;
        }
        else if (detected_color == "Re"){
          mdrive(-180,180);
          delay(1600);
          mdrive(180,180);
          delay(150);
          mdrive(0,0);
          start = 5;
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

//Sound
void Task6(){
  Lcd.clear();
  Lcd.print("Task 6");
  int sensorValue = analogRead(sound_digital); 

  
  if (sensorValue > detectionThreshold) {
    
    if (!soundDetected) {
      soundStartTime = millis(); 
      soundDetected = true; 
      Serial.println("Sound detected!");
    }
    
    if (millis() - soundStartTime >= durationThreshold) {
      Serial.println("Bleep sound detected for a long time!");
      mdrive(0,0);
    }
  } else {
    soundDetected = false;
    read_IR();
    PID_control();
    int ml = 100;
    int mr = 100;
    
    mdrive(ml+speedAdjust, mr-speedAdjust);
    if (IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 ){
      mdrive(-200,200);
      delay(1600); 
    }
    else if ( IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1){
      mdrive(200,-200);
      delay(1600); 
    }
    else if (IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1){
      while((IR_val[0] == 0)|| (IR_val[7] == 0)){
        mdrive(200,200);
      }
      mdrive(200,200);
      delay(3000);
      start = 6;
    }
  }

}

//gaurd
void Task7(){
  Lcd.clear();
  Lcd.print("Task 7");
  if(!guard_robot_detected){
  unsigned long start_time = millis();
  
  while (millis()-start_time <= 5000){
  
  read_IR();
  PID_control();
  int ml = 100;
  int mr = 100;

  mdrive(ml+speedAdjust, mr-speedAdjust);

  
  }
  
  guard_robot_detected = true;
  
    }
  else{
    while (!path_is_free){
// -------------------- path free check -------------------------- //      
    if (check1 != 1){
    USF_dis = SonarFG.ping_cm();

    if (USF_dis < 50 && USF_dis > 0) {
      if (check2 == 1) {
        going_left = true;
        check2 = 0;
      } else {
        check1 = 1;
      }
  
    }
    }
    
    if (check2 != 1){
      USR_dis = SonarR1.ping_cm();
    if (USR_dis < 50 && USR_dis > 0) {
      if (check1 == 1) {
        going_right = true;
        check1 = 0;
      } else {
        check2 = 1;
      }
    }
      }

    if (going_left){
      path_is_free = true;
      }
    else if (going_right){
      USR_dis = SonarR1.ping_cm();
      while (!(USR_dis <= 50 && USR_dis > 0)){
        USR_dis = SonarR1.ping_cm();
        delay(50);
        }
      USF_dis = SonarFG.ping_cm();
      while (!(USF_dis <= 50 && USF_dis > 0)){
        USF_dis = SonarFG.ping_cm();
        delay(50);
        }
       path_is_free = true;
        }
      }
// -------------------- path free check -------------------------- //    
  while (true){
  read_IR();
  PID_control();
  int ml = 100;
  int mr = 100;

  mdrive(ml+speedAdjust, mr-speedAdjust);

  if ( IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1 ){
    mdrive(100, 100);
    delay(100);
    read_IR();
    if ( IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1 ){
      delay(100);
      read_IR();
      if ( IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1 ){
        delay(100);
        read_IR();
        if ( IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1 ){
          while(1){
            delay(200);
            mdrive(0,0);
          }
        }  
        else{
          mdrive(120, -120);
          delay(350);
        }
      }
      else{
        mdrive(120,-120);
        delay(350);
      } 
    }
    else{
        mdrive(120,-120);
        delay(350);
      } 
  
  }
    }
    }
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
      while(1){
        mdrive(0,0);
      }
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
    if (AIR_val[i] <= T_val[i]){
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

void read_US(){
  Us_val[0] = SonarL1.ping_cm();
  Serial.println(Us_val[0]);
  Us_val[1] = SonarR1.ping_cm();
  Serial.println(Us_val[1]);

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
  BIR_val[0] = digitalRead(IRB1);
  BIR_val[1] = digitalRead(IRB2);
  BIR_val[2] = digitalRead(IRB3);
  BIR_val[3] = digitalRead(IRB4);
  BIR_val[4] = digitalRead(IRB5);
}


void loop() {
  if (start == 0){
    Task1();
  }
  else if (start == 1){
    Task2();
    
  }
  else if (start == 2){
    Task3();
  }
  else if (start == 3){
    Task4();
  }
  else if (start == 4){
    Task5();
  }
  else if (start == 5){
    Task6();
  }
  else if (start == 6){
    Task7();
  }

}
