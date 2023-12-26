#include <NewPing.h>
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

int IR_val[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int IR_weights[8] = {-60, -15, -10, -5, 5, 10, 15, 60};


int LMotorSpeed = 0;
int RMotorSpeed = 0;
int speedAdjust = 0;

float P, I, D;
float error;
float previousError = 0;
float Kp = 4;
float Kd = 4;
float Ki = 0;

void read_IR();
void PID_control();
void mdrive();

#define USR1_Trig 38
#define USR1_Echo 39
#define USR2_Trig 25
#define USR2_Echo 53
#define USL1_Trig 37
#define USL1_Echo 31
#define USL2_Trig 50
#define USL2_Echo 48
#define max_distance 20

NewPing SonarL1(USL1_Trig,USL1_Echo,max_distance);
NewPing SonarL2(USL2_Trig,USL2_Echo,max_distance);
NewPing SonarR1(USR1_Trig,USR1_Echo,max_distance);
NewPing SonarR2(USR2_Trig,USR2_Echo,max_distance);

int Us_val[2] = {0,0};
//int Us_weights[4] = {-1,-1,1,1};
float Pw,Iw,Dw;
int errorw = 0;
float previousErrorw = 0;
float Kpw = 4;
float Kdw = 1;
float Kiw = 0;
void read_US();
int speedAdjustw = 0;
int safety_distance = 18;
int Left_US = 0;
int Right_US = 0;

void setup() {
  Serial.begin(9600);

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

  
}

void loop() {
  Serial.println("A");
  int ml = 200;
  int mr = 200;
  read_IR();
  read_US();
  
  if (Us_val[0] != 0){
    Serial.println("B");
    mdrive(0,0);
    delay(200);
    read_IR();
    while (IR_val[0] == 1 || IR_val[1] == 1 || IR_val[2] == 1 || IR_val[3] == 1 || IR_val[4] == 1 || IR_val[5] == 1 || IR_val[6] == 1 || IR_val[7] == 1){
      mdrive(120,-120);
      read_IR();
      }
    mdrive(0,0);delay(1000);
    //IR_val[0] != 1 && IR_val[1] != 1 && IR_val[2] != 1 && IR_val[3] != 1 && IR_val[4] != 1 && IR_val[5] != 1 && IR_val[6] != 1 && 
    read_IR();
    while (IR_val[3] != 1 && IR_val[4] != 1){
      Serial.println("N");
      PID_control("WallL");
      mdrive(ml + speedAdjust + speedAdjustw, mr - speedAdjust - speedAdjustw);
      read_IR();
      }
      mdrive(120,-120);
      delay(1000);
    }
  else if (Us_val[1] != 0){
    Serial.println("B");
    mdrive(0,0);
    delay(200);
    read_IR();
    while (IR_val[0] == 1 || IR_val[1] == 1 || IR_val[2] == 1 || IR_val[3] == 1 || IR_val[4] == 1 || IR_val[5] == 1 || IR_val[6] == 1 || IR_val[7] == 1){
      mdrive(120,-120);
      read_IR();
      }
    mdrive(0,0);delay(1000);
    //IR_val[0] != 1 && IR_val[1] != 1 && IR_val[2] != 1 && IR_val[3] != 1 && IR_val[4] != 1 && IR_val[5] != 1 && IR_val[6] != 1 && 
    read_IR();
    while (IR_val[3] != 1 && IR_val[4] != 1){
      Serial.println("N");
      PID_control("WallR");
      mdrive(ml + speedAdjust - speedAdjustw, mr - speedAdjust + speedAdjustw);
      read_IR();
      }
    }
  
  Serial.println("C");
  PID_control("Line");
  

  mdrive(ml + speedAdjust - speedAdjustw, mr - speedAdjust + speedAdjustw);

  if (IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1) {
    mdrive(100, 100);
    delay(100);
    read_IR();
    if (IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1) {
      delay(100);
      read_IR();
      if (IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1) {
        delay(100);
        read_IR();
        if (IR_val[0] == 1 && IR_val[1] == 1 && IR_val[2] == 1 && IR_val[3] == 1 && IR_val[4] == 1 && IR_val[5] == 1 && IR_val[6] == 1 && IR_val[7] == 1) {
          while (1) {
            delay(200);
            mdrive(0, 0);
          }
        } else {
          mdrive(-120, 120);
          delay(350);
        }
      }
    }
  }
}

void read_IR() {
  Serial.println("I");
  IR_val[0] = !digitalRead(IR1);
  IR_val[1] = !digitalRead(IR2);
  IR_val[2] = !digitalRead(IR3);
  IR_val[3] = !digitalRead(IR4);
  IR_val[4] = !digitalRead(IR5);
  IR_val[5] = !digitalRead(IR6);
  IR_val[6] = !digitalRead(IR7);
  IR_val[7] = !digitalRead(IR8);
}

void PID_control(const char* mode) {

  if (strcmp_P(mode, PSTR("Line")) == 0) {
  Serial.println("Line"); 
  error = 0;

  for (int i = 0; i < 8; i++) {
    error += IR_weights[i] * IR_val[i];
  }
  
  P = error;
  I = I + error;
  D = error - previousError;

  previousError = error;

  speedAdjust = Kp * P + Ki * I + Kd * D;
  speedAdjustw = 0;
  }

  else if (strcmp_P(mode, PSTR("WallL")) == 0){
    Serial.println("WallL"); 

    errorw = 0;
    

    Left_US = SonarL2.ping_cm();
    errorw = safety_distance - Left_US;
    
    Pw = errorw;
    Iw = Iw + errorw;
    Dw = errorw - previousErrorw;
  
    previousErrorw = errorw;
  
    speedAdjustw = Kpw * Pw + Kiw * Iw + Kdw * Dw;
    speedAdjust = 0;
  }

    else if (strcmp_P(mode, PSTR("WallR")) == 0){
    Serial.println("WallR"); 

    errorw = 0;
    
    Right_US = SonarR2.ping_cm();
    errorw = safety_distance - Right_US;
    
    Pw = errorw;
    Iw = Iw + errorw;
    Dw = errorw - previousErrorw;
  
    previousErrorw = errorw;
  
    speedAdjustw = Kpw * Pw + Kiw * Iw + Kdw * Dw;
    speedAdjust = 0;
   

    }
  
}

void mdrive(int ml, int mr) {
  if (ml > 0) {
    if (ml > 255) {
      ml = 255;
    }
    digitalWrite(LMotorA, HIGH);
    digitalWrite(LMotorB, LOW);
    analogWrite(LMotorPWM, ml);
  } else {
    if (ml < -255) {
      ml = -255;
    }
    digitalWrite(LMotorA, LOW);
    digitalWrite(LMotorB, HIGH);
    analogWrite(LMotorPWM, -1 * ml);
  }
  if (mr > 0) {
    if (mr > 255) {
      mr = 255;
    }
    digitalWrite(RMotorA, HIGH);
    digitalWrite(RMotorB, LOW);
    analogWrite(RMotorPWM, mr);
  } else {
    if (mr < -255) {
      mr = -255;
    }
    digitalWrite(RMotorA, LOW);
    digitalWrite(RMotorB, HIGH);
    analogWrite(RMotorPWM, -1 * mr);
  }
}
void read_US(){
  Us_val[0] = SonarL1.ping_cm();
  Serial.println(Us_val[0]);
//  Us_val[1] = SonarL2.ping_cm();
//  Serial.println(Us_val[1]);
  Us_val[1] = SonarR1.ping_cm();
  Serial.println(Us_val[1]);
//  Us_val[3] = SonarR2.ping_cm();
//  Serial.println(Us_val[3]);
  }
