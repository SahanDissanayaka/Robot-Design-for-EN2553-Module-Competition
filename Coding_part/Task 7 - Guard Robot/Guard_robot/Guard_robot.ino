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
//////////////////////////////////
#define USR1_Trig 38
#define USR1_Echo 39
#define USL1_Trig 37
#define USL1_Echo 31
#define USF_Trig 33
#define USF_Echo 35
#define max_distance_guard 80

NewPing SonarLG1(USL1_Trig,USL1_Echo,max_distance_guard);
NewPing SonarRG1(USR1_Trig,USR1_Echo,max_distance_guard);
NewPing SonarFG(USF_Trig,USF_Echo,max_distance_guard);

int Us_val[3] = {0,0,0};

bool guard_robot_detected = false;

int check1 = 0;
int check2 = 0;
bool going_left = false;
bool going_right = false;

int USL_dis;
int USR_dis; 

/////////////////////////////////
#define MAX_SPEED 240

int IR_val[8] = {0,0,0,0,0,0,0,0};
int pre_IR_val[8] = {0,0,0,0,0,0,0,0};
int IR_weights[8] = {-40,-15,-10,-5,5,10,15,40};

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

void read_IR();
void PID_control();
void mdrive();
void stop();
void turn_left();
void turn_right();



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
  pinMode(USR1_trig,OUTPUT);
  pinMode(USR1_Echo,INPUT);
  pinMode(USF_Trig,OUTPUT);
  pinMode(USF_Echo,INPUT);

}

void loop() {

  if(!guard_robot_detected){
  unsigned long start_time = millis();
  
  while (millis()-start_time <= 2000){
  
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
    USF_dis = SonarF.ping_cm();

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
      // should add a delay
      }
    else if (going_right){
      USR_dis = SonarR1.ping_cm();
      while (!(USR_dis <= 50 && USR_dis > 0)){
        USR_dis = SonarR1.ping_cm();
        delay(50);
        }
      USF_dis = SonarF.ping_cm();
      while (!(USF_dis <= 50 && USF_dis > 0)){
        USF_dis = SonarF.ping_cm();
        delay(50);
        }
       path_is_free = true;
       // should add a delay
        }
      }
// -------------------- path free check -------------------------- //    
  while True{
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
