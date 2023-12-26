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

#define sound_digital 45
int sound;

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

  pinMode(sound_digital,INPUT);

}

void loop() {

  int sensorValue = analogRead(soundSensorPin); 

  
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
    int ml = 200;
    int mr = 200;
    
    mdrive(ml+speedAdjust, mr-speedAdjust);
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
