//#define ENCODER_OPTIMIZE_INTERRUPTS

#include <Wire.h>
#include <SPI.h>
#include <Encoder.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h> 


#define trigPinFront A0
#define echoPinFront A1

#define trigPinBack A2
#define echoPinBack A3


#define trigPinLeft A5
#define echoPinLeft A4

#define trigPinRight A6
#define echoPinRight A7

#define sensor_ISR 19
#define raspberry_Counter 31

//Hall Sensors
#define hallA 2
#define hallB 3

#define hallC 20
#define hallD 21

//Motor A
#define motorPin1 8  // Pin 14 of L293
#define motorPin2 7  // Pin 10 of L293

#define enB 4

#define motorPin3 6  // Pin 14 of L293
#define motorPin4 5  // Pin 10 of L293

#define enA 9


Encoder rightEnc(hallA, hallB);
Encoder leftEnc(hallC, hallD);

unsigned long timer = 0;
unsigned long auto_time = 0;


Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();


volatile int isPressed = 0;
volatile int sendInfo = 0;

volatile byte vals[6];
volatile byte prev[6];

volatile int f_sense;
volatile int l_sense;
volatile int r_sense;
volatile int b_sense;

//Incoming Serial Comm--------------------

char receivedChar;
boolean newData = false;

//PI Globals--------------------------------------
char moving = 'n';
char man = 'y';


double previous_error_r = 0;
double integral_r = 0;
double deriv_r = 0;
int setpoint_speed_r = 300;
double k_pr = .25;
double k_ir = 2;
double k_dr = 0;

double previous_error_l = 0;
double integral_l = 0;
double deriv_l = 0;
int setpoint_speed_l = 300;
double k_pl = .5;
double k_il = 2;
double k_dl = 0;

boolean pid_test = true;

//------------------------------------------

void setup() {

  Serial.begin(9600);
  
  pinMode(sensor_ISR, INPUT);

  pinMode(raspberry_Counter,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(sensor_ISR), read_sensors_ISR, RISING);

  pinMode(trigPinFront,OUTPUT);
  pinMode(trigPinBack,OUTPUT);
  pinMode(trigPinLeft,OUTPUT);
  pinMode(trigPinRight,OUTPUT);

  pinMode(echoPinFront,INPUT);
  pinMode(echoPinBack,INPUT);
  pinMode(echoPinLeft,INPUT);
  pinMode(echoPinRight,INPUT);

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enA, OUTPUT);

  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(enB, OUTPUT);
  
  timer = millis();
  auto_time = millis();

}

long right_pos = -999;
long left_pos = -999;


void loop() {

  if (Serial.available() > 0) {
    Serial.print("Entered"); 
    receivedChar = Serial.read();
    newData = true;
    
  }

//  int t_step = 50;
//  int auto_timelimit = 5000;
//  
//  long new_right_pos, new_left_pos;
//  long right_speed, left_speed;
//
//  double error_r;
//  double pid_speed_r;
//  int error_l;
//  int pid_speed_l;
//  double output_r;
//  int output_l;
//
//  int enc_r;
//  int enc_l;

//  if(sendInfo == 1) {
//    
//    Serial.write((const char *)vals, sizeof(vals));
//    sendInfo = 0;
//  }

//  if (f_sense < 5){
//    man = 'n';
//    auto_time = millis();
//  }
//
//  if(millis() - auto_time >= auto_timelimit)
//      man = 'y';
//
//  if (millis() - timer >= t_step){
//    Serial.println(f_sense);
//    timer = millis();
//    new_right_pos = rightEnc.read();
//    new_left_pos = leftEnc.read();
//
//    enc_r = new_right_pos - right_pos;
//    vals[4] = new_right_pos - right_pos;
//    right_pos = new_right_pos;
//
//    enc_l = new_left_pos - left_pos;
//    vals[5] = new_left_pos - left_pos;
//    left_pos = new_left_pos;
//
//
//
//    if (moving == 'y') {
//    
//      error_r = setpoint_speed_r - enc_r;
//      integral_r = integral_r + (error_r*t_step*.001);
//      //deriv_r = (error_r - previous_error_r) / (t_step*.001);
//      output_r = (k_pr * error_r) + (k_ir* integral_r);// + (k_dr * deriv_r);
//      previous_error_r = error_r; 
//  
//      error_l = setpoint_speed_l - enc_l;
//      integral_l = integral_l + (error_l*t_step * .001);
//      //deriv_l = (error_l - previous_error_l) / t_step; 
//      output_l = (k_pl * error_l) + (k_il* integral_l);// + (k_dl * deriv_l);
//      previous_error_l = error_l; 
//  
//      forwardRight((int)output_r);
//      forwardLeft((int)output_l);
//
//    }
//
//    else if (moving == 'n') {
//      output_r = 0;
//      output_l = 0;
//      previous_error_r = 0;
//      integral_r = 0;
//      deriv_r = 0;
//      previous_error_l = 0;
//      integral_l = 0;
//      deriv_l = 0;
//
////      stopLeft();
////      stopRight();
//    }
//    
//  }
//


  recvInfo();
//  if (man == 'y')
    drive();

//  else if (man == 'n'){
//    stopLeft();
//    stopRight();
//  }

}
void read_sensors_ISR(){
  digitalWrite(trigPinFront,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinFront,HIGH);
  delayMicroseconds(10);
  f_sense = (pulseIn(echoPinFront,HIGH)/2)/29.1;
  
  digitalWrite(trigPinBack,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinBack,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinBack,LOW);
  b_sense = (pulseIn(echoPinBack,HIGH)/2)/29.1;
  
  digitalWrite(trigPinLeft,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinLeft,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinLeft,LOW);
  l_sense = (pulseIn(echoPinLeft,HIGH)/2)/29.1;
  
  digitalWrite(trigPinRight,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinRight,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinRight,LOW);
  r_sense = (pulseIn(echoPinRight,HIGH)/2)/29.1;

//  for(int i = 0; i < 6; i++)
//  {
//    if(vals[i] > 200)
//      vals[i] = 200;
//    else if(vals[i] == 0)
//    {
//      vals[i] = 240;
//    }
//
//    prev[i] = vals[i];
//  }

  
  sendInfo = 1;
}

//----------------------------------Forward Left--------------------------------

void forwardLeft(int speed){
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, HIGH);

  analogWrite(enB, speed);

}

//----------------------------------Backward Left--------------------------------

void backwardLeft(int speed){
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW);

  analogWrite(enB, speed);
}

//----------------------------------Stop Left--------------------------------

void stopLeft(){
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);

  analogWrite(enB, 0);
}

//----------------------------------Forward Right--------------------------------

void forwardRight(int speed){
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);

  analogWrite(enA, speed);

}

//----------------------------------Backward Right--------------------------------

void backwardRight(int speed){
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);

  analogWrite(enA, speed);
}

//----------------------------------Stop Right--------------------------------

void stopRight(){
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);

  analogWrite(enA, 0);
}

//---------------------------------RecvINFO----------------------------------

void recvInfo() {

  
  if (Serial.available() > 0) {
    
    receivedChar = Serial.read();
    newData = true;
    
  }
}

void drive(){
  char howToMove = receivedChar;

  int car_speed = 100; //change between 80-150

  while(newData == true){
    if (howToMove == 'f'){
      forwardLeft(car_speed+10);
      forwardRight(car_speed);
    }

    else if (howToMove == 's'){
      moving = 'n';
      stopLeft();
      stopRight();
    }

    else if (howToMove == 'l'){
      moving = 'n';
      stopLeft();
      forwardRight(car_speed);
    }

    else if (howToMove == 'r'){
      moving = 'n';
      stopRight();
      forwardLeft(car_speed+10);
    }

    else if (howToMove == 'b'){
      moving = 'n';
      backwardRight(car_speed);
      backwardLeft(car_speed+10);
    }

    else if (howToMove == 'e'){
      moving = 'n';
      stopLeft();
      stopRight();
    }

    
    newData = false;
  }
}
