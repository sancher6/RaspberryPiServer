#include <Wire.h>
#include <SPI.h>


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

volatile int isPressed = 0;
volatile int sendInfo = 0;

volatile long distance_F = 0;
volatile long distance_B = 0;
volatile long distance_L = 0;
volatile long distance_R = 0;


volatile double gyro_x;
volatile double gyro_y;
volatile double gyro_z;

unsigned long timer = 0;

//void setupSensor()
//{
//  /*
//  // 1.) Set the accelerometer range
//  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
//  
//  // 2.) Set the magnetometer sensitivity
//  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
//
//  // 3.) Setup the gyroscope
//  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
//  */
//
//}

void setup() {

  Serial.begin(9600);
  
//  while (!Serial) {
//    delay(1); // will pause Zero, Leonardo, etc until serial console opens
//  }

  //attachInterrupt(digitalPinToInterrupt(sensor_ISR), read_sensors_ISR, RISING);

  pinMode(trigPinFront,OUTPUT);
  pinMode(trigPinBack,OUTPUT);
  pinMode(trigPinLeft,OUTPUT);
  pinMode(trigPinRight,OUTPUT);
  pinMode(raspberry_Counter,OUTPUT);

  pinMode(echoPinFront,INPUT);
  pinMode(echoPinBack,INPUT);
  pinMode(echoPinLeft,INPUT);
  pinMode(echoPinRight,INPUT);

  timer = millis();

}

void loop() {
///

  int t_step = 100;

  if (millis()-timer >= t_step){
    timer = millis();
    read_sensors_ISR();

    Serial.print("Front: ");
    Serial.println(distance_F);
//    Serial.print("Back: ");
//    Serial.println(distance_B);
//    Serial.print("Left: ");
//    Serial.println(distance_L);
//    Serial.print("Right: ");
//    Serial.println(distance_R);
  }


}

void read_sensors_ISR(){
  digitalWrite(trigPinFront,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinFront,HIGH);
  delayMicroseconds(10);
  distance_F = (pulseIn(echoPinFront,HIGH)/2)/29.1;
  
  digitalWrite(trigPinBack,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinBack,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinBack,LOW);
  distance_B = (pulseIn(echoPinBack,HIGH)/2)/29.1;
  
  digitalWrite(trigPinLeft,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinLeft,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinLeft,LOW);
  distance_L = (pulseIn(echoPinLeft,HIGH)/2)/29.1;
  
  digitalWrite(trigPinRight,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinRight,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinRight,LOW);
  distance_R = (pulseIn(echoPinRight,HIGH)/2)/29.1;
  
  

  sendInfo = 1;
}
