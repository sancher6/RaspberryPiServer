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
/*
volatile long distance_F = 0;
volatile long distance_B = 0;
volatile long distance_L = 0;
volatile long distance_R = 0;*/

volatile byte vals[12];

volatile double gyro_x;
volatile double gyro_y;
volatile double gyro_z;

void setupSensor()
{
  /*
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  */

}

void setup() {

  Serial.begin(115200);
  
  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }

  attachInterrupt(digitalPinToInterrupt(sensor_ISR), read_sensors_ISR, RISING);

  pinMode(trigPinFront,OUTPUT);
  pinMode(trigPinBack,OUTPUT);
  pinMode(trigPinLeft,OUTPUT);
  pinMode(trigPinRight,OUTPUT);

  pinMode(echoPinFront,INPUT);
  pinMode(echoPinBack,INPUT);
  pinMode(echoPinLeft,INPUT);
  pinMode(echoPinRight,INPUT);

}

void loop() {
  if(sendInfo == 1) {
    
    Serial.write((const char *)vals);
    sendInfo = 0;
  }
}

void read_sensors_ISR(){
  byte val[2];

  digitalWrite(trigPinFront,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinFront,HIGH);
  delayMicroseconds(10);
  val[0] = (int)((pulseIn(echoPinFront,HIGH)/2)/29.1) & 255;
  val[1] = (int)((pulseIn(echoPinFront,HIGH)/2)/29.1) >> 8;
  
  vals[0] = (char)val[0];
  vals[1] = (char)val[1];
  
  digitalWrite(trigPinBack,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinBack,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinBack,LOW);
  val[0] = (int)((pulseIn(echoPinBack,HIGH)/2)/29.1) & 255;
  val[1] = (int)((pulseIn(echoPinBack,HIGH)/2)/29.1) >> 8;
  
  vals[2] = (char)val[0];
  vals[3] = (char)val[1];
  
  digitalWrite(trigPinLeft,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinLeft,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinLeft,LOW);
  vals[0] = (int)((pulseIn(echoPinLeft,HIGH)/2)/29.1) & 255;
  vals[1] = (int)((pulseIn(echoPinLeft,HIGH)/2)/29.1) >> 8;
  
  vals[4] = (char)val[0];
  vals[5] = (char)val[1];
  
  digitalWrite(trigPinRight,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinRight,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinRight,LOW);
  val[0] = (int)((pulseIn(echoPinRight,HIGH)/2)/29.1) & 255;
  val[1] = (int)((pulseIn(echoPinRight,HIGH)/2)/29.1) >> 8;
  
  vals[6] = (char)val[0];
  vals[7] = (char)val[1];
  
  
  vals[8] = 0;
  vals[9] = 10;

  
  vals[10] = 0;
  vals[11] = 10;
  
  sendInfo = 1;
}
