#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();


char receivedChar;
boolean newData = false;
//-----------------------------Motor Pins-------------------------------------
//Motor A
#define motorPin1 6  // Pin 14 of L293
#define motorPin2 5  // Pin 10 of L293

#define enB 2

#define motorPin3 4  // Pin 14 of L293
#define motorPin4 3  // Pin 10 of L293

#define enA 7

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);

}


void setup() 
{
  Serial.begin(115200);

  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
  
  Serial.println("LSM9DS1 data read demo");
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, see above!
  setupSensor();

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enA, OUTPUT);

  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(enB, OUTPUT);
}


double deg_turned = 0;

void loop() 
{
  lsm.read();  /* ask it to read in the data */ 

  

  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp); 

//  Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
//  Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
//  Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");

//  Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" gauss");
//  Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" gauss");
//  Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" gauss");

//  Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" dps");
//  Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" dps");
// Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" dps");


  if((g.gyro.z <230 && g.gyro.z > -230)) // && (g.gyro.z > -.5 || g.gyro.z < -1.5))
    deg_turned = deg_turned + ((g.gyro.z + 1)/5);

  if( deg_turned < 90)
  {
    forwardRight(100);
  }
  else
  {
    stopRight();
    delay(10000);
  }


  Serial.println(deg_turned);
  delay(200);
}


//---------------------------------Recieve Info---------------------------------

void recvInfo() {
  
  if (Serial.available() > 0) {
    
    receivedChar = Serial.read();
    newData = true;
    
  }
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
