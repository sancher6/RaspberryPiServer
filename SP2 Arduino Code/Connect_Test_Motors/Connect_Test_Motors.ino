char receivedChar;
boolean newData = false;


//Motor A
#define motorPin1 6  // Pin 14 of L293
#define motorPin2 5  // Pin 10 of L293

#define enB 2

#define motorPin3 4  // Pin 14 of L293
#define motorPin4 3  // Pin 10 of L293

#define enA 7

void setup(){
  
  Serial.begin(9600);
  
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enA, OUTPUT);

  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(enB, OUTPUT);
  
}

void loop() {
  
  recvInfo();
  driveCar();
  
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

//---------------------------------Drive Wheels---------------------------------
void driveCar() {
  
  char led = receivedChar;
  
  while(newData ==true) {
    
    Serial.println(led);
    
    if (led == 'l'){
        forwardRight(100);
        stopLeft();
    }
    
    else if (led == 'r') {
        forwardLeft(100);
        stopRight();
    }
    
    else if (led == 'g') {
        forwardRight(100);
        forwardLeft(100);
    }
    
    else if (led == 's') {
        stopLeft();
        stopRight();
    }
 
    
    newData = false;
    
  }
}

