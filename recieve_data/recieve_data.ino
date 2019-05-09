char receivedChar;
boolean newData = false;

#define y 50
#define g 48

void setup(){
  
  Serial.begin(9600);
  
  pinMode(50, OUTPUT);
  pinMode(48, OUTPUT);
  
}

void loop() {
  
  recvInfo();
  lightLED();
  //digitalWrite(121, HIGH);
  //delay(1000);
  //digitalWrite(y, LOW);
  //delay(1000);
  
}

void recvInfo() {
  
  if (Serial.available() > 0) {
    
    receivedChar = Serial.read();
    newData = true;
    
  }
}

void lightLED() {
  
  char led = receivedChar;
  
  while(newData ==true) {
    
    Serial.println(led);
    
    if (led == 'y') {
      digitalWrite(y, HIGH);
      delay(2000);
      digitalWrite(y, LOW);
      delay(2000);
    }
    else if(led == 'g') {
      digitalWrite(g, HIGH);
      delay(2000);
      digitalWrite(g, LOW);
      delay(2000);
    }

    
    newData = false;
    
  }
}

