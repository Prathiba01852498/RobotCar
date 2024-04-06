#include "SR04.h"
#define ENABLE 5
#define EN 8
#define DIRA 3
#define DIRB 4
#define DIRC 6
#define DIRD 7
#define TRIG_PIN 12
#define ECHO_PIN 11
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
long a;
int i;



void setup() {
  // put your setup code here, to run once:
pinMode(ENABLE,OUTPUT);
  pinMode(EN,OUTPUT);
  pinMode(DIRA,OUTPUT);
  pinMode(DIRB,OUTPUT);
  pinMode(DIRC,OUTPUT);
  pinMode(DIRD,OUTPUT);
  Serial.begin(9600);
}

 void carstop(){
  Serial.println("car stopped");
  digitalWrite(DIRA,LOW); 
  digitalWrite(DIRB,LOW);
  digitalWrite(DIRC,LOW); 
  digitalWrite(DIRD,LOW);
}

void car_halfspeed_forward(){
  Serial.println("car moving forward halfspeed");
  analogWrite(ENABLE,180);
  analogWrite(EN,180);

  for (i=0;i<5;i++) {
    digitalWrite(DIRA,HIGH);
    digitalWrite(DIRB,LOW);
    digitalWrite(DIRC,LOW);
    digitalWrite(DIRD,HIGH);
  }
}
void distance_measurement(){
  a=sr04.Distance();
  Serial.print(a);
  Serial.println("cm");
}
void loop() {
  // put your main code here, to run repeatedly:
   distance_measurement();
   if (a<=5){ //if distance is less than 5 cm car should stop
    carstop();
   }
   else{
    car_halfspeed_forward();
   }
}


