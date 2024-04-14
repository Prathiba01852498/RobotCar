/********************************************************************************************
*********************************************************************************************
Lab 3
April 8, 2024
Prathiba
Cheryl
Salvador
This code will combine the dc motor code and fft code we've been working on
to make a two wheeled robot move around a trash can
and make the robot stop when it detects a certain peak frequency
and make the robot start when it detects another specific peak frequency

Taken from arduinoFFT github (code) written by Enrique Condes
*********************************************************************************************
*********************************************************************************************/
// FFT library
#include "arduinoFFT.h"

// including to use object detecting sensor
#include "SR04.h"
// Goes to control of motor connected to robot's front facing left wheel
#define ENABLE_LEFT_WHEEL 14
// DIR_A LOW and DIR_B HIGH makes wheel move forward, use opposite logic to move backward
#define DIR_A 16
#define DIR_B 15
// Goes to control of motor connected to robot's front facing right wheel
#define ENABLE_RIGHT_WHEEL 21
// DIR_C LOW and DIR_D HIGH makes wheel move forward, use opposite logic to move backward
#define DIR_C 20
#define DIR_D 19

// used for object detecting and measuring distance from object
#define TRIG_PIN 12
#define ECHO_PIN 11
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);

// FFT define
#define CHANNEL A7
const uint16_t samples = 64; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 880; //Hz, must be less than 10000 due to ADC // twice of ths signal frequency
unsigned int sampling_period_us;
unsigned long microseconds;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

/* Create FFT object */
arduinoFFT FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency);

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03


long a;
int i;
int noteFreq;
bool start = true;
bool stop = false;

void setup() {
  //---set pin direction
  pinMode(ENABLE_LEFT_WHEEL, OUTPUT);
  pinMode(ENABLE_RIGHT_WHEEL, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(DIR_C, OUTPUT);
  pinMode(DIR_D, OUTPUT);

  // FFT code
  init_Sound_Sensor();
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Ready");
  
}

void init_Sound_Sensor() //Sound sampling done using FFT
{
   sampling_period_us = round(1000000*(1.0/samplingFrequency));  //sampling_period_us = round(1000000*(1.0/samplingFrequency));
}

int sound_sampling_fft()
{
   /*SAMPLING*/
  microseconds = micros();
  for(int i=0; i<samples; i++)
  {
      vReal[i] = analogRead(CHANNEL);
      vImag[i] = 0;
      while(micros() - microseconds < sampling_period_us){
        //empty loop
      }
      microseconds += sampling_period_us;
  }
  PrintVector(vReal, samples, SCL_TIME);/*  the results of the sampling according to time */
  FFT.Windowing(FFTWindow::Hamming, FFTDirection::Forward);	/* Weigh data */
  PrintVector(vReal, samples, SCL_TIME);
  FFT.Compute(FFTDirection::Forward); /* Compute FFT */
  PrintVector(vReal, samples, SCL_INDEX);
  PrintVector(vImag, samples, SCL_INDEX);
  FFT.ComplexToMagnitude(); /* Compute magnitudes */
  PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
  double x = FFT.MajorPeak();
  Serial.println(x, 6); //Print out what frequency is the most dominant.
 
  //  Detect sound is 440hz and would be acceptable +/- 2%
  if((x >= 431.2) && (x <= 448.8))    
  {
    noteFreq = 440;
    start = false;
    stop = true;
    return noteFreq; // 440Hz is the stopping frequency
  }
  else if ((x >= 257) && (x < 267))
  {
    noteFreq = 261;
    start = true;
    stop = false;
    return noteFreq; // 261 is the starting frequency
  }
  else {
    noteFreq = 1;
    return noteFreq;
  }

}

bool sound_status;

// both motors 100% duty cycle
void motor_duty_cycle_100() {
  analogWrite(ENABLE_LEFT_WHEEL, 255);
  analogWrite(ENABLE_RIGHT_WHEEL, 255);
}

// both motors 75% duty cycle
void motor_duty_cycle_75() {
  analogWrite(ENABLE_LEFT_WHEEL, 190);
  analogWrite(ENABLE_RIGHT_WHEEL, 190);
}

// both motors 50% duty cycle
void motor_duty_cycle_50() {
  analogWrite(ENABLE_LEFT_WHEEL, 127);
  analogWrite(ENABLE_RIGHT_WHEEL, 127);
}

// both motors 25% duty cycle
void motor_duty_cycle_25() {
  analogWrite(ENABLE_LEFT_WHEEL, 64);
  analogWrite(ENABLE_RIGHT_WHEEL, 64);
}

// left motor 100% duty cycle, right motor 25% duty cycle
void motor_duty_cycle_100_25() {
  analogWrite(ENABLE_LEFT_WHEEL, 255);
  analogWrite(ENABLE_RIGHT_WHEEL, 64);
}

// left motor 100% duty cycle, right motor 50% duty cycle
void motor_duty_cycle_100_50() {
  analogWrite(ENABLE_LEFT_WHEEL, 255);
  analogWrite(ENABLE_RIGHT_WHEEL, 127);
}

// left motor 100% duty cycle, right motor 75% duty cycle
void motor_duty_cycle_100_75() {
  analogWrite(ENABLE_LEFT_WHEEL, 255);
  analogWrite(ENABLE_RIGHT_WHEEL, 190);
}

// left motor 25% duty cycle, right motor 100% duty cycle
void motor_duty_cycle_25_100() {
  analogWrite(ENABLE_LEFT_WHEEL, 64);
  analogWrite(ENABLE_RIGHT_WHEEL, 255);
}

// left motor 50% duty cycle, right motor 100% duty cycle
void motor_duty_cycle_50_100() {
  analogWrite(ENABLE_LEFT_WHEEL, 127);
  analogWrite(ENABLE_RIGHT_WHEEL, 255);
}

// left motor 75% duty cycle, right motor 100% duty cycle
void motor_duty_cycle_75_100() {
  analogWrite(ENABLE_LEFT_WHEEL, 190);
  analogWrite(ENABLE_RIGHT_WHEEL, 255);
}

// right wheel will go cw and left wheel will go ccw
void wheels_forward() {
  digitalWrite(DIR_A, LOW);
  digitalWrite(DIR_B, HIGH);
  digitalWrite(DIR_C, HIGH);
  digitalWrite(DIR_D, LOW);
}

// right wheel will go ccw and left wheel will cw
void wheels_reverse() {
  digitalWrite(DIR_A, HIGH);
  digitalWrite(DIR_B, LOW);
  digitalWrite(DIR_C, LOW);
  digitalWrite(DIR_D, HIGH);
}

// both wheels will go cw
void wheels_clockwise() {
  digitalWrite(DIR_A, HIGH);
  digitalWrite(DIR_B, LOW);
  digitalWrite(DIR_C, HIGH);
  digitalWrite(DIR_D, LOW);
}

// both wheels will go ccw
void wheels_counter_clockwise() {
  digitalWrite(DIR_A, LOW);
  digitalWrite(DIR_B, HIGH);
  digitalWrite(DIR_C, LOW);
  digitalWrite(DIR_D, HIGH);
}

// robot will move forward at the predetermined acceptable speeds
void car_forward(int duty_cycle) {
  switch (duty_cycle) {
    case 100:
      // full power forward
      motor_duty_cycle_100();
      wheels_forward();
    case 75:
      // 3/4 power forward
      motor_duty_cycle_75();
      wheels_forward();
    case 50:
      // 1/2 power forward
      motor_duty_cycle_50();
      wheels_forward();
    case 25:
    // full speed forward
      motor_duty_cycle_25();
      wheels_forward();
  }
}

// car will move backward at the predetermined acceptable speeds
void car_reverse(int duty_cycle) {
  switch (duty_cycle) {
    case 100:
      // full power forward
      motor_duty_cycle_100();
      wheels_reverse();
    case 75:
      // 3/4 power forward
      motor_duty_cycle_75();
      wheels_reverse();
    case 50:
      // 1/2 power forward
      motor_duty_cycle_50();
      wheels_reverse();
    case 25:
    // full speed forward
      motor_duty_cycle_25();
      wheels_reverse();
  }
}

// car will stop by forcing logic out pins to motor drivers low
 void car_stop(){
  digitalWrite(DIR_A,LOW);
  digitalWrite(DIR_B,LOW);
  digitalWrite(DIR_C,LOW);
  digitalWrite(DIR_D,LOW);
}

// both wheels move forward, and they can move at diffeerent speeds to cause tilt with different duty cycles
// you can also move the car forward with not tilt and at different speeds by using the same duty cycle for both ints
void car_forward_tilt (int left_duty_cycle, int right_duty_cycle) {
  if (left_duty_cycle >= 0 && left_duty_cycle <= 255 && right_duty_cycle >= 0 && right_duty_cycle <= 255) {
    analogWrite(ENABLE_LEFT_WHEEL, left_duty_cycle);
    analogWrite(ENABLE_RIGHT_WHEEL, right_duty_cycle);
    wheels_forward();
  }
}

// both wheels move backward, and they can move at diffeerent speeds to cause tilt with different duty cycles
// you can also move the car backward with not tilt and at different speeds by using the same duty cycle for both ints
void car_backward_tilt (int left_duty_cycle, int right_duty_cycle) {
  if (left_duty_cycle >= 0 && left_duty_cycle <= 255 && right_duty_cycle >= 0 && right_duty_cycle <= 255) {
    analogWrite(ENABLE_LEFT_WHEEL, left_duty_cycle);
    analogWrite(ENABLE_RIGHT_WHEEL, right_duty_cycle);
    wheels_reverse();
  }
}

// wheels move clockwise at given duty cycles to turn robot left 
void car_turn_left (int left_duty_cycle, int right_duty_cycle) {
  if (left_duty_cycle >= 0 && left_duty_cycle <= 255 && right_duty_cycle >= 0 && right_duty_cycle <= 255) {
    analogWrite(ENABLE_LEFT_WHEEL, left_duty_cycle);
    analogWrite(ENABLE_RIGHT_WHEEL, right_duty_cycle);
    wheels_clockwise();
  }
}

// wheels move counter clockwise at given duty cycles to turn robot right 
void car_turn_right (int left_duty_cycle, int right_duty_cycle) {
  if (left_duty_cycle >= 0 && left_duty_cycle <= 255 && right_duty_cycle >= 0 && right_duty_cycle <= 255) {
    analogWrite(ENABLE_LEFT_WHEEL, left_duty_cycle);
    analogWrite(ENABLE_RIGHT_WHEEL, right_duty_cycle);
    wheels_counter_clockwise();
  }
}

void distance_measurement() {
  a=sr04.Distance();
  Serial.print(a);
  Serial.println("cm");
  delay(1000);
  }

// FFT printVector
void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
  break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
  break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
  break;
    }
    Serial.print(abscissa, 6);
    if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}

void loop() {

// TODO
// a finite state machine that constantly is checkinf distance from object and adjusting car and or individual wheel speeds to adjust and keep close to object
// detect two specific frequencies to start and stop car
// detect certain distances to finely adjust wheel speeds to stay within a range of object distance
  /*****************************
  ******************************
  states for robot moving
  state 0: stop
  state 1: moving forward 75% duty cycle
  state 2: moving left wheel duty cycle 100, right wheel duty cycle 50
  state 3: moving left wheel duty cycle 100, right wheel duty cycle 25
  state 4: 
  state 5: YELLOW
  state 6: RED
  state 7: RED FLASHING
  ******************************
  *****************************/
  // intilaize start-up
  int currentState;
  int nextState;
  nextState = 1;
  long distance;
  distance = sr04.Distance();
  stop = false;
  while (1) {
    currentState = nextState;
    switch (currentState){
      case 0: // come in here to stop car, in case of note C4
        // TODO
        // add a way to start robot with C4 with if statement checking a flag and changing nextState
        car_stop();
        break;
      case 1: // come here to start the car, in case of note A4
        car_forward_tilt(255, 255);
        break;
      case 2: // come here in case getting too close to object
        car_forward_tilt(255, 150);
        break;
      case 3: // come here in case getting too far from object
        car_forward_tilt(130, 255);
        break;
      case 4: // come here if way too close to object
        //car_stop();
        car_turn_right(178, 0);
        //car_forward_tilt(255,150);
        break;
      case 5: // come here if way too far from object
        //car_stop();
        car_turn_left(0, 178);
        //car_forward_tilt(255, 255);
        break;
    }
    distance = sr04.Distance(); // distance measure in cm
    if ((distance >=  10) && (distance < 50)) { // getting a little far from object
      nextState = 3;
    }
    else if ((distance <= 5) && (distance > 2)) { // getting a little close to object
      nextState = 2;
    }
    else if (distance <= 2) { // got way too close to object
      nextState = 4;
    }
    else if (distance >= 50) { // got way too far from object
      nextState = 5;
    }
    else { // at a reasonable distance from object, continue at forward pace
    nextState = 1;
    }
    sound_sampling_fft();
    if (stop == true) {
      nextState = 0;
    }

  }
  
}




 
   
