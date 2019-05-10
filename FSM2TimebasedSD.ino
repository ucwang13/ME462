#include <SPI.h>
#include <SD.h>
#include <Servo.h>

// Servo stuff
Servo servo1;

// SD Card
File dataFile;
boolean useSD = true;
const int chipSelect = 10;
float speedDifference;
unsigned long plotTime;


// Wheel speed variables
// IR sensor interrupt
const int interruptPin1 = 2;  // interrupt pullup
const int interruptPin2 = 3;  // interrupt pullup
// timer and velocity variables
volatile unsigned long previousTime1;
volatile unsigned long previousTime2;
volatile float velocity1;
volatile float velocity2;
volatile float outputVelocity1;
volatile float outputVelocity2;
volatile float previousOutputVelocity1;
volatile float previousOutputVelocity2;
// filter stuff
volatile float previousVelocity1;
volatile float previousVelocity2;
volatile float prepreviousVelocity1;
volatile float prepreviousVelocity2;

// Hbridge variables
const int in1F = 40;  // digital
const int in2F = 42;  // digital
const int in1B = 44;  // digital
const int in2B = 46;  // digital

// solenoid variables
const int solenoid1 = 31; // digital
const int solenoid2 = 30; // digital

// LED Pins
const int ledB = 32;  // digital
const int ledG = 34;  // digital
const int ledR = 36;  // digital

//FSM
// Assign "code" values to each state
const int READY = 101;
const int PULSE1  = 102;
const int PULSE2 = 103;
const int RESTORE1 = 104;
const int RESTORE2 = 105;

// Create a state variable and initialize it to READY
int state = READY;

// Control timers
int delayTime;

// Creat FSM input variables: Lock, timebased -> LP
int lock1 = 0;   // wheel1 is not locked
int lock2 = 0;   // wheel2 is not locked


// Setup
void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600);
  if(!SD.begin(chipSelect)) {
    useSD = false;
  }

  // Servo stuff
  servo1.attach(6); // PWM
  servo1.write(50);
  
  // Wheel speed details
  pinMode(interruptPin1, INPUT_PULLUP);
  pinMode(interruptPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin1), IRtick1, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), IRtick2, RISING);
  
  // set pin mode output
  // Hbridge
  pinMode(in1F, OUTPUT);
  pinMode(in2F, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);
  
  // Solenoid
  pinMode(solenoid1, OUTPUT);
  pinMode(solenoid2, OUTPUT);

  //LED
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);
  analogWrite(ledR, 255);
  analogWrite(ledB, 50);
  analogWrite(ledG, 255);
}


// Loop
void loop() {
  // put your main code here, to run repeatedly:

  //SD Card Storage
  if (useSD == true){
    dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (dataFile) {
      plotTime = millis();
      dataFile.print(plotTime);
      dataFile.print("\t");
      dataFile.print(outputVelocity1);
      dataFile.print("\t");
      dataFile.println(outputVelocity2);
      dataFile.close();
    }
  }
  
  // Finite State Machine Below
  switch(state) {
    //S0 idle ready and detecting
    case READY:
      if ((outputVelocity1 > 4) || (outputVelocity2 > 4)){
        analogWrite(ledR, 100);
        analogWrite(ledG, 100);
        analogWrite(ledB, 255);
      } else {
        analogWrite(ledR, 255);
        analogWrite(ledG, 255);
        analogWrite(ledB, 100);
      }
      
      CompareWheelSpeed();
      
      if (lock1 == 1){
        analogWrite(ledB, 255);
        analogWrite(ledR, 255);
        analogWrite(ledG, 100);
        state = PULSE1;
        delayTime = millis();
      }
      break;
      
    //S1 start ABS
    case PULSE1:
      servo1.write(62);
      digitalWrite(solenoid1, HIGH);
      // delay between solenoid retract and motor start. 
      if ((millis() - delayTime) > 50) {
        state = PULSE2;
        delayTime = millis();
      }
      break;

    case PULSE2:
      FCamMotorON();
      // duration cam motor is on. 
      if ((millis() - delayTime) > 4000){
        analogWrite(ledG, 255);
        analogWrite(ledR, 100);
        state = RESTORE1;
        delayTime = millis();
      }
      break;
      
    //S2 restore everything to intial state
    case RESTORE1:
      servo1.write(50);
      digitalWrite(solenoid1, LOW);
      // delay Time between solenoid insert and motor reverse
      if ((millis() - delayTime) > 1000) {
        state = RESTORE2;
        delayTime = millis();
      }
      break;

    case RESTORE2:
      // reverse to relax the motor
      FCamMotorReverse();
      // amount of time to reverse to release pressure on the solenoid. 
      if ((millis() - delayTime) > 100) {
        FCamMotorOFF();
        // resume interrupts
        analogWrite(ledR, 255);
        analogWrite(ledG, 255);
        analogWrite(ledB, 100);
        lock1 = 0;
        state = READY;
      }
  }
  
}


void CompareWheelSpeed() {
  if ((outputVelocity1 > 2) || (outputVelocity2 > 2)) {
    // When the speed difference between wheels is too great, activate
    if ((outputVelocity1 - outputVelocity2) < -0.777) {
      //digitalWrite(13, HIGH);
      lock1 = 1;
    // When the deceleration in either wheel is too greate, activate
    } else if ((outputVelocity1 < 0.5 * previousOutputVelocity1) || (outputVelocity2 < 0.5 * previousOutputVelocity2)) {
      //digitalWrite(13, HIGH);
      lock1 = 1;
    // If speed is high but none of the above, dormant
    } else {
      //digitalWrite(13, LOW);
      lock1 = 0;
    }
  // When Speed is too low, there is no need. 
  } else {
    //digitalWrite(13, LOW);
    lock1 = 0;
  }
}


// Motor movements
void FCamMotorON() {
  digitalWrite(in1F, HIGH);
  digitalWrite(in2F, LOW);
}

void FCamMotorOFF() {
  digitalWrite(in1F, LOW);
  digitalWrite(in2F, LOW);
}

void FCamMotorReverse() {
  digitalWrite(in1F, LOW);
  digitalWrite(in2F, HIGH);
}

// Interrupt functions
void IRtick1() {
  velocity1 = 54279.7/(micros() - previousTime1);
  previousOutputVelocity1 = outputVelocity1;
  if (velocity1 >= 1.3 * prepreviousVelocity1) {
    outputVelocity1 = prepreviousVelocity1;
  } else {
    outputVelocity1 = velocity1;
  }
  prepreviousVelocity1 = previousVelocity1;
  previousVelocity1 = velocity1;
  previousTime1 = micros();
}

void IRtick2() {
  velocity2 = 54279.7/(micros() - previousTime2);
  previousOutputVelocity2 = outputVelocity2;
  if (velocity2 >= 1.3 * prepreviousVelocity2) {
    outputVelocity2 = prepreviousVelocity2;
  } else {
    outputVelocity2 = velocity2;
  }
  prepreviousVelocity2 = previousVelocity2;
  previousVelocity2 = velocity2;
  previousTime2 = micros();
}
