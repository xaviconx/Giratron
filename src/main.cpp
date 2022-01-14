#include <Arduino.h>

// ESP32 pins:
// PWM outputs
#define PWMPinA 25
#define PWMPinB 33

// Input buttons
#define CWButtonA 1
#define CCWButtonA 2
#define CWButtonB 3
#define CCWButtonB 4

// For H bridge
#define MotorA1 20
#define MotorA2 21
#define MotorB1 20
#define MotorB2 21

// For Encoders
#define MotorEncoderA1 20
#define MotorEncoderA2 21
#define MotorEncoderB1 20
#define MotorEncoderB2 21

// For potentiometers
#define PotA 22
#define PotB 22

// Aux variables
const bool CW = true;
const bool CCW = false;
volatile int contador1 = 0;
volatile int contador2 = 0;
int PWMA = 0;
int PWMB = 0;
int readA = 0;
int readB = 0;


// setting PWM properties
const int freq = 5000;
const int MotorAChannel = 0;
const int MotorBChannel = 1;
const int resolution = 10;

// Function Headers
void MotorA(bool,int);
void MotorB(bool,int);
void EncoderA();
void EncoderB();
void SerialUpdate();

void setup(){
    Serial.begin(115200);
    pinMode(MotorA1,OUTPUT);
    pinMode(MotorA2,OUTPUT);
    pinMode(MotorB1,OUTPUT);
    pinMode(MotorB2,OUTPUT);

    pinMode(CWButtonA,INPUT_PULLUP);
    pinMode(CCWButtonA,INPUT_PULLUP);
    pinMode(CWButtonB,INPUT_PULLUP);
    pinMode(CCWButtonB,INPUT_PULLUP);

    pinMode(PotA,INPUT);
    pinMode(PotB,INPUT);

    pinMode(MotorEncoderA2,INPUT);
    pinMode(MotorEncoderB2,INPUT);

    attachInterrupt(digitalPinToInterrupt(MotorEncoderA1), EncoderA, RISING);
    attachInterrupt(digitalPinToInterrupt(MotorEncoderB1), EncoderB, RISING);

    ledcSetup(MotorAChannel, freq, resolution);
    ledcSetup(MotorBChannel, freq, resolution);
    ledcAttachPin(PWMPinA, MotorAChannel);
    ledcAttachPin(PWMPinB, MotorBChannel);
}

void loop(){
    PWMA = map(analogRead(PotA),0,4096,0,1024);
    PWMB = map(analogRead(PotB),0,4096,0,1024);

    digitalRead(CWButtonA);
    digitalRead(CCWButtonA);
    digitalRead(CWButtonB);
    digitalRead(CCWButtonB);

    if((digitalRead(CWButtonA) == LOW)&&(digitalRead(CCWButtonA) == HIGH)){
        MotorA(CW,PWMA);
    } else if((digitalRead(CWButtonA) == HIGH)&&(digitalRead(CCWButtonA) == LOW)){
        MotorA(CCW,PWMA);
    }

    if((digitalRead(CWButtonB) == LOW)&&(digitalRead(CCWButtonB) == HIGH)){
        MotorB(CW,PWMB);
    } else if((digitalRead(CWButtonB) == HIGH)&&(digitalRead(CCWButtonB) == LOW)){
        MotorB(CCW,PWMB);
    }
}

void MotorA(bool Direction,int PWM){
  if(Direction == CW)
  {
    digitalWrite(MotorA1,LOW);
    digitalWrite(MotorA2,HIGH);
  }
  else
  {
    digitalWrite(MotorA1,HIGH);
    digitalWrite(MotorA2,LOW);
  }
  ledcWrite(MotorAChannel,PWM);
}

void MotorB(bool Direction, int PWM){
  if(Direction == CW)
  {
    digitalWrite(MotorB1,LOW);
    digitalWrite(MotorB2,HIGH);
  }
  else
  {
    digitalWrite(MotorB1,HIGH);
    digitalWrite(MotorB2,LOW);
  }
  ledcWrite(MotorBChannel,PWM);
}

void EncoderA(){
if(digitalRead(MotorEncoderA2)==LOW)
  contador1++;
else
  contador1--;
}

void EncoderB() {            
if(digitalRead(MotorEncoderB2)==HIGH)
  contador2++;
else
  contador2--;
}

void SerialUpdate(){
    Serial.print("Motor A: " );
    Serial.print(contador1);
    Serial.print(" Motor B: " );
    Serial.println(contador2);
}