#include <PID_v1.h>
#include "Wire.h"
#include <MPU6050_light.h>

#define ENA 9
#define ENB 10
#define IN1 4
#define IN2 5
#define IN3 7
#define IN4 6

#define LT_R !digitalRead(12)
#define LT_M !digitalRead(13)
#define LT_L !digitalRead(11)

#define trigPin 2
#define echoPin 3

int carSpeedA ;
int carSpeedB ;


double Kp = 500, Ki = 300, Kd = 0.01;

double Setpoint, Input, Output, Zangle, angle, ZValue, Zlimit;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

MPU6050 mpu(Wire);
unsigned long timer = 0;

void END() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  Serial.println("Stop!");
}

void forward()
{
  carSpeedA = 150;
  carSpeedB = 150;

  // Set Motor A Forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  // Set Motor B Backward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, carSpeedA);
  analogWrite(ENB, carSpeedB);

  


}


void CCW()
{
  carSpeedA = 100;
  carSpeedB = 100;

  // Set Motor A Forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  // Set Motor B Backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, carSpeedA);
  analogWrite(ENB, carSpeedB);


}

void CW()
{
  carSpeedA = 100;
  carSpeedB = 100;
  // Set Motor A Forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  // Set Motor B Backward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, carSpeedA);
  analogWrite(ENB, carSpeedB);


}

void arc()
{
  carSpeedA = 90;
  carSpeedB = 200;
  // Set Motor A Forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  // Set Motor B Backward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, carSpeedA);
  analogWrite(ENB, carSpeedB);

Serial.print(Zangle);

}


void STOP()
{

  // Set Motor A Forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  // Set Motor B Backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, carSpeedA);
  analogWrite(ENB, carSpeedB);
  
Serial.print(Zangle);
}
void Avoidance()
{

STOP();

delay(500);

  while (Zangle < 60) {
    mpu.update();
  Zangle = mpu.getAngleZ();
    Serial.println("Turning LEFT1");
      Serial.print(Zangle);
  Serial.print(" ");
    CCW();
  }

STOP();

delay(500);

 arc();

 delay(3000);

 STOP();

delay(500);

  while (Zangle > 0) {
    mpu.update();
  Zangle = mpu.getAngleZ();
    Serial.println("Turning LEFt4");
      Serial.print(Zangle);
  Serial.print(" ");
    CCW();
  }

  STOP();

delay(500000);
}


void setup() {



  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(LT_R, INPUT);
  pinMode(LT_M, INPUT);
  pinMode(LT_L, INPUT);

  myPID.SetMode(AUTOMATIC);

  Serial.begin(9600);
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) { } // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");

  myPID.SetOutputLimits(-255, 255);
  Zangle = mpu.getAngleZ();
  Setpoint = 0;
}
void loop() {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(10);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  int duration = pulseIn(echoPin, HIGH);
  int distanceCm = (duration * 0.0171);

  // put your main code here, to run repeatedly:
  mpu.update();
  Zangle = mpu.getAngleZ();
//  Serial.print(distanceCm);
//  Serial.print(" ");
//  Serial.print(Input);
//  Serial.print(" ");
//  Serial.print(Setpoint);
//  Serial.print(" ");
//  Serial.print(Output);
//  Serial.print(" ");
  //Serial.print(Zangle);
 // Serial.print(" ");
//  Serial.print(ZValue);
//  Serial.print(" ");
//  Serial.print(carSpeedA);
//  Serial.print(" ");
//  Serial.print(" ");
//  Serial.print(carSpeedB);
//  Serial.print(" ");
//  Serial.print(Zlimit);
  // Serial.print(Forward);
  // Serial.print(" ");
  //Serial.print(Backward);
  //Serial.print(" ");
  Serial.println();
  Zlimit = constrain(Zangle, -5, 5);
  ZValue = map(Zlimit , -5, 5, -100, 100) ;
  Input = Zangle;
  //int right =  map(Output, 0, 255, 0, 200);
  //int left = map(Output, -255,0,200,0);
  myPID.Compute();

  if (Output > 5) //move right

  {

    carSpeedA = 90 + ZValue * -1 ;
    carSpeedB = 90;
    analogWrite(ENA, carSpeedA );
    analogWrite(ENB, carSpeedB );


    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);


  }
  else if (Output < -5) //move left

  {

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    carSpeedA = 90;
    carSpeedB = 90 + ZValue * 1;
    analogWrite(ENA, carSpeedA);
    analogWrite(ENB, carSpeedB);

  }
  else

  {
    Serial.println("Forward");
    analogWrite(ENA, 120);
    analogWrite(ENB, 120);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);



  }

  if (distanceCm < 25)
  {
    Serial.println("Avoidance");
     Avoidance();
      
  }

  if (LT_M & LT_R & LT_L) {
    END();
    Serial.println("DONE!!!!!!!");
  }

    //delay(400);
  

}
