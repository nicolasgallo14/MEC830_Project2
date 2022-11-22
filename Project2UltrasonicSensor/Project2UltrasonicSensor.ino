#define enA 9
#define in1 4
#define in2 5
#define enB 10
#define in3 7
#define in4 6

#define trigPin 2
#define echoPin 3

int motorSpeedA = 0;
int motorSpeedB = 0;

void setup() {
  Serial.begin(9600);
   
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

 pinMode(trigPin, OUTPUT);
 pinMode(echoPin, INPUT);
}

void loop() {
 
 digitalWrite(trigPin, LOW);
 delayMicroseconds(10);
 digitalWrite(trigPin, HIGH);
 delayMicroseconds(10);
 digitalWrite(trigPin, LOW);
 int duration = pulseIn(echoPin, HIGH);
 int distanceCm = (duration * 0.0171);

 Serial.println(distanceCm);

  

if (distanceCm <15)
{
   motorSpeedA = 255;
   motorSpeedB = 255;
   
    // Set Motor A Forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // Set Motor B Backward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

     delay(400);
     
    // Set Motor A Forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // Set Motor B Forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

      delay(700);
      
   // Set Motor A Forward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    // Set Motor B backward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
       
    delay(400);
}


   else {

   motorSpeedA = 255;
   motorSpeedB = 255;
   
     // Set Motor A Forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // Set Motor B Forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    
   }
analogWrite(enA, motorSpeedA); // Send PWM signal to motor A
analogWrite(enB, motorSpeedB); // Send PWM signal to motor B
}
