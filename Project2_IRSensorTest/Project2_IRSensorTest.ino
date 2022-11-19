#include <IRremote.h>

int RECV_PIN = 7;

IRrecv irrecv(RECV_PIN);

decode_results results;

void setup()
{
  Serial.begin(9600);
  // In case the interrupt driver crashes on setup, give a clue
  // to the user what's going on.
  Serial.println("Enabling IRin");
  irrecv.enableIRIn(); // Start the receiver
  Serial.println("Enabled IRin");
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
}

void loop()
{
  
  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    irrecv.resume(); // Receive the next value
    
    if(results.value == 0xFF629D){
      //forward
      move(1,0,1,0);
    }
    else if (results.value == 0xFFC23D){
      //right
      move(0,1,1,0);
    }
    else if (results.value == 0xFF22DD){
      //left
      move(1,0,0,1);
    }
    else if (results.value == 0xFFA857){
     //back
      move(0,1,0,1);
    }
    else if (results.value == 0xFF02FD) {
      //stop
      move(0,0,0,0);
    }
  }
  delay(100);
  
}

void move(int m1a, int m1b, int m2a, int m2b){
  digitalWrite(2,m1a);
  digitalWrite(3,m1b);
  digitalWrite(4,m2a);
  digitalWrite(5,m2b);
}
