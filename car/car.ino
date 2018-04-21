#include <SoftwareSerial.h>

#define BT_VCC 8
#define BT_TX  10
#define BT_RX  11

SoftwareSerial mySerial(BT_TX, BT_RX); //tx rx

#define RIGHT_A 7
#define RIGHT_B 5
#define RIGHT_E 6

#define LEFT_A 4
#define LEFT_B 2
#define LEFT_E 3

void setup() {
  // put your setup code here, to run once:
  pinMode(BT_VCC, OUTPUT);
  digitalWrite(BT_VCC, HIGH);

  mySerial.begin(9600);
  
  pinMode(RIGHT_A, OUTPUT);
  pinMode(RIGHT_B, OUTPUT);
  pinMode(RIGHT_E, OUTPUT);
  
  pinMode(LEFT_A, OUTPUT);
  pinMode(LEFT_B, OUTPUT);
  pinMode(LEFT_E, OUTPUT);

  digitalWrite(RIGHT_A, LOW);
  digitalWrite(RIGHT_B, LOW);
  digitalWrite(RIGHT_E, HIGH);

  digitalWrite(LEFT_A, LOW);
  digitalWrite(LEFT_B, LOW);
  digitalWrite(LEFT_E, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(mySerial.available()>0)
  {
    int state = mySerial.read();
    if(state==1)
    {
        stopRight();
        stopLeft();
    }
    else if(state==2)
    {
      stopLeft();
      startRight();
    }
    else
    {
      stopRight();
      startLeft();
    }
  }
}

void startRight()
{
  digitalWrite(RIGHT_A, HIGH);
}

void stopRight()
{
  digitalWrite(RIGHT_A, LOW);
}

void startLeft()
{
  digitalWrite(LEFT_A, HIGH);
}

void stopLeft()
{
  digitalWrite(LEFT_A, LOW);
}

