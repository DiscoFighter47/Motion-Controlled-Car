#include <SoftwareSerial.h>

#define BT_VCC 8
#define BT_TX  10
#define BT_RX  11

SoftwareSerial mySerial(BT_TX, BT_RX); //tx rx

void setup() {
  // put your setup code here, to run once:
  pinMode(BT_VCC, OUTPUT);
  digitalWrite(BT_VCC, HIGH);
  
  mySerial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  mySerial.write(1);
  delay(2000);
  mySerial.write(2);
  delay(2000);

  mySerial.write(1);
  delay(2000);
  mySerial.write(3);
  delay(2000);
}
