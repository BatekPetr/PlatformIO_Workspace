#include<Arduino.h>

/**
 * Bruno Santos, 2013
 * feiticeir0@whatgeek.com.pt
 * Small code to test DC motors - 2x with a L298 Dual H-Bridge Motor Driver
 * Free to share
 **/
 //Testing the DC Motors
//Define Pins
 //Motor A
 int enableA = 3;
 int pinA1 = 2;
 int pinA2 = 4;
//Motor B
 int enableB = 11;
 int pinB1 = 10;
 int pinB2 = 12;

void setup() {
 Serial.begin (9600);
 //configure pin modes
 pinMode (enableA, OUTPUT);
 pinMode (pinA1, OUTPUT);
 pinMode (pinA2, OUTPUT);
 pinMode (enableB, OUTPUT);
 pinMode (pinB1, OUTPUT);
 pinMode (pinB2, OUTPUT);
}
void loop() {
 //enabling motor A
 Serial.println ("Enabling Motor A");
 digitalWrite (enableA, HIGH);
//do something
 //forward
 Serial.println ("Forward");
 digitalWrite (pinA1, HIGH);
 digitalWrite (pinA2, LOW);
//5s forward
 delay (5000);
//reverse
 digitalWrite (pinA1,LOW);
 digitalWrite (pinA2,HIGH);
//5s backwards
 delay (5000);
//stop
 digitalWrite (enableA, LOW);
 delay (5000);
//enabling motor B
 //Since motor B is mounted reversed, PINs must be exchanged
 Serial.println ("Enabling Motor B");
 digitalWrite (enableB, HIGH);
//do something
 //forward
 Serial.println ("Forward");
 digitalWrite (pinB1, LOW);
 digitalWrite (pinB2, HIGH);
//5s forward
 delay (5000);
//reverse
 digitalWrite (pinB1,HIGH);
 digitalWrite (pinB2,LOW);
//5s backwards
 delay (5000);
//stop
 digitalWrite (enableB, LOW);
 delay (5000);
 }
