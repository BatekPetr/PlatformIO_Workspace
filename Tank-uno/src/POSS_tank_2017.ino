#include <Servo.h>
#include <NewPing.h>
#include <PinChangeInterrupt.h>


// nastavení rychlosti motoru
#define MAX_RYCHLOST 200

int enableA = 3;
int pinA1 = 2;
int pinA2 = 4;
//Motor B
int enableB = 11;
int pinB1 = 10;
int pinB2 = 12;

int speed = MAX_RYCHLOST;

void setup (){
  // vytvor seriovy kanal do PC
  Serial.begin(9600);

  ///inicializace radice motoru
  pinMode (enableA, OUTPUT);
  pinMode (pinA1, OUTPUT);
  pinMode (pinA2, OUTPUT);
  pinMode (enableB, OUTPUT);
  pinMode (pinB1, OUTPUT);
  pinMode (pinB2, OUTPUT);
}

void loop () {

  dopredu();
  delay(3000);
  /*
  servoSonar.write(SERVO_MIN);
  delay(1000);
  servoSonar.write(SERVO_MAX);
  */
  doleva();
  delay(2000);
  zastavit();
  delay(1000);
}

void dopredu(){
  pravyMotorVpred(MAX_RYCHLOST);
  levyMotorVpred(MAX_RYCHLOST);
}

void dozadu(){
  pravyMotorVzad(MAX_RYCHLOST);
  levyMotorVzad(MAX_RYCHLOST);
}

void doleva(){
  pravyMotorVpred(MAX_RYCHLOST);
  levyMotorStop();
}

void ostreDoleva() {
  pravyMotorVpred(MAX_RYCHLOST);
  levyMotorVzad(MAX_RYCHLOST);
}

void doprava(){
  pravyMotorStop();
  levyMotorVpred(MAX_RYCHLOST);
}

void ostreDoprava() {
  pravyMotorVzad(MAX_RYCHLOST);
  levyMotorVpred(MAX_RYCHLOST);
}

void zastavit() {
  pravyMotorStop();
  levyMotorStop();
}

void pravyMotorVpred(int rychlost) {
  digitalWrite (pinA1, HIGH);
  digitalWrite (pinA2, LOW);

  analogWrite (enableA, speed);
}

void pravyMotorVzad(int rychlost) {
  digitalWrite (pinA1,LOW);
  digitalWrite (pinA2,HIGH);

  analogWrite (enableA, speed);
}

void pravyMotorStop() {
  digitalWrite(pinA1, HIGH);
  digitalWrite(pinA2, HIGH);

  digitalWrite (enableA, LOW);
}

void levyMotorVpred(int rychlost) {
  digitalWrite (pinB1, LOW);
  digitalWrite (pinB2, HIGH);

  analogWrite (enableB, speed);
}

void levyMotorVzad(int rychlost) {
  digitalWrite (pinB1, HIGH);
  digitalWrite (pinB2, LOW);

  analogWrite (enableB, speed);
}

void levyMotorStop() {
  digitalWrite(pinB1, HIGH);
  digitalWrite(pinB2, HIGH);

  digitalWrite (enableB, LOW);
}
