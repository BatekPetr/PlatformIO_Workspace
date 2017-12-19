#include <Arduino.h>

int ledPin=2;     //define a pin for LED

void setup()
{
    //Serial.begin(9600);  //Begin serial communcation
    pinMode( ledPin, OUTPUT );
}

void loop()
{
    //Serial.println(analogRead(lightPin)); //Write the value of the photoresistor to the serial monitor.
    digitalWrite(ledPin, 1);
    delay(1000); // time in ms
    digitalWrite(ledPin, 0);
    delay(1000); // time in ms
}
