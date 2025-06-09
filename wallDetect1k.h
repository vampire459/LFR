#include <Arduino.h>

#define TrigPin 12
#define EchoPin 13

//Distance in centimetre
bool checkWall(int cm)
{
    
    digitalWrite(TrigPin,LOW);
    delayMicroseconds(1);
    digitalWrite(TrigPin,HIGH);
    delayMicroseconds(10);
    digitalWrite(TrigPin,LOW);

    int d= pulseIn(EchoPin,HIGH)*0.017; //Distance in centimetre

    if (d<cm )   return true;
    else return false;
    
}