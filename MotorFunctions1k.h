#include <PinDefinitions1k.h>

void Right(int SPD)
{
    // Serial.println("Right-->");
    analogWrite(LEFT_MOTOR_PWM,SPD);
    analogWrite(RIGHT_MOTOR_PWM,SPD);
    digitalWrite(LEFT_MOTOR1,HIGH);
    digitalWrite(LEFT_MOTOR2,LOW);
    digitalWrite(RIGHT_MOTOR1,LOW);
    digitalWrite(RIGHT_MOTOR2,HIGH);
    // delay(10);
}

void Left(int SPD)
{
    // Serial.println("<--Left");    
    analogWrite(LEFT_MOTOR_PWM,SPD);
    analogWrite(RIGHT_MOTOR_PWM,SPD);
    digitalWrite(RIGHT_MOTOR1,HIGH);
    digitalWrite(RIGHT_MOTOR2,LOW);
    digitalWrite(LEFT_MOTOR1,LOW);
    digitalWrite(LEFT_MOTOR2,HIGH);
    // delay(10);
}

void MotorStop()
{
    analogWrite(LEFT_MOTOR_PWM,0);
    analogWrite(RIGHT_MOTOR_PWM,0);
    digitalWrite(LEFT_MOTOR1,HIGH);
    digitalWrite(RIGHT_MOTOR2,HIGH);
    digitalWrite(LEFT_MOTOR2,HIGH);
    digitalWrite(RIGHT_MOTOR1,HIGH);   
    delay(5000);   
}

void Forward(int lms, int rms)
{
    analogWrite(LEFT_MOTOR_PWM,lms);
    analogWrite(RIGHT_MOTOR_PWM,rms);
    digitalWrite(LEFT_MOTOR1,HIGH);
    digitalWrite(LEFT_MOTOR2,LOW);
    digitalWrite(RIGHT_MOTOR1,HIGH);
    digitalWrite(RIGHT_MOTOR2,LOW);      
}