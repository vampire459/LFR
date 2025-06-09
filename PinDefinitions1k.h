#define S1 10//A7
#define S2 11//A6
#define S3 A5
#define S4 A4
#define S5 A3
#define S6 A2
#define S7 A1
#define S8 A0

#define TrigPin 12
#define EchoPin 13

#define LED 2

#define LEFT_MOTOR_PWM 3
#define LEFT_MOTOR1 4//5 // Left motor pin 1
#define LEFT_MOTOR2 5//4 // Left motor pin 2
#define stby 6// 11
#define RIGHT_MOTOR1 7 // Right motor pin 1
#define RIGHT_MOTOR2 8 // Right motor pin 2
#define RIGHT_MOTOR_PWM 9 //6

int customADC(int a)
{
    if(a>490)   return 1;
    return 0;
}

void pinSetup(){
  pinMode(S1,INPUT);
  pinMode(S2,INPUT);
  pinMode(S3,INPUT);
  pinMode(S4,INPUT);
  pinMode(S5,INPUT);
  pinMode(S6,INPUT);
  pinMode(S7,INPUT);
  pinMode(S8,INPUT);
  pinMode(LEFT_MOTOR_PWM,OUTPUT);
  pinMode(RIGHT_MOTOR_PWM,OUTPUT);
  pinMode(LEFT_MOTOR1, OUTPUT);
  pinMode(LEFT_MOTOR2, OUTPUT);
  pinMode(RIGHT_MOTOR1, OUTPUT);
  pinMode(RIGHT_MOTOR2, OUTPUT);
  pinMode(stby,OUTPUT);
}