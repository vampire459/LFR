#include <Arduino.h>
#include <stdint.h>
#include <MotorFunctions1k.h>
// #include <ArduinoJson.h>
#include <string.h>
#include <wallDetect1k.h>

#define TX_DOC_MAX_DATA_LEN 192
#define BLUETOOTH_SERIAL Serial

#define WHITE_LINE_BLACK_TRACK 1
#define BLACK_LINE_WHITE_TRACK 0
#define SENSOR_COUNT 8
#define POSITIONAL_WEIGHT_W 100

#define OUT_OF_LINE_ERROR_VALUE 400
// #define OBJECT_DETECTED 6969
#define WHITE_LINE_BLACK_TRACK 1
#define BLACK_LINE_WHITE_TRACK 0

/*
Modifiables 
KP 459              _10     _10     _10     _24     _24     _40
KD  63              __0     __0     __2     _15     _15     _23
MOTOR_SPEED         _50     _80     _80     200     200     220   
State_Hold_Delay    200     200     200     _21     _21     _16
turn_Delay          _10     _10     _10     _10     __5     __5
SPEED_LIMITER       100     150     150     255     255     255  

// ool delay           _15     _15     _15     _15     __5     __5
// TURN_SPEED          _80     120     120     180     255     255   
*/
#define DEFAULT_KP 41 // proprtional gain 
#define DEFAULT_KD 59 // Derivative gain   
#define DEFAULT_MOTOR_SPEED 80 // Motor speed    
#define Default_State_Hold_Delay 15 // or LOOP DELAY - delay after which readings update as a whole
#define turn_Delay 5 // Duration over which bot turns, ~ (1/turning spd)
#define SPEED_LIMITER 120 // Speed Limit 
// #define OoL_Delay 5 // delay after which turns execute, ~ (sensor to wheel distance)
// #define TURN_SPEED 255 // Turn speed     

// This variable holds the value for the type of track.
int trackType = WHITE_LINE_BLACK_TRACK;



int KP = DEFAULT_KP;
int KD = DEFAULT_KD;
int MOTOR_SPEED = DEFAULT_MOTOR_SPEED;
int State_Hold_Delay = Default_State_Hold_Delay;

int P=0, D=0, prev_error=0,error_dir=0,turn_timer=0;
int timerr=0,brak=0,brakk=0;
bool flag =0;

double error=0, PID_Value=0;

void setup() {
    pinSetup();

  pinMode(stby,OUTPUT);
  digitalWrite(stby,HIGH);
  pinMode(LED,OUTPUT);


  BLUETOOTH_SERIAL.begin(115200);
}

void loop() {
// Serial.println(timerr++);
readSensors();
controlMotors();
//Turn_timer 3, Led timerr 15, Brakk 5, Brak 5 - all depends on LOOP DELAY
// If Loop Delay Changes, Change these accordingly...............
delay(State_Hold_Delay);
}


// The function below returns the sensor data in 8 byte.
uint8_t getSensorReadings()
{
    StaticJsonDocument<TX_DOC_MAX_DATA_LEN> txDoc;
    uint8_t reading = 0x00;

    uint8_t s1 = (uint8_t)digitalRead(S1); // leftmost channel
    uint8_t s2 = (uint8_t)digitalRead(S2);
    uint8_t s3 = (uint8_t)digitalRead(S3);
    uint8_t s4 = (uint8_t)digitalRead(S4);
    uint8_t s5 = (uint8_t)digitalRead(S5);
    uint8_t s6 = (uint8_t)digitalRead(S6);
    uint8_t s7 = (uint8_t)digitalRead(S7);
    uint8_t s8 = (uint8_t)digitalRead(S8); // rightmost channel
    reading = (s1 << 7) | (s2 << 6) | (s3 << 5) | (s4 << 4) | (s5 << 3) | (s6 << 2) | (s7 << 1) | s8;
    
    // #if BLUETOOTH_LOGGING_ENABLED == 1
        // char ss[16];
        // char output[TX_DOC_MAX_DATA_LEN];
        // sprintf(ss,"%d%d%d%d%d%d%d%d",s1,s2,s3,s4,s5,s6,s7,s8);
        // txDoc["di"] = String(ss);
        // serializeJson(txDoc,output);
        // BLUETOOTH_SERIAL.println(output);
    // #endif

    // Serial.print(s1); Serial.print(s2); Serial.print(s3); Serial.print(s4); Serial.print(s5); Serial.print(s6); Serial.print(s7); Serial.println(s8);

     if (reading == 0b00110000)                             trackType = WHITE_LINE_BLACK_TRACK;
   else if (reading == 0b00111000 || reading == 0b00010000) trackType = WHITE_LINE_BLACK_TRACK;
   else if (reading == 0b00011000)                          trackType = WHITE_LINE_BLACK_TRACK;
   else if (reading == 0b00011100 || reading == 0b00001000) trackType = WHITE_LINE_BLACK_TRACK;
   else if (reading == 0b00001100)                          trackType = WHITE_LINE_BLACK_TRACK;

    if (reading == 0b11001111)                          trackType = BLACK_LINE_WHITE_TRACK;
   else if (reading == 0b11000111 || reading == 0b11101111) trackType = BLACK_LINE_WHITE_TRACK;
   else if (reading == 0b11100111)                          trackType = BLACK_LINE_WHITE_TRACK;
   else if (reading == 0b11100011 || reading == 0b11110111) trackType = BLACK_LINE_WHITE_TRACK;
   else if (reading == 0b11110011)                          trackType = BLACK_LINE_WHITE_TRACK;
    // Incase of a black line with white background, invert the sensor
    // output using bitwise XOR operator
    return reading;
}

int getCalculatedError()
{
   uint8_t sensorReading = getSensorReadings();
    if (trackType == BLACK_LINE_WHITE_TRACK) sensorReading ^= 0b11111111;
   int numeratorSum = 0, denominatorSum = 0;

   // Assuming that the the MSB represents the index 0 of the array (left to right)
   for (int i = 0; i < SENSOR_COUNT; i++)
   {
       uint8_t sensorValue = ((sensorReading & (1 << (SENSOR_COUNT - 1 - i))) >> (SENSOR_COUNT - 1 - i));
       numeratorSum += (i + 1) * POSITIONAL_WEIGHT_W * (int)sensorValue;
       denominatorSum += sensorValue;
   }
   int errorr = 0;
   // Check if the denominator is 0, if yes return the fallback error.
   if (denominatorSum != 0)
       errorr = ((numeratorSum / (denominatorSum * (POSITIONAL_WEIGHT_W / 2))) - (SENSOR_COUNT + 1));
   return errorr;
}

void readSensors() {


    uint8_t sensorData = getSensorReadings();

   // left most sensor value
    int s1 = (sensorData & (1<<7))>>7;
   // rightmost sensor value
    int s8 = (sensorData & 0x01);

    if (s1 != s8)    error_dir = s1 - s8;
    // {
    // turn_timer=1;}// turn_timer for gaps


    //Checkpoint logic for Black line
    if(sensorData == 0b00000000 || sensorData == 0b10000000||sensorData==0b00000001 || sensorData==10000001 )
    {
        if ( trackType == BLACK_LINE_WHITE_TRACK) {
        digitalWrite(LED,HIGH);
        flag=1;
        brak++;
        //Stop logic    after 5 loops if still Black 
            if(getSensorReadings() == 0b00000000 && brak>6){    // 5 loops
                Serial.println("Brake: BLWT");
                MotorStop();
                delay(5000);
                digitalWrite(LED,LOW);  
                // PORTB |= 0b00100000;
                brak=0;  
            }
        }
    }
    else brak=0;

// Out_of_line logic for Black Line
   if (sensorData == 0b11111111 || ((digitalRead(S4)==1) & (digitalRead(S5)==1))) //Y section entry and exit    
    {
        if (trackType == BLACK_LINE_WHITE_TRACK)
       {          
           if (error_dir > 0)  // T section Logic          //problem for gapssssssssssccccccccccccccccccccccccccccccccc
               error = OUT_OF_LINE_ERROR_VALUE;
           else if (error_dir < 0)
               error = -1 * OUT_OF_LINE_ERROR_VALUE;
       }
    }

//Out_of_line logic for White Line
    if (sensorData == 0b00000000 )// Y section entry and exit  problem for gapsssssssss
    {
       if ( trackType == WHITE_LINE_BLACK_TRACK ){ 
           if (error_dir < 0)              // T Section Logic
               error = OUT_OF_LINE_ERROR_VALUE;
           else if (error_dir > 0)
               error = -1 * OUT_OF_LINE_ERROR_VALUE;
        }   
    }    
//Checkpoint logic for White Line
    if(sensorData == 0b11111111 || sensorData == 0b01111111||sensorData==0b11111110 || sensorData==01111110 )
    {
        if ( trackType == WHITE_LINE_BLACK_TRACK) {
        digitalWrite(LED,HIGH);
        flag=1;
        brakk++;
        //Stop logic    after 5 loops if still White 
        if(getSensorReadings() == 0b11111111 && brakk>2){
            Serial.println("Brake: WLBT");
            MotorStop();
            delay(500);  
            digitalWrite(LED,LOW);
            brakk=0;
        }            
       }
    }
    else brakk=0;


    
    //Checkpoint Led timer
    // if(flag) timerr++;
    // if(timerr>25)   
    // {
    //     digitalWrite(LED,LOW);
    //     timerr=0;
    //     flag=0;
    // }
// */
    // if(digitalRead(OBJECT_DETECTOR) == 1)
    // {
    //     error = OBJECT_DETECTED;
    // }

}

void controlMotors() {          
    // Serial.println(error); 

    //Right Turn
    if (error == OUT_OF_LINE_ERROR_VALUE)
    {
        // delay(OoL_Delay); 
        uint8_t sensorReadings = getSensorReadings();

       // The following function returns true if out of line
       while (isOutOfLine(sensorReadings))
       {
            // turns Right
            // Serial.println(" Right --->>>");
            Right(MOTOR_SPEED);
           sensorReadings = getSensorReadings();
        //    delay(turn_Delay);
       }
   }

   //Left Turn
   else if (error == (-1 * OUT_OF_LINE_ERROR_VALUE))
   {
        // delay(OoL_Delay);
        uint8_t sensorReadings = getSensorReadings();// decides OOL based on trackType
       while (isOutOfLine(sensorReadings)) // BL - 11111111 - OOL
       {                                   // WL - 00000000 - OOL
            // Serial.println(" <<<--- Left ");
            Left(MOTOR_SPEED);
           sensorReadings = getSensorReadings();
        //    delay(turn_Delay);
       }
       
   }

//     if (checkWall(20))
//     {
//        while (checkWall(20))
//        {
//             Right(MOTOR_SPEED);
//             // Serial.println("Wall Detected, turn right");
//             delay(700);
//        }
//    }

   //Straight Line 
   error = getCalculatedError();
   
   if(error<OUT_OF_LINE_ERROR_VALUE && error>(-OUT_OF_LINE_ERROR_VALUE))
   {
        // /*
        StaticJsonDocument<64> rxDoc;
        if(BLUETOOTH_SERIAL.available()){
            BLUETOOTH_SERIAL.flush();
            String data = BLUETOOTH_SERIAL.readStringUntil('\n');
            DeserializationError error = deserializeJson(rxDoc,data);
            if (error)
		    {
			    BLUETOOTH_SERIAL.print("E|deseriaize json failed : ");
			    BLUETOOTH_SERIAL.println(error.f_str());
			    return;
			    rxDoc.clear();
		    }
            KP = rxDoc["P"];
            KD = rxDoc["D"];
            MOTOR_SPEED = rxDoc["ms"];
            State_Hold_Delay = rxDoc["de"];
            rxDoc.clear();
        }
        // */

        // Use the calculated PID_value to as a difference in the motor
        // speeds depending on whether the value is negative or positive.        
        PID_Value = (KP * error)+ (KD * (error - prev_error));
        prev_error = error;
        
        // Set motor speed based on PID output
        int left_motor_speed = MOTOR_SPEED + PID_Value;
        int right_motor_speed = MOTOR_SPEED - PID_Value;

        if (left_motor_speed>SPEED_LIMITER)   left_motor_speed = SPEED_LIMITER;
        else if(left_motor_speed < 0) left_motor_speed = 0;
        if(right_motor_speed >SPEED_LIMITER) right_motor_speed = SPEED_LIMITER;
        else if(right_motor_speed < 0) right_motor_speed=0;

        Forward(left_motor_speed,right_motor_speed);
   }
}

bool isOutOfLine(uint8_t senRead)
{
    if(((trackType == BLACK_LINE_WHITE_TRACK) && (senRead==0b11111111)) || ((trackType == WHITE_LINE_BLACK_TRACK) && (senRead==0b00000000)))
        return true;
    else 
        return false;
}
































// #include <Arduino.h>
// #include <stdint.h>
// #include <wallDetect1k.h>
// #include <MotorFunctions1k.h>
// #include <ArduinoJson.h>
// #include <string.h>


// #define TX_DOC_MAX_DATA_LEN 192
// #define BLUETOOTH_SERIAL Serial

// #define WHITE_LINE_BLACK_TRACK 1
// #define BLACK_LINE_WHITE_TRACK 0
// #define SENSOR_COUNT 8
// #define POSITIONAL_WEIGHT_W 100

// #define OUT_OF_LINE_ERROR_VALUE 400
// // #define OBJECT_DETECTED 6969
// #define WHITE_LINE_BLACK_TRACK 1
// #define BLACK_LINE_WHITE_TRACK 0

// /*
// Modifiables 
// KP 459              _10     _10     _10     _24     _24     _40     _24     _41
// KD  63              __0     __0     __2     _15     _15     _23     _15     _59
// MOTOR_SPEED         _50     _80     _80     200     200     220     _60     150
// State_Hold_Delay    200     200     200     _21     _21     _16     _15     _15
// turn_Delay          _10     _10     _10     _10     __5     __5     __5     __3
// SPEED_LIMITER       100     150     150     255     255     255     100     255

// // ool delay           _15     _15     _15     _15     __5     __5
// // TURN_SPEED          _80     120     120     180     255     255   
// */
// #define DEFAULT_KP 41 // proprtional gain 
// #define DEFAULT_KD 59 // Derivative gain   
// #define DEFAULT_MOTOR_SPEED 180 //150// Motor speed    
// #define Default_State_Hold_Delay 15 // or LOOP DELAY - delay after which readings update as a whole
// // #define turn_Delay 3 // Duration over which bot turns, ~ (1/turning spd)
// #define SPEED_LIMITER 200 // Speed Limit 
// // #define OoL_Delay 5 // delay after which turns execute, ~ (sensor to wheel distance)
// // #define TURN_SPEED 255 // Turn speed     

// // This variable holds the value for the type of track.
// int trackType = WHITE_LINE_BLACK_TRACK;



// int KP = DEFAULT_KP;
// int KD = DEFAULT_KD;
// int MOTOR_SPEED = DEFAULT_MOTOR_SPEED;
// int State_Hold_Delay = Default_State_Hold_Delay;

// int P=0, D=0, prev_error=0,error_dir=0,turn_timer=0;
// int timerr=0,brak=0,brakk=0;
// bool flag =0;

// double error=0, PID_Value=0;

// void setup() {

//     pinSetup();

// //   pinMode(TrigPin,OUTPUT);
// //   pinMode(EchoPin,INPUT);

//   digitalWrite(stby,HIGH);
//   pinMode(LED,OUTPUT);

// //   BLUETOOTH_SERIAL.begin(115200);
//   Serial.begin(9600);
// }

// void loop() {
// // Serial.println(timerr++);
// readSensors();
// controlMotors();
// //Turn_timer 3, Led timerr 15, Brakk 5, Brak 5 - all depends on LOOP DELAY
// // If Loop Delay Changes, Change these accordingly...............
// delay(State_Hold_Delay);
// }


// // The function below returns the sensor data in 8 byte.
// uint8_t getSensorReadings()
// {
//     StaticJsonDocument<TX_DOC_MAX_DATA_LEN> txDoc;
//     uint8_t reading = 0x00;

//     uint8_t s1 = (uint8_t)digitalRead(S1); // leftmost channel
//     uint8_t s2 = (uint8_t)digitalRead(S2);
//     uint8_t s3 = (uint8_t)digitalRead(S3);
//     uint8_t s4 = (uint8_t)digitalRead(S4);
//     uint8_t s5 = (uint8_t)digitalRead(S5);
//     uint8_t s6 = (uint8_t)digitalRead(S6);
//     uint8_t s7 = (uint8_t)digitalRead(S7);
//     uint8_t s8 = (uint8_t)digitalRead(S8); // rightmost channel
//     reading = (s1 << 7) | (s2 << 6) | (s3 << 5) | (s4 << 4) | (s5 << 3) | (s6 << 2) | (s7 << 1) | s8;
    

//     // #if BLUETOOTH_LOGGING_ENABLED == 1
//         // char ss[16];
//         // char output[TX_DOC_MAX_DATA_LEN];
//         // sprintf(ss,"%d%d%d%d%d%d%d%d",s1,s2,s3,s4,s5,s6,s7,s8);
//         // txDoc["di"] = String(ss);
//         // serializeJson(txDoc,output);
//         // BLUETOOTH_SERIAL.println(output);
//     // #endif

//     // Serial.print(s1); Serial.print(s2); Serial.print(s3); Serial.print(s4); Serial.print(s5); Serial.print(s6); Serial.print(s7); Serial.println(s8);

//      if (reading == 0b00110000)                             trackType = WHITE_LINE_BLACK_TRACK;
//    else if (reading == 0b00111000 || reading == 0b00010000) trackType = WHITE_LINE_BLACK_TRACK;
//    else if (reading == 0b00011000)                          trackType = WHITE_LINE_BLACK_TRACK;
//    else if (reading == 0b00011100 || reading == 0b00001000) trackType = WHITE_LINE_BLACK_TRACK;
//    else if (reading == 0b00001100)                          trackType = WHITE_LINE_BLACK_TRACK;

//     if (reading == 0b11001111)                          trackType = BLACK_LINE_WHITE_TRACK;
//    else if (reading == 0b11000111 || reading == 0b11101111) trackType = BLACK_LINE_WHITE_TRACK;
//    else if (reading == 0b11100111)                          trackType = BLACK_LINE_WHITE_TRACK;
//    else if (reading == 0b11100011 || reading == 0b11110111) trackType = BLACK_LINE_WHITE_TRACK;
//    else if (reading == 0b11110011)                          trackType = BLACK_LINE_WHITE_TRACK;
//     // Incase of a black line with white background, invert the sensor
//     // output using bitwise XOR operator
//     return reading;
// }

// int getCalculatedError()
// {
//    uint8_t sensorReading = getSensorReadings();
//     if (trackType == BLACK_LINE_WHITE_TRACK) sensorReading ^= 0b11111111;
//    int numeratorSum = 0, denominatorSum = 0;

//    // Assuming that the the MSB represents the index 0 of the array (left to right)
//    for (int i = 0; i < SENSOR_COUNT; i++)
//    {
//        uint8_t sensorValue = ((sensorReading & (1 << (SENSOR_COUNT - 1 - i))) >> (SENSOR_COUNT - 1 - i));
//        numeratorSum += (i + 1) * POSITIONAL_WEIGHT_W * (int)sensorValue;
//        denominatorSum += sensorValue;
//    }
//    int errorr = 0;
//    // Check if the denominator is 0, if yes return the fallback error.
//    if (denominatorSum != 0)
//        errorr = ((numeratorSum / (denominatorSum * (POSITIONAL_WEIGHT_W / 2))) - (SENSOR_COUNT + 1));
//    return errorr;
// }

// void readSensors() {


//     uint8_t sensorData = getSensorReadings();

//    // left most sensor value
//     int s1 = (sensorData & (1<<7))>>7;
//    // rightmost sensor value
//     int s8 = (sensorData & 0x01);

//     if (s1 != s8)    error_dir = s1 - s8;
//     // {
//     // turn_timer=1;}// turn_timer for gaps


// // // Checkpoint logic for Black line
// //     if(sensorData == 0b00000000 || sensorData == 0b10000000||sensorData==0b00000001 || sensorData==10000001 )
// //     {
// //         if ( trackType == BLACK_LINE_WHITE_TRACK) {
// //         digitalWrite(LED,HIGH);
// //         flag=1;
// //         brak++;
// //         //Stop logic    after 3 loops if still Black 
// //             if(getSensorReadings() == 0b00000000 && brak>5){    // 5 loops
// //                 Serial.println("Brake: BLWT");
// //                 MotorStop();
// //                 delay(1000);
// //                 digitalWrite(LED,LOW);  
// //                 brak=0;  
// //             }
// //         }
// //     }
// //     else brak=0;
// // //Out_of_line logic for Black Line
// //    if (sensorData == 0b11111111 || ((digitalRead(S4)==1) & (digitalRead(S5)==1))) //Y section entry and exit    
// //     {
// //         if (trackType == BLACK_LINE_WHITE_TRACK)
// //        {          
// //            if (error_dir > 0)  // T section Logic          //problem for gapssssssssssccccccccccccccccccccccccccccccccc
// //                error = OUT_OF_LINE_ERROR_VALUE;
// //            else if (error_dir < 0)
// //                error = -1 * OUT_OF_LINE_ERROR_VALUE;
// //        }
// //     }


// //Out_of_line logic for White Line
//     if (sensorData == 0b00000000 )// Y section entry and exit  problem for gapsssssssss
//     {
//        if ( trackType == WHITE_LINE_BLACK_TRACK ){ 
//            if (error_dir < 0)              // T Section Logic
//                error = OUT_OF_LINE_ERROR_VALUE;
//            else if (error_dir > 0)
//                error = -1 * OUT_OF_LINE_ERROR_VALUE;
//         }   
//     }    
// //Checkpoint logic for White Line
//     if(sensorData == 0b11111111 || sensorData == 0b01111111||sensorData==0b11111110 || sensorData==01111110 )
//     {
//         if ( trackType == WHITE_LINE_BLACK_TRACK) {
//         digitalWrite(LED,HIGH);
//         flag=1;
//         brakk++;
//         //Stop logic    after 5 loops if still White 
//         if(getSensorReadings() == 0b11111111 && brakk>5){
//             Serial.println("Brake: WLBT");
//             MotorStop();
//             delay(500);  
//             digitalWrite(LED,LOW);
//             brakk=0;
//         }            
//        }
//     }
//     else brakk=0;
    


//     //Checkpoint Led timer
//     // if(flag) timerr++;
//     // if(timerr>25)   
//     // {
//     //     digitalWrite(LED,LOW);
//     //     timerr=0;
//     //     flag=0;
//     // }

// }

// void controlMotors() {          
//     Serial.println(error); 
//     // object detected
    
         
//     //Right Turn
//     if (error == OUT_OF_LINE_ERROR_VALUE)
//     {
//         // delay(OoL_Delay); 
//         uint8_t sensorReadings = getSensorReadings();// The following function returns true if out of line
//        while (isOutOfLine(sensorReadings))
//        {
//             // turns Right
//             // Serial.println(" Right --->>>");
//             Right(MOTOR_SPEED);
//            sensorReadings = getSensorReadings();
//         //    delay(turn_Delay);
//        }
//    }

//    //Left Turn
//    else if (error == -1*OUT_OF_LINE_ERROR_VALUE )
//    {
//         // delay(OoL_Delay);
//         uint8_t sensorReadings = getSensorReadings();// decides OOL based on trackType
//        while (isOutOfLine(sensorReadings)) // BL - 11111111 - OOL
//        {                                   // WL - 00000000 - OOL
//             // Serial.println(" <<<--- Left ");
//             Left(MOTOR_SPEED);
//            sensorReadings = getSensorReadings();
//         //    delay(turn_Delay);
//        }
       
//    }
    
// //     if (checkWall(25))
// //     {
// //        while (checkWall(25))
// //        {
// //             Right(MOTOR_SPEED);
// //             // Serial.println("Wall Detected, turn right");
// //             delay(500);
// //        }
// //    }   

//    //Straight Line 
//    error = getCalculatedError();
   
   
//    if(error<OUT_OF_LINE_ERROR_VALUE && error< (-OUT_OF_LINE_ERROR_VALUE) )
//    {
//         // /*
//         StaticJsonDocument<64> rxDoc;
//         if(BLUETOOTH_SERIAL.available()){
//             BLUETOOTH_SERIAL.flush();
//             String data = BLUETOOTH_SERIAL.readStringUntil('\n');
//             DeserializationError error = deserializeJson(rxDoc,data);
//             if (error)
// 		    {
// 			    BLUETOOTH_SERIAL.print("E|deseriaize json failed : ");
// 			    BLUETOOTH_SERIAL.println(error.f_str());
// 			    return;
// 			    rxDoc.clear();
// 		    }
//             KP = rxDoc["P"];
//             KD = rxDoc["D"];
//             MOTOR_SPEED = rxDoc["ms"];
//             State_Hold_Delay = rxDoc["de"];
//             rxDoc.clear();
//         }
//         // */

//         // Use the calculated PID_value to as a difference in the motor
//         // speeds depending on whether the value is negative or positive.        
//         PID_Value = (KP * error)+ (KD * (error - prev_error));
//         prev_error = error;
        
//         // Set motor speed based on PID output
//         int left_motor_speed = MOTOR_SPEED + PID_Value;
//         int right_motor_speed = MOTOR_SPEED - PID_Value;

//         if (left_motor_speed>SPEED_LIMITER)   left_motor_speed = SPEED_LIMITER;
//         else if(left_motor_speed < 0) left_motor_speed = 0;
//         if(right_motor_speed >SPEED_LIMITER) right_motor_speed = SPEED_LIMITER;
//         else if(right_motor_speed < 0) right_motor_speed=0;

//         Forward(left_motor_speed,right_motor_speed);
//    }
// }

// bool isOutOfLine(uint8_t senRead)
// {
//     if(((trackType == BLACK_LINE_WHITE_TRACK) && (senRead==0b11111111)) || ((trackType == WHITE_LINE_BLACK_TRACK) && (senRead==0b00000000)))
//         return true;
//     else 
//         return false;
// }
