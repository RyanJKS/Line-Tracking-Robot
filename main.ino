/*  ODOMETRY FEEG2001
 *  GROUP C12
 */

#include <Wire.h>    // Calls for I2C bus library
#include <Stepper.h> // Calls for Stepper library

#define MD25ADDRESS 0x58  // Address of the MD25
#define SPEED1 0x00       // Byte to send speed to both motors for forward and backwards motion if operated in MODE 2 or 3 and Motor 1 Speed if in MODE 0 or 1
#define SPEED2 0x01       // Byte to send speed for turn speed if operated in MODE 2 or 3 and Motor 2 Speed if in MODE 0 or 1
#define ENCODERONE 0x02   // Byte to read motor encoder 1
#define ENCODERTWO 0x06   // Byte to read motor encoder 2
#define ACCELERATION 0xE  // Byte to define motor acceleration
#define CMD 0x10          // Byte to reset encoder values
#define MODE_SELECTOR 0xF // Byte to change between control MODES
#define STEPS 2038

Stepper stepper(STEPS, 8, 10, 9, 11); // Define the pins for the dropping mechanism

int Mode = 1;        // MODE in which the MD25 will operate selector value
float LW_Travel = 0; // Left Wheel travel distance variable
float RW_Travel = 0; // Right Wheel travel distance variable

// Experimentally Found Fudge Factors
float StraightCalib = 0.78;
// RightTurnCalib=
// LeftTurnCalib=

void forward(int dist)
{               // This function moves the platform forward by an input interger dist
    encoder1(); // Calls a function that reads value of encoder 1
    encoder2(); // Calls a function that reads value of encoder 2

    LW_Travel = StraightCalib * dist;
    RW_Travel = StraightCalib * dist;

    if (encoder1() <= LW_Travel && encoder2() <= RW_Travel)
    { // If statement to check the status of the traveled distance

        Wire.beginTransmission(MD25ADDRESS); // Sets the acceleration to register 1 (6.375s)
        Wire.write(ACCELERATION);
        Wire.write(3);
        Wire.endTransmission();

        Wire.beginTransmission(MD25ADDRESS); // Sets a constant speed for Left wheel
        Wire.write(SPEED1);
        Wire.write(30);
        Wire.endTransmission();

        Wire.beginTransmission(MD25ADDRESS); // Sets a constant speed for Right wheel
        Wire.write(SPEED2);
        Wire.write(30);
        Wire.endTransmission();

        forward(dist);
    }
}

void Right_Turn(int factor1)
{               // This function turns the platform right by an input float number that could be an offset from 90 degree angle
    encoder1(); // Calls a function that reads value of encoder 1
    encoder2(); // Calls a function that reads value of encoder 2
    LW_Travel = factor1 * 18;
    RW_Travel = factor1 * 18;

    if (encoder1() <= LW_Travel && encoder2() <= RW_Travel)
    { // If statement to check the status of the traveled distance

        Wire.beginTransmission(MD25ADDRESS); // Sets the acceleration to register 1 (6.375s)
        Wire.write(ACCELERATION);
        Wire.write(1);
        Wire.endTransmission();

        Wire.beginTransmission(MD25ADDRESS); // Sets a constant speed for Left wheel
        Wire.write(SPEED1);
        Wire.write(20);
        Wire.endTransmission();

        Wire.beginTransmission(MD25ADDRESS); // Sets a constant speed for Right wheel
        Wire.write(SPEED2);
        Wire.write(-20);
        Wire.endTransmission();

        Right_Turn(factor1);
    }
}

void Left_Turn(int factor2)
{               // This function turns the platform left by an input float number that could be an offset from 90 degree angle
    encoder1(); // Calls a function that reads value of encoder 1
    encoder2(); // Calls a function that reads value of encoder 2
    LW_Travel = factor2 * 18;
    RW_Travel = factor2 * 18;

    if (encoder1() <= LW_Travel && encoder2() <= RW_Travel)
    { // If statement to check the status of the traveled distance

        Wire.beginTransmission(MD25ADDRESS); // Sets the acceleration to register 1 (6.375s)
        Wire.write(ACCELERATION);
        Wire.write(1);
        Wire.endTransmission();

        Wire.beginTransmission(MD25ADDRESS); // Sets a constant speed for Left wheel
        Wire.write(SPEED1);
        Wire.write(-20);
        Wire.endTransmission();

        Wire.beginTransmission(MD25ADDRESS); // Sets a constant speed for Right wheel
        Wire.write(SPEED2);
        Wire.write(20);
        Wire.endTransmission();

        Left_Turn(factor2);
    }
}

void Turn(int o)
{ //
    encoder1();
    encoder2();
    LW_Travel = o;
    RW_Travel = o;
    if (encoder1() <= LW_Travel && encoder2() <= RW_Travel)
    {

        Wire.beginTransmission(MD25ADDRESS); // Sets the acceleration to register 1 (6.375s)
        Wire.write(ACCELERATION);
        Wire.write(5);
        Wire.endTransmission();

        Wire.beginTransmission(MD25ADDRESS); // Sets a combined motor speed value
        Wire.write(SPEED1);
        Wire.write(-20);
        Wire.endTransmission();

        Wire.beginTransmission(MD25ADDRESS); // Sets a combined motor speed value
        Wire.write(SPEED2);
        Wire.write(20);
        Wire.endTransmission();

        Turn(o);
    }
}

void ShortCurvePath()
{
    encoder1();
    encoder2();

    if (encoder1() <= 58)
    {
        Wire.beginTransmission(MD25ADDRESS);
        Wire.write(ACCELERATION); // Set acceleration register to 3
        Wire.write(2);
        Wire.endTransmission();

        Wire.beginTransmission(MD25ADDRESS);
        Wire.write(SPEED1);
        Wire.write(40); // Ratio of speed
        Wire.endTransmission();

        Wire.beginTransmission(MD25ADDRESS);
        Wire.write(SPEED2);
        Wire.write(15); // Ratio Speed
        Wire.endTransmission();
        ShortCurvePath();
    }
}

void LongCurvePath()
{
    encoder1();
    encoder2();
    int abc = 144;
    if (encoder1() <= abc)
    {
        Wire.beginTransmission(MD25ADDRESS);
        Wire.write(ACCELERATION); // Set acceleration register to 3
        Wire.write(2);
        Wire.endTransmission();

        Wire.beginTransmission(MD25ADDRESS);
        Wire.write(SPEED1);
        Wire.write(50); // Ratio of speed
        Wire.endTransmission();

        Wire.beginTransmission(MD25ADDRESS);
        Wire.write(SPEED2);
        Wire.write(8); // Ratio Speed
        Wire.endTransmission();
        LongCurvePath();
    }
}

void DropMarker()
{
    stepper.setSpeed(10); // 10 rpm
    stepper.step(340);    // do 340 steps -- corresponds to move dropping mechanism 60 degrees
}

void Signal()
{
    digitalWrite(13, HIGH); // turn the LED on (HIGH is the voltage level)
    delay(100);             // wait for a second
    digitalWrite(13, LOW);  // turn the LED off by making the voltage LOW
    delay(100);             // wait for a second
    digitalWrite(13, HIGH); // turn the LED on (HIGH is the voltage level)
    delay(100);             // wait for a second
    digitalWrite(13, LOW);  // turn the LED off by making the voltage LOW
    delay(100);             // wait for a second
}

float encoder1()
{                                        // Function to read and display value of encoder 1 as a long
    Wire.beginTransmission(MD25ADDRESS); // Send byte to get a reading from encoder 1
    Wire.write(ENCODERONE);
    Wire.endTransmission();

    Wire.requestFrom(MD25ADDRESS, 4); // Request 4 bytes from MD25
    while (Wire.available() < 4)
        ;                     // Wait for 4 bytes to arrive
    long poss1 = Wire.read(); // First byte for encoder 1, HH.
    poss1 <<= 8;
    poss1 += Wire.read(); // Second byte for encoder 1, HL
    poss1 <<= 8;
    poss1 += Wire.read(); // Third byte for encoder 1, LH
    poss1 <<= 8;
    poss1 += Wire.read();  // Fourth byte for encoder 1, LLalue
    delay(5);              // Wait for everything to make sure everything is sent
    return (poss1 * 0.09); // Convert encoder value to cm
}

float encoder2()
{ // Function to read and display velue of encoder 2 as a long
    Wire.beginTransmission(MD25ADDRESS);
    Wire.write(ENCODERTWO);
    Wire.endTransmission();

    Wire.requestFrom(MD25ADDRESS, 4); // Request 4 bytes from MD25
    while (Wire.available() < 4)
        ;                     // Wait for 4 bytes to become available
    long poss2 = Wire.read(); // First byte for encoder 2, HH
    poss2 <<= 8;
    poss2 += Wire.read(); // Second byte for encoder 2, HL
    poss2 <<= 8;
    poss2 += Wire.read(); // Third byte for encoder 2, LH
    poss2 <<= 8;
    poss2 += Wire.read();  // Fourth byte for encoder 2, LLalue
    delay(5);              // Wait to make sure everything is sent
    return (poss2 * 0.09); // Convert encoder value to cm
}

void encodeReset()
{ // This function resets the encoder values to 0
    Wire.beginTransmission(MD25ADDRESS);
    Wire.write(CMD);
    Wire.write(0x20);
    Wire.endTransmission();
    delay(50);
}

void stopMotor()
{ // Function to stop motors

    Wire.beginTransmission(MD25ADDRESS); // Sets the acceleration to register 10 (0.65s)
    Wire.write(ACCELERATION);
    Wire.write(10);
    Wire.endTransmission();

    Wire.beginTransmission(MD25ADDRESS); // Stops motors motor 1 if operated in MODE 0 or 1 and Stops both motors if operated in MODE 2 or 3
    Wire.write(SPEED1);
    Wire.write(0);
    Wire.endTransmission();

    Wire.beginTransmission(MD25ADDRESS); // Stops motors motor 2 when operated in MODE 0 or 1 and Stops both motors while in turning sequence if operated in MODE 2 or 3
    Wire.write(SPEED2);
    Wire.write(0);
    Wire.endTransmission();

    delay(50);
    encodeReset(); // Resets encoder everytime it reaches a stop/checkpoint
}

void setup()
{
    Wire.begin();       // Begin I2C bus
    Serial.begin(9600); // Begin serial
    delay(50);          // Wait for everything to power up

    Wire.beginTransmission(MD25ADDRESS); // Set MD25 operation MODE
    Wire.write(MODE_SELECTOR);
    Wire.write(Mode); // Set to Mode 1
    Wire.endTransmission();

    pinMode(13, OUTPUT); // Make Pin 13 an output for LED

    delay(50);
    encodeReset(); // Cals a function that resets the encoder values to 0

    // Path to be taken
    Serial.println("Course Starts");
    forward(38); // Calls a function that moves the platform forward
    stopMotor(); // Calls a function that stops the platform
    Signal();
    Serial.println("Reached Checkpoint 1");
    Right_Turn(1.05);
    stopMotor();

    forward(26); // Calls a function that moves the platform forward
    stopMotor(); // Calls a function that stops the platform
    Signal();
    DropMarker();
    Serial.println("Reached Checkpoint 2");
    forward(4);
    stopMotor();

    Left_Turn(1.05);
    stopMotor();

    forward(62); // Calls a function that moves the platform forward
    stopMotor(); // Calls a function that stops the platform
    Signal();
    Serial.println("Reached Checkpoint 3");
    Left_Turn(1.05);
    stopMotor();

    ShortCurvePath(); // d0 = 18.85 di=21.99
    stopMotor();
    Signal();
    DropMarker();
    Serial.println("Reached Checkpoint 4");
    Right_Turn(1.17); // RT2
    stopMotor();
    DropMarker();

    forward(88); // Calls a function that moves the platform forward
    stopMotor(); // Calls a function that stops the platform
    Serial.println("Reached Checkpoint 5");
    Left_Turn(1.1); // LT2
    stopMotor();

    forward(45);
    stopMotor();
    Signal();
    DropMarker();
    Serial.println("Reached Checkpoint 6");
    Left_Turn(1.05);
    stopMotor();

    forward(46); // Calls a function that moves the platform forward
    stopMotor(); // Calls a function that stops the platform
    Signal();
    Serial.println("Reached Checkpoint 7");
    Left_Turn(1.1); // LT2
    stopMotor();

    forward(43); // Calls a function that moves the platform forward
    stopMotor(); // Calls a function that stops the platform
    Signal();
    DropMarker();
    Serial.println("Reached Checkpoint 8");
    Turn(8.8); // Left_Turn(0.49); d = 8.377, 40 degree left turn
    stopMotor();

    forward(84); // Calls a function that moves the platform forward Small diagonal line distance
    stopMotor(); // Calls a function that stops the platform
    Signal();
    Serial.println("Reached Checkpoint 9");
    Turn(32); // Left_Turn(1.8); d = 20
    stopMotor();

    forward(23); // Calls a function that moves the platform forward
    stopMotor(); // Calls a function that stops the platform
    Signal();
    Serial.println("Reached Checkpoint 10");
    Right_Turn(1.17); // RT2
    stopMotor();
    DropMarker();

    LongCurvePath(); // d0=141.37 di=69.9
    stopMotor();
    Signal();
    Serial.println("Reached Checkpoint 11");
    Left_Turn(1.1); // LT2
    stopMotor();

    Turn(13); // Left_Turn(0.72)
    stopMotor();

    forward(43); // Calls a function that moves the platform forward
    stopMotor(); // Calls a function that stops the platform
    Signal();
    Serial.println("Reached Checkpoint 12");
    forward(31); // Calls a function that moves the platform forward
    stopMotor(); // Calls a function that stops the platform
    Signal();
    Serial.println("Finished Course");
}

void loop()
{
    // do nothing
}
