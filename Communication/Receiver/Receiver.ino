/**
 * Receiver.ino
*/
#include "Communication.h"
#include "IMU.h"
#include "Radio.h"
#include "Motor.h"

#define SERIAL_BAUD 115200

IMU* imu;
#define LEFT_MOTOR_PIN 9
#define RIGHT_MOTOR_PIN 10
Motor* leftMotor;
Motor* rightMotor;
Radio* radio;

unsigned long prevTime = 0;
unsigned long currTime = 0;

//===============================================================================
//  Initialization
//===============================================================================
void setup()
{
    // Start serial port to display messages on Serial Monitor Window
    Serial.begin(SERIAL_BAUD);

    Serial.println("Initializing IMU...");
    // imu = new IMU();
    Serial.println("Failed because Matthew made an oopsie!");

    Serial.println("Initializing motors...");
    leftMotor = new Motor(LEFT_MOTOR_PIN);
    rightMotor = new Motor(RIGHT_MOTOR_PIN);
    Serial.println("Success!");

    Serial.println("Initializing radio...");
    radio = new Radio();
    Serial.println("Success!");
}

/**
 *Updates the time data for the current loop iteration
 */
int getDt()
{
    currTime = millis();
    int dt = currTime - prevTime;
    prevTime = currTime;
    return dt;
}

/**
 * Applys the drive command to the robot
*/
void Drive(DriveCommand& command)
{
    // // print the message to the serial monitor window
    // String print = "";
    // print += "Received drive command movement: ";
    // print += command.movement;
    // print += ", turn: ";
    // print += command.turn;
    // Serial.println(print);

    // compute powers
    double leftPower = command.movement - command.turn;
    double rightPower = command.movement + command.turn;

    // normalize
    double maxPower = max(abs(leftPower), abs(rightPower));
    if (maxPower > 1)
    {
        leftPower /= maxPower;
        rightPower /= maxPower;
    }

    // apply powers
    leftMotor->SetPower(leftPower);
    rightMotor->SetPower(rightPower);
}

/**
 * Computes response message of robot state data
*/
RobotMessage update(int dt)
{
    RobotMessage ret {0};

    // get accelerometer data and set accel
    ret.accel = {0,0,0};//imu->getAccel();

    // now compute velocity
    ret.velocity = {0,0,0};//imu->getVelocity(ret.accel, dt);

    // get gyro data and set gyro
    ret.rotation = 0;//imu->getRotation();

    return ret;
}


//===============================================================================
//  Main
//===============================================================================
void loop()
{
    if (radio->Available())
    {
        // Read the incoming message 
        DriveCommand command = radio->Receive();
        
        // Drive the robot
        Drive(command);

        // Compute robot state
        int dt = getDt();
        RobotMessage message = update(dt);
        // Send data back to transmitter and driver station
        radio->Send(message);
    }
  
    // if (imu->dataReady())
    // {
    //     imu->Update();
    //     imu->printScaledAGMT();
    // }
}