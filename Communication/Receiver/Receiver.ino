/**
 * Receiver.ino
*/
#include "Communication.h"
#include "IMU.h"
#include "Radio.h"
#include "Motor.h"

#define SERIAL_BAUD 9600

IMU* imu;
#define LEFT_MOTOR_PIN 9
#define RIGHT_MOTOR_PIN 8
Motor* leftMotor;
Motor* rightMotor;
Radio* radio;

double prevTime = 0;
double currTime = 0;

//===============================================================================
//  Initialization
//===============================================================================
void setup()
{
    // Start serial port to display messages on Serial Monitor Window
    Serial.begin(SERIAL_BAUD);

    Serial.println("Initializing IMU...");
    imu = new IMU();
    Serial.println("Failed because Matthew made an oopsie!");

    Serial.println("Initializing motors...");
    leftMotor = new Motor(LEFT_MOTOR_PIN);
    rightMotor = new Motor(RIGHT_MOTOR_PIN);
    Serial.println("Success!");

    Serial.println("Initializing radio...");
    radio = new Radio();
    Serial.println("Success!");

    //Explicitly set both motors to 0 before loop
    leftMotor->SetPower(0); // -1 for One direction, 0 for bidirection
    rightMotor->SetPower(0);
}

/**
 *Updates the time data for the current loop iteration
 */
double getDt()
{
    currTime = micros() / 1000;
    double dt = currTime - prevTime;
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
    // Serial.print("Writing "); Serial.print(leftPower); Serial.println(" to left motor.");
    double rightPower = command.movement + command.turn;
    // Serial.print("Writing "); Serial.print(rightPower); Serial.println(" to right motor.");

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
RobotMessage update(double deltaTimeMS)
{
    RobotMessage ret {0};

    imu->Update(deltaTimeMS);
    // get accelerometer data and set accel
    ret.accel = imu->getAccel();

    // now compute velocity
    ret.velocity = imu->getVelocity();

    // get gyro data and set gyro
    ret.rotation = imu->getRotation();

    return ret;
}

unsigned long lastReceiveTime = 0;
#define STOP_ROBOT_TIMEOUT_MS 250

//===============================================================================
//  Main
//===============================================================================
void loop()
{
    // if a message is available, receive it
    if (imu->dataReady())
    {
        // Get the delta time
        double dt = getDt();
        imu->Update(dt);
        // Compute response message
        RobotMessage message = update(dt);

        // send the message
        radio->Send(message);
    }

    // if a message is available, receive it
    if (radio->Available())
    {
        // get the message
        DriveCommand command = radio->Receive();

        Serial.print("Received drive command movement: ");
        Serial.println(command.movement);

        // apply the message
        Drive(command);

        // update the last receive time
        lastReceiveTime = millis();
    }

    // if haven't received a message in a while, stop the robot
    if (millis() - lastReceiveTime > STOP_ROBOT_TIMEOUT_MS)
    {
        DriveCommand command {0, 0};
        Drive(command);
    }

}