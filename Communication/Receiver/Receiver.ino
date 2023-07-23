/**
 * Receiver.ino
*/
#include "WorkingDir.h"
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

    //Explicitly set both motors to 0 before loop
    leftMotor->SetPower(0); // -1 for One direction, 0 for bidirection
    rightMotor->SetPower(0);

    // calibration
    // leftMotor->SetPower(1);
    // Serial.println("high");
    // delay(5000);
    // leftMotor->SetPower(-1);
    // Serial.println("low");
    // delay(5000);
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
// RobotMessage update(int dt)
// {
//     RobotMessage ret {0};

//     // get accelerometer data and set accel
//     ret.accel = imu->getAccel(); //{1,2.5,3.999}

//     // now compute velocity
//     ret.velocity = imu->getVelocity(ret.accel, dt);   //0,0,0}

//     // get gyro data and set gyro
//     ret.rotation = imu->getRotation(dt);

//     return ret;
// }


unsigned long lastReceiveTime = 0;
#define STOP_ROBOT_TIMEOUT_MS 500
//===============================================================================
//  Main
//===============================================================================
void loop()
{
    if (radio->Available())
    {
        lastReceiveTime = millis();
        /*
        IMPORTANT NOTE:
        The radio will sometimes think it is receiving data when it is clearly not.
        In that case, movement and turn will be 0.
        This is fine as long as the ESC supports going in reverse.
        Otherwise it's bad - the motor will run at half power.
        */

        // Read the incoming message 
        // Serial.println("Receiving Radio Transmission");
        DriveCommand command = radio->Receive();
        
        // Drive the robot
        Drive(command);
    }

    if (millis() - lastReceiveTime > STOP_ROBOT_TIMEOUT_MS)
    {
        DriveCommand command {0, 0};
        Drive(command);
    }
    
    // // Compute robot state
    // int dt = getDt();
    // RobotMessage message = update(dt);
    // // Send data back to transmitter and driver station
    // radio->Send(message);

    // Serial.println("about to print");
    // // Get IMU Readings
    // if (imu->dataReady())
    // {
    //     Serial.println("about to update");
    //     imu->Update();
    //     Serial.println("updated");
    //     imu->printScaledAGMT();
    //     Serial.println("finished");
    // }
}