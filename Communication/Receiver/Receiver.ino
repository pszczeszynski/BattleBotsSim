/**
 * Receiver.ino
*/
#include "Motor.h"
#include "Communication.h"
#include "Radio.h"
#include "IMU.h"

#define SERIAL_BAUD 9600

IMU* imu;
#define LEFT_MOTOR_PIN 9
#define RIGHT_MOTOR_PIN 8
Motor* leftMotor;
Motor* rightMotor;
Radio<RobotMessage, DriveCommand>* radio;

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
    radio = new Radio<RobotMessage, DriveCommand>();
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
RobotMessage Update()
{
    RobotMessage ret {0};

    // get the time since the last update
    double deltaTimeMS = getDt();
    // call update for imu
    imu->Update(deltaTimeMS);
  
    // get accelerometer data and set accel
    ret.accel = imu->getAccel();

    // now compute velocity
    ret.velocity = imu->getVelocity();

    // get gyro data and set gyro
    ret.rotation = imu->getRotation();

    // calculate rotation velocity
    ret.rotationVelocity = imu->getRotationVelocity();

    return ret;
}






/**
 * Waits for a radio packet to be available
 * Times out after 50 ms
*/
#define RECEIVE_TIMEOUT_MS 50
void WaitForRadioData()
{
    static int TIMEOUT_COUNT = 0;
    unsigned long waitReceiveStartTimeMS = millis();

    // while the radio isn't available
    while (!radio->Available())
    {
        // if too much time has passed
        if (millis() - waitReceiveStartTimeMS >= RECEIVE_TIMEOUT_MS)
        {
            // print error message
            Serial.println("ERROR: receive timeout");

            // increment counter
            TIMEOUT_COUNT ++;

            if (TIMEOUT_COUNT % 10 == 0)
            {
                // attempt to reinitialize the radio
                radio->InitRadio();
            }

            // break because we must move on
            break;
        }
    }
}


/**
 * Checks if there is a message and then calls drive
 * There is a watchdog timer that auto stops after 250 ms
*/
unsigned long lastReceiveTime = 0;
#define STOP_ROBOT_TIMEOUT_MS 250
void DriveWithLatestMessage()
{
    static int numMessagesReceived = 0;
    if (radio->Available())
    {
        // receive latest drive command
        DriveCommand command = radio->Receive();
        if (numMessagesReceived % 100 == 0)
        {
            Serial.print("Received drive command movement: ");
            Serial.println(command.movement);
        }
        numMessagesReceived ++;

        // drive with message
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



//===============================================================================
//  Main
//===============================================================================
void loop()
{
    // wait for a message to be available
    WaitForRadioData();

    // drive with the latest message if there is one
    DriveWithLatestMessage();

    // Compute response message
    RobotMessage message = Update();

    // send the message
    radio->Send(message);
}