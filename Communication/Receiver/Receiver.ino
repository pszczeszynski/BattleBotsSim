/**
 * Receiver.ino
 */
// #include "Motor.h"


#include "Communication.h"
#include "Vesc.h"

#define VERBOSE_RADIO
#include "Radio.h"
#include "IMU.h"

#define SERIAL_BAUD 9600

IMU* imu;

#define LEFT_MOTOR_CAN_ID 3
#define RIGHT_MOTOR_CAN_ID 1
#define FRONT_WEAPON_CAN_ID 2
#define BACK_WEAPON_CAN_ID 4

VESC* vesc;
Radio<RobotMessage, DriveCommand>* radio;

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

    Serial.println("Initializing Canbus motors...");
    vesc = new VESC(LEFT_MOTOR_CAN_ID, RIGHT_MOTOR_CAN_ID, FRONT_WEAPON_CAN_ID, BACK_WEAPON_CAN_ID);
    Serial.println("Success!");

    Serial.println("Initializing radio...");
    radio = new Radio<RobotMessage, DriveCommand>();
    Serial.println("Success!");

    vesc->Drive(0, 0);
    vesc->DriveWeapons(0, 0);
}

/**
 * Applys the drive command to the robot
 */
void Drive(DriveCommand &command)
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
    vesc->Drive(leftPower, rightPower);
}

/**
 * Applys the weapon command to the robot
*/
void DriveWeapons(DriveCommand &command)
{
    vesc->DriveWeapons(command.frontWeaponPower, command.backWeaponPower);
}

/**
 * Computes response message of robot state data
 */
RobotMessage Update()
{
    RobotMessage ret{0};

    // call update for imu
    // imu->Update();

    // get accelerometer data and set accel
    // Point accel = imu->getAccel();
    // ret.accelX = accel.x;
    // ret.accelY = accel.y;

    // // now compute velocity
    // Point velocity = imu->getVelocity();
    // ret.velocityX = velocity.x;
    // ret.velocityY = velocity.y;

#ifdef PRINT_VELOCITY_ACCEL
    // print on serial
    Serial.print("Accel: ");
    Serial.print(ret.accel.x);
    Serial.print(", ");
    Serial.print(ret.accel.y);
    Serial.print(", ");
    Serial.print(ret.accel.z);

    Serial.print(" | Velocity: ");
    Serial.print(ret.velocity.x);
    Serial.print(", ");
    Serial.print(ret.velocity.y);
    Serial.print(", ");
    Serial.print(ret.velocity.z);
    Serial.println("");
#endif

    // // get gyro data and set gyro
    // ret.rotation = imu->getRotation();

    // // calculate rotation velocity
    // ret.rotationVelocity = imu->getRotationVelocity();

    ret.valid = true;

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
            TIMEOUT_COUNT++;

            if (TIMEOUT_COUNT % 10 == 0)
            {
                // attempt to reinitialize the radio
                radio->InitRadio();
            }

            // break because we must move on
            break;
        }

        // // call update for imu
        // imu->Update();
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
        numMessagesReceived++;

        // drive with message
        Drive(command);

        DriveWeapons(command);

        // update the last receive time
        lastReceiveTime = millis();
    }

    // if haven't received a message in a while, stop the robot
    if (millis() - lastReceiveTime > STOP_ROBOT_TIMEOUT_MS)
    {
        DriveCommand command{0};
        command.movement = 0;
        command.turn = 0;
        command.frontWeaponPower = 0;
        command.backWeaponPower = 0;
        command.valid = true;
        Drive(command);
        DriveWeapons(command);
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