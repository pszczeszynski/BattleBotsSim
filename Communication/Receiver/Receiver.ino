/**
 * Receiver.ino
 */
// #include "Motor.h"

#include "Vesc.h"
#include "Communication.h"
#include "Motor.h"

#define VERBOSE_RADIO
// #define LOG_DATA

#include "Radio.h"
#include "IMU.h"
#include "Logging.h"

#define SERIAL_BAUD 9600

IMU* imu;

#define LEFT_MOTOR_CAN_ID 3
#define RIGHT_MOTOR_CAN_ID 1
#define FRONT_WEAPON_CAN_ID 2
#define BACK_WEAPON_CAN_ID 4

// closest to the middle of the teensy
#define SELF_RIGHTER_MOTOR_PIN 9

VESC* vesc;
Motor* selfRightMotor;
Radio<RobotMessage, DriveCommand>* radio;
Logger* logger;

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

    Serial.println("Initializing Canbus motors...");
    vesc = new VESC(LEFT_MOTOR_CAN_ID, RIGHT_MOTOR_CAN_ID, FRONT_WEAPON_CAN_ID, BACK_WEAPON_CAN_ID);
    Serial.println("Success!");

    Serial.println("Initializing Self-Righting Motor...");
    selfRightMotor = new Motor(SELF_RIGHTER_MOTOR_PIN);
    Serial.println("Success!");

    Serial.println("Initializing radio...");
    radio = new Radio<RobotMessage, DriveCommand>();
    Serial.println("Success!");

    Serial.println("Initializing SD card...");
    logger = new Logger(const_cast<char *>("dataLog.txt"));
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
 * Drives the self righting motor
*/
void DriveSelfRighter(DriveCommand &command)
{
    selfRightMotor->SetPower(command.selfRighterPower);
}

/**
 * Computes response message of robot state data
 */
#define CAN_UPDATE_INTERVAL 10
RobotMessage Update()
{
    static int updateCount = 0;
    RobotMessage ret{RobotMessageType::INVALID};

    // call update for imu
    imu->Update();

    // increase update count and wrap around
    updateCount ++;
    updateCount %= CAN_UPDATE_INTERVAL;

    // if time to send a can message
    if (updateCount == 0)
    {
        ret.type = CAN_DATA;
        vesc->GetCurrents(ret.canData.motorCurrent);
        vesc->GetVolts(ret.canData.motorVoltage);
        vesc->GetRPMs(ret.canData.motorRPM);
        vesc->GetFETTemps(ret.canData.escFETTemp);
    }
    // else send imu data
    else
    {
        ret.type = IMU_DATA;

        // get accelerometer data and set accel
        Point accel = imu->getAccel();
        ret.imuData.accelX = accel.x;
        ret.imuData.accelY = accel.y;

        // get gyro data and set gyro
        ret.imuData.rotation = imu->getRotation();

        // calculate rotation velocity
        ret.imuData.rotationVelocity = imu->getRotationVelocity();
    }

#ifdef LOG_DATA
    logger->logMessage(logger->formatRobotMessage(ret));
#endif

    return ret;
}

/**
 * Waits for a radio packet to be available
 * Times out after 1 ms
 */
#define RECEIVE_TIMEOUT_MS 1
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
            // // print error message
            // Serial.println("ERROR: receive timeout");

            // increment counter
            TIMEOUT_COUNT++;

            if (TIMEOUT_COUNT % 10 == 0)
            {
                // reset counter
                TIMEOUT_COUNT = 0;

                // attempt to reinitialize the radio
#ifdef LOG_DATA
                logger->logMessage("Re-initializing Radio");
#endif
                // radio->InitRadio();
            }

            // break because we must move on
            break;
        }

        // call update for imu
        imu->Update();
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
    static long lastPrintTime = 0;
    static int numMessagesReceived = 0;
    if (radio->Available())
    {
        // receive latest drive command
        DriveCommand command = radio->Receive();

        // print every 100 messages
        if (numMessagesReceived % 1 == 0)
        {
            Serial.print("Received drive command movement: ");
            Serial.println(command.movement);

            Serial.print("Packets per second: ");
            Serial.println(1000.0 / (millis() - lastPrintTime));
            lastPrintTime = millis();
        }
        // increment message count
        numMessagesReceived++;

        Drive(command);
        DriveWeapons(command);
        DriveSelfRighter(command);

        // update the last receive time
        lastReceiveTime = millis();

#ifdef LOG_DATA
        logger->logMessage(logger->formatDriveCommand(command));
#endif
    }

    // if haven't received a message in a while, stop the robot
    if (millis() - lastReceiveTime > STOP_ROBOT_TIMEOUT_MS)
    {
        DriveCommand command{0};
        command.movement = 0;
        command.turn = 0;
        command.frontWeaponPower = 0;
        command.backWeaponPower = 0;
        command.selfRighterPower = 0;
        command.valid = true;
        Drive(command);
        DriveWeapons(command);
        DriveSelfRighter(command);

#ifdef LOG_DATA
        logger->logMessage("Radio Timeout");
#endif
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

    Serial.println("sending message");
    // send the message
    SendOutput result = radio->Send(message);
    if (result != SEND_SUCCESS)
    {
        Serial.println("failed to send");
    }

#ifdef LOG_DATA
    if (result == FIFO_FAIL) logger->logMessage("Radio fifo failed to clear");
    else if (result == HW_FAULT) logger->logMessage("Radio hardware failure detected");
#endif
}