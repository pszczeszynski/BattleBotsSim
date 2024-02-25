/**
 * Receiver.ino
 */
// #include "Motor.h"

#include "Vesc.h"
#include "Communication.h"
#include "Motor.h"

#define VENDOR_ID               0x16C0
#define PRODUCT_ID              0x0480
#define RAWHID_USAGE_PAGE       0xFFAC  // recommended: 0xFF00 to 0xFFFF
#define RAWHID_USAGE            0x0300  // recommended: 0x0100 to 0xFFFF


#define VERBOSE_RADIO
 #define LOG_DATA

#include "Radio.h"
#include "IMU.h"
#include "Logging.h"

#define SERIAL_BAUD 460800

// time to wait for received packets. If too short, likely to miss packets from driver station
#define RECEIVE_TIMEOUT_MS 0

// time to wait before stopping the robot if no packets are received
#define STOP_ROBOT_TIMEOUT_MS 250

// time in between printing drive commands for debugging
#define PRINT_DRIVE_COMMAND_MS 500

// if not defined, will not use imu or send imu data
#define USE_IMU

// can bus ids
#define LEFT_MOTOR_CAN_ID 3
#define RIGHT_MOTOR_CAN_ID 1
#define FRONT_WEAPON_CAN_ID 2
#define BACK_WEAPON_CAN_ID 4

// closest to the middle of the teensy
#define SELF_RIGHTER_MOTOR_PIN 9

// components
IMU *imu;
VESC vesc(LEFT_MOTOR_CAN_ID, RIGHT_MOTOR_CAN_ID, FRONT_WEAPON_CAN_ID, BACK_WEAPON_CAN_ID);
Motor *selfRightMotor;
Radio<RobotMessage, DriveCommand> *radio;
Logger *logger;

// variables

// radio data
int validMessageCount = 0;
int invalidMessageCount = 0;
DriveCommand lastDriveCommand;
int maxReceiveDelayMs = 0;

void setup()
{
    // Start serial port to display messages on Serial Monitor Window
    Serial.begin(SERIAL_BAUD);

    Serial.println("hello");

#ifdef USE_IMU
    Serial.println("Initializing IMU...");
    imu = new IMU();
    Serial.println("Success!");
#else
    Serial.println("Not using IMU");
#endif

    Serial.println("Initializing Self-Righting Motor...");
    selfRightMotor = new Motor(SELF_RIGHTER_MOTOR_PIN);
    Serial.println("Success!");

    Serial.println("Initializing radio...");
    radio = new Radio<RobotMessage, DriveCommand>();
    Serial.println("Success!");

#ifdef LOG_DATA
    Serial.println("Initializing SD card...");
    logger = new Logger();
    Serial.println("Success!");
#else
    Serial.println("Not logging data");
#endif

    vesc.Drive(0, 0);
    vesc.DriveWeapons(0, 0);
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
    vesc.Drive(leftPower, rightPower);
}

/**
 * Applys the weapon command to the robot
 */
void DriveWeapons(DriveCommand &command)
{
    vesc.DriveWeapons(command.frontWeaponPower, command.backWeaponPower);
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
#define TELEMETRY_UPDATE_FREQUENCY 10
#define MAX_DELAY_SAMPLE
RobotMessage Update()
{
    static int updateCount = 0;
    static unsigned long lastRadioStatsRefreshTime = 0;

    RobotMessage ret{RobotMessageType::INVALID};

#ifdef USE_IMU
    // call update for imu
    imu->Update();
#endif

    vesc.Update();

    // increase update count and wrap around
    updateCount++;
    updateCount %= TELEMETRY_UPDATE_FREQUENCY;

    // if time to send a can message
    if (updateCount == 0)
    {
        ret.type = CAN_DATA;
        vesc.GetCurrents(ret.canData.motorCurrent);
        vesc.GetVolts(ret.canData.motorVoltage);
        vesc.GetRPMs(ret.canData.motorERPM);
        vesc.GetFETTemps(ret.canData.escFETTemp);
    }
    // send radio data
    else if (updateCount == TELEMETRY_UPDATE_FREQUENCY / 2)
    {
        // send radio info
        ret.type = RADIO_DATA;
        // send average delay (0 if no messages received yet)
        ret.radioData.averageDelayMS = validMessageCount == 0 ? -1 : ((float) (millis() - lastRadioStatsRefreshTime) / validMessageCount);
        // set invalid message count
        ret.radioData.invalidPackets = (short) invalidMessageCount;
        // set max delay
        ret.radioData.maxDelayMS = maxReceiveDelayMs;

        // set the last drive command fields
        ret.radioData.movement = lastDriveCommand.movement;
        ret.radioData.turn = lastDriveCommand.turn;
        ret.radioData.frontWeaponPower = lastDriveCommand.frontWeaponPower;
        ret.radioData.backWeaponPower = lastDriveCommand.backWeaponPower;
        
        // reset max delay
        if (millis() - lastRadioStatsRefreshTime > 1000)
        {
            invalidMessageCount = 0;
            validMessageCount = 0;
            maxReceiveDelayMs = 0;
            lastRadioStatsRefreshTime = millis();
        }
    }
    // else send imu data
    else
    {
        ret.type = IMU_DATA;

#ifdef USE_IMU
        // get accelerometer data and set accelsdf
        Point accel = imu->getAccel();
        ret.imuData.accelX = accel.x;
        ret.imuData.accelY = accel.y;

        // get gyro data and set gyro
        ret.imuData.rotation = imu->getRotation();
        // calculate rotation velocity
        ret.imuData.rotationVelocity = imu->getRotationVelocity();
#else
        ret.imuData.accelX = 0;
        ret.imuData.accelY = 0;
        ret.imuData.rotation = 0;
        ret.imuData.rotationVelocity = 0;
#endif
    }

    return ret;
}

/**
 * Waits for a radio packet to be available
 * Times out after 4 ms
 */
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
            // break because we must move on
            break;
        }

#ifdef USE_IMU
        // call update for imu while waiting
        imu->Update();
#endif
    }
}

/**
 * Checks if there is a message and then calls drive
 * There is a watchdog timer that auto stops after 250 ms
 */
void DriveWithLatestMessage()
{
    static unsigned long lastReceiveTime = 0;
    static long lastPrintTime = 0;

    // if there is a message available
    if (radio->Available())
    {
        // receive latest drive command
        DriveCommand command = radio->Receive();
        // save the last drive command
        lastDriveCommand = command;

        // sanity check command fields
        if (command.movement > 1 || command.movement < -1 ||
            command.turn > 1 || command.turn < -1 ||
            command.frontWeaponPower > 1 || command.frontWeaponPower < -1 ||
            command.backWeaponPower > 1 || command.backWeaponPower < -1 ||
            command.selfRighterPower > 1 || command.selfRighterPower < -1)
        {
            command.valid = false;
        }

        // check for NAN
        if (isnan(command.movement) || isnan(command.turn) ||
            isnan(command.frontWeaponPower) || isnan(command.backWeaponPower) ||
            isnan(command.selfRighterPower))
        {
            command.valid = false;
        }

        // check if the command is valid
        if (command.valid)
        {
            // drive the robot, weapons, self righter
            Drive(command);
            DriveWeapons(command);
            DriveSelfRighter(command);

            // increment message count
            validMessageCount++;

            // save the max delay
            int delayMs = millis() - lastReceiveTime;
            if (delayMs > maxReceiveDelayMs)
            {
                maxReceiveDelayMs = delayMs;
            }

            // update the last receive time
            lastReceiveTime = millis();

        }
        // if the command is invalid
        else
        {
            // increment invalid message count
            invalidMessageCount++;
            // print error message
            Serial.println("Invalid drive command received");
        }
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


    }
}

void loop()
{
    // wait for a message to be available
    WaitForRadioData();

    // drive with the latest message if there is one
    DriveWithLatestMessage();

    // Compute response message
    RobotMessage message = Update();

    // send the message
    SendOutput result = radio->Send(message);

    if (result != SEND_SUCCESS)
    {
        Serial.println("Failed to send radio message. Result: " + (String) result);

    }
    logger->update();
}