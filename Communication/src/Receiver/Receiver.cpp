/**
 * Receiver.ino
 */
#include "Receiver.h"
#include "Receiver/Vesc.h"
#include "Communication.h"
#include "Receiver/Motor.h"
#include "Receiver/RobotMovement.h"
#include "Radio.h"
#include "Hardware.h"
#include "PowerMonitor.h"
#include <FastLED.h>

#define ENABLE_AUTONOMOUS_DRIVE

#define VERBOSE_RADIO
// #define DEBUG_AUTO

#include "Receiver/IMU.h"
#include "Receiver/Logging.h"

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



#define LED_BRIGHTNESS 255
#define NUM_LEDS 3
CRGB leds[NUM_LEDS];

// components
#ifdef USE_IMU
IMU imu;
#endif
VESC vesc{LEFT_MOTOR_CAN_ID, RIGHT_MOTOR_CAN_ID, FRONT_WEAPON_CAN_ID, BACK_WEAPON_CAN_ID};
Radio<RobotMessage, DriverStationMessage> rx_radio{};
Motor selfRightMotor{SELF_RIGHTER_MOTOR_PIN};
Logger logger{};

PowerMonitor rx_power_monitor;

// variables

// radio data
int validMessageCount = 0;
int invalidMessageCount = 0;
DriveCommand lastDriveCommand;
AutoDrive lastAutoCommand;
DriverStationMessage lastMessage;
int maxReceiveDelayMs = 0;

uint8_t radioChannel = 0;

extern enum board_placement placement;

void determineChannel()
{
    switch(placement)
    {
        case rxLeft:
            radioChannel = TEENSY_RADIO_1;
            Serial.println("Firmware select: left receiver");
            break;
        case rxCenter:
            radioChannel = TEENSY_RADIO_2;
            Serial.println("Firmware select: center receiver");
            break;
        case rxRight:
            radioChannel = TEENSY_RADIO_3;
            Serial.println("Firmware select: right receiver");
            break;
        case tx:
        case invalidPlacement:
        default:
            // shouldn't happen 
            Serial.println("Error: running rx radio setup with tx board ID");
            break;
    }
}

void rx_setup()
{
    Serial.begin(SERIAL_BAUD);
    Serial.println("Hello, I'm Orbitron :)");

    
    determineChannel();
    rx_radio.SetChannel(radioChannel);

    // Start serial port to display messages on Serial Monitor Window
    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);

    if(placement == rxCenter)
    {
        selfRightMotor.init();
    }

    if (!logger.init())
    {
        Serial.println("WARNING: logger failed to initialize");
    }

    imu.Initialize(placement);
    imu.ForceCalibrate();

    pinMode(STATUS_1_LED_PIN, OUTPUT);
    pinMode(STATUS_2_LED_PIN, OUTPUT);
    pinMode(STATUS_3_LED_PIN, OUTPUT);
    pinMode(STATUS_4_LED_PIN, OUTPUT);

    // set the program status led on
    digitalWrite(STATUS_4_LED_PIN, true);
}

/**
 * Applys the drive command to the robot
 */
void Drive(DriveCommand &command)
{
    // compute powers
    float leftPower = command.movement - command.turn;
    float rightPower = command.movement + command.turn;

    // normalize
    float maxPower = max(abs(leftPower), abs(rightPower));
    if (maxPower > 1)
    {
        leftPower /= maxPower;
        rightPower /= maxPower;
    }

    // set status led
    digitalWrite(STATUS_1_LED_PIN, HIGH);

    // apply powers
    vesc.Drive(leftPower, rightPower);
}

/**
 * Applys the weapon command to the robot
 */
void DriveWeapons(DriveCommand &command)
{
    // Serial.println("Driving weapons with powers: " + String(command.frontWeaponPower) + ", " + String(command.backWeaponPower));
    // set status led
    digitalWrite(STATUS_2_LED_PIN, HIGH);

    vesc.DriveWeapons(command.frontWeaponPower, command.backWeaponPower);
}

/**
 * Drives the self righting motor
 */
void DriveSelfRighter(DriveCommand &command)
{
    if(placement == rxCenter)
    {
        selfRightMotor.SetPower(command.selfRighterPower);
        logger.updateSelfRighterData(command.selfRighterPower);
    }
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
    imu.Update();
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
        vesc.GetMotorTemps(ret.canData.motorTemp);
    }
    // send radio data
    else if (updateCount == TELEMETRY_UPDATE_FREQUENCY / 3)
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
    else if (updateCount == (2*TELEMETRY_UPDATE_FREQUENCY) / 3)
    {
        rx_power_monitor.readSensors();
        // send radio info
        ret.type = BOARD_TELEMETRY_DATA;

        ret.boardTelemetryData.current_5v = rx_power_monitor.get5vCurrent();
        ret.boardTelemetryData.voltage_3v3 = rx_power_monitor.get3v3Voltage();
        ret.boardTelemetryData.voltage_5v = rx_power_monitor.get5vVoltage();
        ret.boardTelemetryData.voltage_batt = rx_power_monitor.getBattVoltage();
    }
    // else send imu data
    else
    {
        ret.type = IMU_DATA;

#ifdef USE_IMU
        // get accelerometer data and set accelsdf
        Point accel = imu.getAccel();
        ret.imuData.accelX = accel.x;
        ret.imuData.accelY = accel.y;

        // get gyro data and set gyro
        ret.imuData.rotation = imu.getRotation();
        // calculate rotation velocity
        ret.imuData.rotationVelocity = imu.getRotationVelocity();
        logger.updateIMUData(ret.imuData.rotation, ret.imuData.rotationVelocity, ret.imuData.accelX, ret.imuData.accelY);
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
    while (!rx_radio.Available())
    {
        // if too much time has passed
        if (millis() - waitReceiveStartTimeMS >= RECEIVE_TIMEOUT_MS)
        {
            // break because we must move on
            break;
        }

#ifdef USE_IMU
        // call update for imu while waiting
        imu.Update();
#endif
    }
}

unsigned long lastReceiveTime = 0;

/**
 * Checks if there is a message and then calls drive
 * There is a watchdog timer that auto stops after 250 ms
 */
void DriveWithLatestMessage()
{
    static long lastPrintTime = 0;
    static long lastReinitRadioTime = 0;
    static bool noPacketWatchdogTrigger = false;

    // if there is a message available
    if (rx_radio.Available())
    {
        // receive latest driver station message
        DriverStationMessage msg = rx_radio.Receive();

        // if the message is a drive command
        if (msg.type == DRIVE_COMMAND)
        {
            DriveCommand command = msg.driveCommand;

            // sanity check command fields
            if (command.movement > 1.1 || command.movement < -1.1 ||
                command.turn > 1.1 || command.turn < -1.1 ||
                command.frontWeaponPower > 1000 || command.frontWeaponPower < -1000 ||
                command.backWeaponPower > 1000 || command.backWeaponPower < -1000 ||
                command.selfRighterPower > 1.1 || command.selfRighterPower < -1.1)
            {
                Serial.println("invalid case 1");
                msg.valid = false;
            }

            // check for NAN
            if (isnan(command.movement) || isnan(command.turn) ||
                isnan(command.frontWeaponPower) || isnan(command.backWeaponPower) ||
                isnan(command.selfRighterPower))
            {
                Serial.println("invalid case 2 (NAN)");
                msg.valid = false;
            }
        }
        // else if the message is an auto drive command
        else if (msg.type == AUTO_DRIVE)
        {
            // sanity check command fields
            if (msg.autoDrive.movement > 1.1 || msg.autoDrive.movement < -1.1 ||
                msg.autoDrive.targetAngle > 2 * M_PI || msg.autoDrive.targetAngle < -2 * M_PI ||
                msg.autoDrive.ANGLE_EXTRAPOLATE_MS < 0 ||
                msg.autoDrive.TURN_THRESH_1_DEG < 0 || msg.autoDrive.TURN_THRESH_1_DEG > 360 ||
                msg.autoDrive.TURN_THRESH_2_DEG < 0 || msg.autoDrive.TURN_THRESH_2_DEG > 360 ||
                msg.autoDrive.MAX_TURN_POWER_PERCENT < 0 || msg.autoDrive.MAX_TURN_POWER_PERCENT > 100 ||
                msg.autoDrive.MIN_TURN_POWER_PERCENT < 0 || msg.autoDrive.MIN_TURN_POWER_PERCENT > 100 ||
                msg.autoDrive.SCALE_DOWN_MOVEMENT_PERCENT < 0 || msg.autoDrive.SCALE_DOWN_MOVEMENT_PERCENT > 100)
            {
                msg.valid = false;
                Serial.println("invalid case 1 (AUTO DRIVE)");
            }

            // check for nan
            if (isnan(msg.autoDrive.movement) || isnan(msg.autoDrive.targetAngle) ||
                isnan(msg.autoDrive.ANGLE_EXTRAPOLATE_MS) || isnan(msg.autoDrive.TURN_THRESH_1_DEG) ||
                isnan(msg.autoDrive.TURN_THRESH_2_DEG) || isnan(msg.autoDrive.MAX_TURN_POWER_PERCENT) ||
                isnan(msg.autoDrive.MIN_TURN_POWER_PERCENT) || isnan(msg.autoDrive.SCALE_DOWN_MOVEMENT_PERCENT))
            {
                msg.valid = false;
                Serial.println("invalid case 2 (AUTO DRIVE)");
            }
        }
        // otherwise, the message is invalid
        else
        {
            Serial.println("invalid case 3 invalid type");

            msg.valid = false;
        }







        // check if the message is valid
        if (msg.valid)
        {
            // save the last message
            if (msg.type == DRIVE_COMMAND)
            {
                lastDriveCommand = msg.driveCommand;
            }
            else if (msg.type == AUTO_DRIVE)
            {
                lastAutoCommand = msg.autoDrive;
            }
            lastMessage = msg;

            // increment valid message count
            validMessageCount++;

            // save the max delay
            int delayMs = millis() - lastReceiveTime;
            if (delayMs > maxReceiveDelayMs)
            {
                maxReceiveDelayMs = delayMs;
            }

            // update the last receive time
            lastReceiveTime = millis();
            // unset the watchdog trigger
            noPacketWatchdogTrigger = false;
        }
        // if the message is invalid
        else
        {
            // increment invalid message count
            invalidMessageCount++;
            // print error message
            Serial.println("Invalid message received");
        }
    }

    // if it's time to stop the robot
    if (millis() - lastReceiveTime > STOP_ROBOT_TIMEOUT_MS)
    {
        // if haven't received a message in a while, stop the robot. But only once
        // this allows us to switch to a backup teensy, without this one stopping the robot
        if (!noPacketWatchdogTrigger)
        {
            Serial.println("NO PACKET WATCHDOG TRIGGERED, STOPPING ROBOT!");
            DriveCommand command{0};
            command.movement = 0;
            command.turn = 0;
            command.frontWeaponPower = 0;
            command.backWeaponPower = 0;
            command.selfRighterPower = 0;
            Drive(command);
            DriveWeapons(command);
            noPacketWatchdogTrigger = true;
        }

        // force self righer to 0 power if watchdog triggered
        DriveCommand command{0};
        DriveSelfRighter(command);

        // check if we should reinit the radio
        if (millis() - lastReinitRadioTime > 5000)
        {
            lastReinitRadioTime = millis();
            Serial.println("Reinit radio");
            rx_radio.InitRadio(radioChannel);
        }

        // set all status leds off since the watchdog has triggered
        digitalWrite(STATUS_1_LED_PIN, LOW);
        digitalWrite(STATUS_2_LED_PIN, LOW);
        digitalWrite(STATUS_3_LED_PIN, LOW);
    }
    else if (lastMessage.type == AUTO_DRIVE)
    {
        // drive to the target angle
        DriveCommand command = DriveToAngle(imu.getRotation(),
                                            imu.getRotationVelocity(),
                                            lastAutoCommand.targetAngle,
                                            lastAutoCommand.ANGLE_EXTRAPOLATE_MS,
                                            lastAutoCommand.TURN_THRESH_1_DEG,
                                            lastAutoCommand.TURN_THRESH_2_DEG,
                                            lastAutoCommand.MAX_TURN_POWER_PERCENT,
                                            lastAutoCommand.MIN_TURN_POWER_PERCENT,
                                            lastAutoCommand.SCALE_DOWN_MOVEMENT_PERCENT);

        // apply the movement from the last drive command
        command.movement = lastMessage.driveCommand.movement;
        command.frontWeaponPower = (float) lastAutoCommand.frontWeaponPower;
        command.backWeaponPower = (float) lastAutoCommand.frontWeaponPower;

#ifdef DEBUG_AUTO
        Serial.println("imu rotation: " + (String)imu.getRotation());
        Serial.println("imu rotation velocity: " + (String)imu.getRotationVelocity());
        Serial.println("target angle: " + (String)lastAutoCommand.targetAngle);
        Serial.println("Auto drive command translated to: " + (String)command.movement + " " + (String)command.turn);

        // print coefficients
        Serial.println("ANGLE_EXTRAPOLATE_MS: " + (String)lastAutoCommand.ANGLE_EXTRAPOLATE_MS);
        Serial.println("TURN_THRESH_1_DEG: " + (String)lastAutoCommand.TURN_THRESH_1_DEG);
        Serial.println("TURN_THRESH_2_DEG: " + (String)lastAutoCommand.TURN_THRESH_2_DEG);
        Serial.println("MAX_TURN_POWER_PERCENT: " + (String)lastAutoCommand.MAX_TURN_POWER_PERCENT);
        Serial.println("MIN_TURN_POWER_PERCENT: " + (String)lastAutoCommand.MIN_TURN_POWER_PERCENT);
        Serial.println("SCALE_DOWN_MOVEMENT_PERCENT: " + (String)lastAutoCommand.SCALE_DOWN_MOVEMENT_PERCENT);
        
#endif
        // invert the turn if we need to
        if (lastAutoCommand.invertTurn)
        {
            command.turn *= -1;
        }

#ifdef ENABLE_AUTONOMOUS_DRIVE
        // drive the robot using the imu
        Drive(command);
        // still drive the weapons with the last drive command
        DriveWeapons(command);

        // set status led for autonomous drive
        digitalWrite(STATUS_3_LED_PIN, HIGH);
#endif
    }
    else if (lastMessage.type == DRIVE_COMMAND)
    {
        // add to the logger
        logger.updateRadioCommandData(lastDriveCommand.movement,
                                      lastDriveCommand.turn,
                                      lastDriveCommand.frontWeaponPower,
                                      lastDriveCommand.backWeaponPower);
#ifdef DEBUG_AUTO
        Serial.println("Manual drive command translated to: " + (String)lastDriveCommand.movement + " " + (String)lastDriveCommand.turn);
#endif
        // drive the robot, weapons, self righter
        Drive(lastDriveCommand);
        DriveWeapons(lastDriveCommand);
        DriveSelfRighter(lastDriveCommand);
    }
}

/**
 * Drives the LEDs based on the message
 */
void DriveLEDs(RobotMessage &message)
{
    static unsigned char lastFETTemps[4] = {0, 0, 0, 0};
    static long flashStartTime = 0;

    // color variable
    static CRGB color = CRGB::Blue;

    // if the message is a radio message
    if (message.type == RobotMessageType::RADIO_DATA)
    {
        // green if recent packet
        if (millis() - lastReceiveTime < 100)
        {
            color = CRGB::Green;
        }
        // yellow if delay is high
        else if (message.radioData.averageDelayMS > 30)
        {
            color = CRGB::Yellow;
        }
        // red if no packets
        else
        {
            color = CRGB::Red;
        }
    }
    // blink whenever a vesc temp comes back valid
    else if (message.type == RobotMessageType::CAN_DATA)
    {
        // if any of the fet temps changed
        for (int i = 0; i < 4; i++)
        {
            // turned on
            if ((int) message.canData.escFETTemp[i] > 0 && (int) lastFETTemps[i] == 0)
            {
                flashStartTime = millis();
            }
        }

        // save the last temps
        for (int i = 0; i < 4; i++)
        {
            lastFETTemps[i] = message.canData.escFETTemp[i];
        }
    }

    if (millis() - flashStartTime < 1000)
    {
        // for first 300 ms, turn on
        if (millis() - flashStartTime < 333)
        {
            // purple
            color = CRGB::Blue;
        }
        else if (millis() - flashStartTime < 666)
        {
            // off
            color = CRGB::Black;
        }
        else
        {
            // purple
            color = CRGB::Blue;
        }
    }

    // set the color
    for (int i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = color;
    }

    // show the color
    FastLED.show();
}

void rx_loop()
{
    // wait for a message to be available
    WaitForRadioData();

    // drive with the latest message if there is one
    DriveWithLatestMessage();

    // Compute response message
    RobotMessage message = Update();

    DriveLEDs(message);

    // send the message
    SendOutput result = rx_radio.Send(message);
    logger.updateRadioData((int)result, 0, 0, 0);

    if (result != SEND_SUCCESS)
    {
        Serial.println("Failed to send radio message. Result: " + (String)result);
    }

    float fetTemps[4];
    float voltages[4];
    float currents[4];
    float motorTemps[4];
    int erpms[4];
    float dutyCycle[4];

    vesc.GetFloatFETTemps(fetTemps);
    vesc.GetFloatVolts(voltages);
    vesc.GetFloatCurrents(currents);
    vesc.GetFloatMotorTemps(motorTemps);
    vesc.GetIntRPMs(erpms);
    vesc.GetFloatDutyCycle(dutyCycle);

    logger.updateVescData(fetTemps, voltages, currents, motorTemps, erpms, dutyCycle);

    logger.update();
}