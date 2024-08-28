#include "Receiver/CommandHandlers.h"

#include <FastLED.h>
#include "Arduino.h"

#include "Receiver/Vesc.h"
#include "Receiver/RobotMovement.h"
#include "Receiver/IMU.h"
#include "Receiver/Logging.h"

extern CRGB leds[NUM_LEDS];
extern uint32_t lastReceiveTime;
extern VESC vesc;
extern DriveCommand lastDriveCommand;
extern AutoDrive lastAutoCommand;
extern DriverStationMessage lastMessage;
extern IMU imu;
extern Logger logger;


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

void DriveWeapons(DriveCommand &command)
{
    // Serial.println("Driving weapons with powers: " + String(command.frontWeaponPower) + ", " + String(command.backWeaponPower));
    // set status led
    digitalWrite(STATUS_2_LED_PIN, HIGH);

    vesc.DriveWeapons(command.frontWeaponPower, command.backWeaponPower);
}

void DriveSelfRighter(DriveCommand &command)
{
    // TODO
}

void DriveWithMessage(DriverStationMessage &msg)
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

    if (lastMessage.type == AUTO_DRIVE)
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

bool ErrorCheckMessage(DriverStationMessage &msg)
{
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
            return false;
        }

        // check for NAN
        if (isnan(command.movement) || isnan(command.turn) ||
            isnan(command.frontWeaponPower) || isnan(command.backWeaponPower) ||
            isnan(command.selfRighterPower))
        {
            Serial.println("invalid case 2 (NAN)");
            return false;
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
            return false;
            Serial.println("invalid case 1 (AUTO DRIVE)");
        }

        // check for nan
        if (isnan(msg.autoDrive.movement) || isnan(msg.autoDrive.targetAngle) ||
            isnan(msg.autoDrive.ANGLE_EXTRAPOLATE_MS) || isnan(msg.autoDrive.TURN_THRESH_1_DEG) ||
            isnan(msg.autoDrive.TURN_THRESH_2_DEG) || isnan(msg.autoDrive.MAX_TURN_POWER_PERCENT) ||
            isnan(msg.autoDrive.MIN_TURN_POWER_PERCENT) || isnan(msg.autoDrive.SCALE_DOWN_MOVEMENT_PERCENT))
        {
            return false;
            Serial.println("invalid case 2 (AUTO DRIVE)");
        }
    }
    // otherwise, the message is invalid
    else
    {
        Serial.println("invalid case 3 invalid type");

        return false;
    }
    return true;
}