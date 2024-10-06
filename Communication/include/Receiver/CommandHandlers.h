#pragma once

#include "Communication.h"
#include "Hardware.h"

#define LED_BRIGHTNESS 60
#define NUM_LEDS 14

/**
 * Applys the drive command to the robot
 */
void Drive(DriveCommand &command);

/**
 * Applys the weapon command to the robot
 */
void DriveWeapons(DriveCommand &command);

/**
 * Drives the self righting motor
 */
void DriveSelfRighter(DriveCommand &command);

void DriveWithMessage(DriverStationMessage &msg);

/**
 * Drives the LEDs based on the message
 */
void DriveLEDs(RobotMessage &message);

bool ErrorCheckMessage(DriverStationMessage &msg);