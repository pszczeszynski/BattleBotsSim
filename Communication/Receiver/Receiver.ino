/*
 * nRF24L01 Receiver Test Software
 *
 * Exercises the nRF24L01 Module.  This code runs on the receiver 'slave' device.
 * Use the nRF24L01_Transmitter_Test software for the transmitting 'master' device
 *
 * This uses the RF24.h library which can be installed from the Arduino IDE
 * Pins used for the SPI interface are determined by the Arduino being used.
 * The other two pins are arbitrary and can be changed if needed.  Redfine them in the RF24
 * statement.  Default shown here is to use pins 7 &
 */
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "../Communication.h"

RF24 radio(14, 10);              // CE, CSN      // Define instance of RF24 object called 'radio' and define pins used
const byte address[6] = "000s01"; // Define address/pipe to use. This can be any 5 alphnumeric letters/numbers
//===============================================================================
//  Initialization
//===============================================================================
void setup()
{
    Serial.begin(9600);                // Start serial port to display messages on Serial Monitor Window
    radio.begin();                     // Start instance of the radio object
    radio.openReadingPipe(0, address); // Setup pipe to read data to the address that was defined
    radio.openWritingPipe(address);    // Setup pipe to write data to the address that was defined
    radio.setPALevel(RF24_PA_MAX);     // Set the Power Amplified level to MAX in this case
    radio.startListening();            // We are going to be the receiver, so we need to start listening
}

/**
 * Applys the drive command to the robot
*/
void Drive(DriveCommand& command)
{
    // print the message to the serial monitor window
    serial.println("Received drive command movement: " + command.movement + ", turn " + command.turn);
}

/**
 * Gets the acceleration of the robot
*/
Point getAccel()
{
    Point accel = {0, 0, 0};

    // get accelerometer data and set accel

    return accel;
}

/**
 * Gets the velocity of the robot
*/
Point getVelocity(Point accel)
{
    Point v = {0, 0, 0};

    // integrate accel to get velocity

    // save velocity for next time

    return v;
}

/**
 * Computes response message of robot state data
*/
RobotMessage update()
{
    RobotMessage ret {0};

    // get accelerometer data and set accel
    ret.accel = getAccel();

    // now compute velocity
    ret.velocity = getVelocity(ret.accel);

    // get gyro data and set gyro
    ret.rotation = getRotation();

    return ret;
}


//===============================================================================
//  Main
//===============================================================================
void loop()
{
    if (radio.available())
    {
        // Read the incoming message 
        DriveCommand command;
        radio.read(&command, sizeof(command));

        // Drive the robot
        Drive(command);

        // Compute robot state
        RobotMessage message = update();

        // Send data back to transmitter and driver station
        radio.write(&message, sizeof(message));
    }
}