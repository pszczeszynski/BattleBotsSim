#include "Receiver.h"

#include <FastLED.h>

#include "Hardware.h"
#include "Receiver/CANBUS.h"
#include "Receiver/CommandHandlers.h"
#include "Receiver/IMU.h"
#include "Receiver/Logging.h"
#include "Receiver/Vesc.h"
#include "PowerMonitor.h"
#include "Radio.h"
#include "Utils.h"


// RECEIVER SPECIFIC BUILD OPTION DEFINES

// RECEIVER SPECIFIC CONFIG DEFINES

#define ARBITRATION_TIMEOUT_MS 500

// these interrupts are ordered by priority highest -> lowest
#define WATCHDOG_PRIORITY 0x70
#define WATCHDOG_INTERVAL 10000 // 10000us = 100hz

#define CAN_EVENTS_PRIORITY 0x71
#define CAN_EVENTS_INTERVAL 100 // 10kHz

// radio interrupt should be 0x80 priority
// Place watchdog above radio and everything else below

#define IMU_PRIORITY 0xFD
#define IMU_INTERVAL 10000 // microseconds -> 2000us = 500hz

#define ANGLE_SYNC_PRIORITY 0xFE
#define ANGLE_SYNC_INTERVAL 100000 // 100,000us = 10hz

#define STOP_ROBOT_TIMEOUT_MS 250

#define POWER_STATS_TELEMETRY_INTERVAL 50 // every 50 packets -> updates at 4hz
#define RADIO_STATS_TELEMETRY_INITERVAL 20 // every 20 packets -> 10hz
#define CAN_DATA_TELEMETRY_INTERVAL 10 // every 10 packets -> 20 

// RECEIVER SPECIFIC GLOBAL VARIABLES
extern enum board_placement placement;
uint8_t radioChannel = 0;
uint32_t lastReinitRadioTime = 0;
volatile uint32_t lastReceiveTime = 0;
CRGB leds[NUM_LEDS];
volatile bool noPacketWatchdogTrigger = false;
volatile int validMessageCount = 0;
volatile int invalidMessageCount = 0;
DriveCommand lastDriveCommand;
AutoDrive lastAutoCommand;
DriverStationMessage lastMessage;
int maxReceiveIntervalMs = 0;
volatile uint32_t lastPacketID = 0;
volatile bool receivedPacketSinceBoot = false;

// RECEIVER COMPONENTS
IMU imu;
Radio<RobotMessage, DriverStationMessage> rxRadio;
PowerMonitor monitor;
Logger logger{};
CANBUS can{};
VESC vesc{&can, LEFT_MOTOR_CAN_ID, RIGHT_MOTOR_CAN_ID, FRONT_WEAPON_CAN_ID, BACK_WEAPON_CAN_ID};

// FUNCTION PROTOTYPES
RobotMessage GenerateTelemetryPacket();

// RECEIVER ISRs

IntervalTimer imuTimer;
void ServiceImu()
{
    imu.Update();
}

IntervalTimer angleSyncTimer;
void ServiceAngleSync()
{
    // Only send current angle after a set time
    // Ensures that if one board reboots it does not send incorrect values
    if(millis() > BOOT_GYRO_MERGE_MS)
    {
        CANMessage syncMessage;
        syncMessage.type = ANGLE_SYNC;
        syncMessage.angle = float(imu.getRotation());

        can.SendTeensy(&syncMessage);
    }
}

IntervalTimer packetWatchdogTimer;
void ServicePacketWatchdog()
{
    // NOTE: don't stop robot unless comms are lost on all three boards
    // IE lastReceiveTime must update when another board receives a packet
    if (millis() - lastReceiveTime > STOP_ROBOT_TIMEOUT_MS)
    {
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
            // TODO: self righter
            noPacketWatchdogTrigger = true;
        }
    }
}

IntervalTimer CANEventsTimer;
void ServiceCANEvents()
{
    can.Update();
}

void HandlePacket()
{
    if (!rxRadio.Available()) return;
    DriverStationMessage msg = rxRadio.Receive();

    RobotMessage return_msg = GenerateTelemetryPacket();
    DriveLEDs(return_msg);
    return_msg.timestamp = msg.timestamp;
    SendOutput result = rxRadio.Send(return_msg);
    logger.updateRadioData((int)result, 0, 0, 0);

    if(ErrorCheckMessage(msg))
    {
        DriveWithMessage(msg);
        lastReceiveTime = millis();
        validMessageCount++;
        maxReceiveIntervalMs = max(maxReceiveIntervalMs, millis() - lastReceiveTime);
    }
    else
    {
        invalidMessageCount++;
    }

    noPacketWatchdogTrigger = false;
    digitalWriteFast(STATUS_1_LED_PIN, HIGH);
    receivedPacketSinceBoot = true;
}

// RECEIVER HELPER FUNCTIONS
void DetermineChannel()
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

void GenerateIMUPacket(RobotMessage &msg)
{
    msg.type = IMU_DATA;

    // get accelerometer data and set accelsdf
    Point accel = imu.getAccel();
    msg.imuData.accelX = accel.x;
    msg.imuData.accelY = accel.y;

    // get gyro data and set gyro
    msg.imuData.rotation = imu.getRotation();
    // calculate rotation velocity
    msg.imuData.rotationVelocity = imu.getRotationVelocity();
}

void GenerateCANTelemetryPacket(RobotMessage &msg)
{
    msg.type = CAN_DATA;
    vesc.GetCurrents(msg.canData.motorCurrent);
    vesc.GetVolts(msg.canData.motorVoltage);
    vesc.GetRPMs(msg.canData.motorERPM);
    vesc.GetFETTemps(msg.canData.escFETTemp);
    vesc.GetMotorTemps(msg.canData.motorTemp);
}

void GenerateRadioStatsPacket(RobotMessage &msg)
{
    static unsigned long lastRadioStatsRefreshTime = 0;

    msg.type = RADIO_DATA;
    // send average delay (0 if no messages received yet)
    msg.radioData.averageDelayMS = validMessageCount == 0 ? -1 : ((float) (millis() - lastRadioStatsRefreshTime) / validMessageCount);
    // set invalid message count
    msg.radioData.invalidPackets = (short) invalidMessageCount;
    // set max delay
    msg.radioData.maxDelayMS = maxReceiveIntervalMs;

    // set the last drive command fields
    msg.radioData.movement = lastDriveCommand.movement;
    msg.radioData.turn = lastDriveCommand.turn;
    msg.radioData.frontWeaponPower = lastDriveCommand.frontWeaponPower;
    msg.radioData.backWeaponPower = lastDriveCommand.backWeaponPower;
    
    // reset max delay
    if (millis() - lastRadioStatsRefreshTime > 1000)
    {
        invalidMessageCount = 0;
        validMessageCount = 0;
        maxReceiveIntervalMs = 0;
        lastRadioStatsRefreshTime = millis();
    }
}

void GeneratePowerStatsPacket(RobotMessage &msg)
{
    msg.type = BOARD_TELEMETRY_DATA;

    msg.boardTelemetryData.current_5v = monitor.get5vCurrent();
    msg.boardTelemetryData.voltage_3v3 = monitor.get3v3Voltage();
    msg.boardTelemetryData.voltage_5v = monitor.get5vVoltage();
    msg.boardTelemetryData.voltage_batt = monitor.getBattVoltage();
}

RobotMessage GenerateTelemetryPacket()
{
    static uint32_t updateCount = 0;
    RobotMessage msg;

    // offset them by increments of 2 so at a worst case
    // every other packet is an IMU message
    if ((updateCount % CAN_DATA_TELEMETRY_INTERVAL) == 2)
    {
        GenerateCANTelemetryPacket(msg);
    }
    else if ((updateCount % RADIO_STATS_TELEMETRY_INITERVAL) == 4)
    {
        GenerateRadioStatsPacket(msg);
    }
    else if ((updateCount % RADIO_STATS_TELEMETRY_INITERVAL) == 6)
    {
        GeneratePowerStatsPacket(msg);
    }
    else
    {
        GenerateIMUPacket(msg);
        logger.updateIMUData(msg.imuData.rotation, msg.imuData.rotationVelocity, msg.imuData.accelX, msg.imuData.accelY);
    }
    updateCount++;
 
    return msg;
}

void OnTeensyMessage(const CAN_message_t &msg)
{
    CANMessage message;
    memcpy(&message, msg.buf, sizeof(CANMessage));

    switch(message.type)
    {
        case ANGLE_SYNC:
            imu.MergeExternalInput(message.angle);
            break;
        case PING_REQUEST:
            if (message.ping.pingID == CANBUS::GetCanID(placement)) {
                message.type = PING_RESPONSE;
                can.SendTeensy(&message);
            }
            Serial.print(message.ping.pingID);
            Serial.println("received ping");
            break;
        case PING_RESPONSE:
            Serial.println("received response");
            DownsampledPrintf("Received ping response from %x, time taken: %d\n", message.ping.pingID, micros() - message.ping.timestamp);
            break;
        case COMMAND_PACKET_ID:
            lastPacketID = message.packetID;
            lastReceiveTime = millis();
        case CHANNEL_CHANGE:
            if (message.channel.targetTeensyID == CANBUS::GetCanID(placement)) {
                radioChannel = message.channel.newChannel;
                rxRadio.SetChannel(radioChannel);
            }
        default:
            break;
    }
}

// ONE TIME INIT ON RECEIVER BOOT
void rx_setup()
{
    Serial.begin(SERIAL_BAUD);
    Serial.println("RX Setup: boot started");

    if (!logger.init())
    {
        Serial.println("WARNING: logger failed to initialize");
    }

    CANBUS::SetTeensyHandler(&OnTeensyMessage);
    CANBUS::SetVESCHandler(&VESC::OnMessage);

    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);

    imu.Initialize(placement);
    imuTimer.priority(IMU_PRIORITY);
    imuTimer.begin(ServiceImu, IMU_INTERVAL);

    angleSyncTimer.priority(ANGLE_SYNC_PRIORITY);
    angleSyncTimer.begin(ServiceAngleSync, ANGLE_SYNC_INTERVAL);

    CANEventsTimer.priority(CAN_EVENTS_PRIORITY);
    CANEventsTimer.begin(ServiceCANEvents, CAN_EVENTS_INTERVAL);

    packetWatchdogTimer.priority(WATCHDOG_PRIORITY);
    packetWatchdogTimer.begin(ServicePacketWatchdog, WATCHDOG_INTERVAL);

    DetermineChannel();
    delay(100);
    rxRadio.InitRadio(radioChannel);
    lastReinitRadioTime = millis();

    pinMode(STATUS_1_LED_PIN, OUTPUT);
    pinMode(STATUS_2_LED_PIN, OUTPUT);
    pinMode(STATUS_3_LED_PIN, OUTPUT);
    pinMode(STATUS_4_LED_PIN, OUTPUT);

    digitalWrite(STATUS_1_LED_PIN, LOW);
    digitalWrite(STATUS_2_LED_PIN, LOW);
    digitalWrite(STATUS_3_LED_PIN, LOW);
    digitalWrite(STATUS_4_LED_PIN, HIGH);

    Serial.println("RX Setup: boot complete");

    imu.Update();

    attachInterrupt(digitalPinToInterrupt(RADIO_IRQ_PIN), HandlePacket, FALLING);
}

// CONTINUOUSLY CALLED ON RECEIVER
void rx_loop()
{
    if(millis() - lastReceiveTime > STOP_ROBOT_TIMEOUT_MS)
    {
        if (!receivedPacketSinceBoot && millis() - lastReinitRadioTime > 500)
        {
            lastReinitRadioTime = millis();
            Serial.println("Reinit radio");
            rxRadio.InitRadio(radioChannel);
        }
        digitalWriteFast(STATUS_1_LED_PIN, LOW);
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

    monitor.readSensors();
}
