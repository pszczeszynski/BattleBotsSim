#pragma once

#include <fstream>
#include <iostream>
#include <sstream>
#include "Globals.h"

extern std::string SAVE_FILE_NAME;

#define DECLARE_GLOBAL_VARIABLE(name) \
    {                                 \
        #name, save##name, load##name \
    }

extern int TURN_THRESH_1_DEG_ORBIT;
extern int TURN_THRESH_2_DEG_ORBIT;
extern int MAX_TURN_POWER_PERCENT_ORBIT;
extern int MIN_TURN_POWER_PERCENT_ORBIT;
extern int TURN_THRESH_1_DEG_KILL;
extern int TURN_THRESH_2_DEG_KILL;
extern int MAX_TURN_POWER_PERCENT_KILL;
extern int MIN_TURN_POWER_PERCENT_KILL;
extern bool INVERT_TURN;
extern bool INVERT_MOVEMENT;
extern int SCALE_DOWN_MOVEMENT_PERCENT_ORBIT;
extern int SCALE_DOWN_MOVEMENT_PERCENT_KILL;
extern int ORBIT_ANGLE_EXTRAPOLATE_MS;
extern int KILL_ANGLE_EXTRAPOLATE_MS;
extern int POSITION_EXTRAPOLATE_MS;
extern int OPPONENT_POSITION_EXTRAPOLATE_MS;
extern int ORBIT_RADIUS;
extern int PURE_PURSUIT_RADIUS;
extern float ORBIT_RADIUS_MOVAVG_SPEED;
extern int MASTER_MOVE_SCALE_PERCENT;
extern int MASTER_TURN_SCALE_PERCENT;
extern float MAX_FRONT_WEAPON_SPEED;
extern float MAX_BACK_WEAPON_SPEED;
extern int preprocess_tl_x;
extern int preprocess_tl_y;
extern int preprocess_tr_x;
extern int preprocess_tr_y;
extern int preprocess_bl_x;
extern int preprocess_bl_y;
extern int preprocess_br_x;
extern int preprocess_br_y;
extern int MIN_INTER_SEND_TIME_MS;
extern int MIN_ROBOT_BLOB_SIZE;
extern int MAX_ROBOT_BLOB_SIZE;
extern int MIN_OPPONENT_BLOB_SIZE;
extern int MAX_OPPONENT_BLOB_SIZE;
extern int MOTION_LOW_THRESHOLD;
extern float BLOBS_MIN_FPS;
extern bool ROTATION_NET_ENABLED;
extern bool GYRO_ENABLED;
extern int RADIO_CHANNEL;
extern int GO_AROUND_RADIUS;
extern bool LEAD_WITH_BAR;

template <typename T>
std::string convertValueToString(const T &value);

template <typename T>
T convertValue(const std::string &value);

struct GlobalVariable
{
    std::string name;
    std::string (*saveFunc)();
    void (*loadFunc)(const std::string &);
};

extern GlobalVariable globalVariables[];

void saveGlobalVariablesToFile(const std::string &filename);
void loadGlobalVariablesFromFile(const std::string &filename);
