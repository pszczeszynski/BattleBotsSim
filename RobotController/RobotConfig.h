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

extern int TURN_THRESH_1_DEG;
extern int TURN_THRESH_2_DEG;
extern int MAX_TURN_POWER_PERCENT;
extern int MIN_TURN_POWER_PERCENT;
extern bool INVERT_TURN;
extern bool INVERT_MOVEMENT;
extern int SCALE_DOWN_MOVEMENT_PERCENT;
extern int ORBIT_ANGLE_EXTRAPOLATE_MS;
extern int KILL_ANGLE_EXTRAPOLATE_MS;
extern int POSITION_EXTRAPOLATE_MS;
extern int OPPONENT_POSITION_EXTRAPOLATE_MS;
extern int ORBIT_RADIUS;
extern int PURE_PURSUIT_RADIUS;
extern int ORBIT_RADIUS_MOVAVG_SPEED;
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
extern int HEU_FOREGROUND_RATIO;
extern int HEU_ROBOT_PROCESSORS;
// Heuristic settings
extern int HEU_BACKGROUND_AVGING;
extern int HEU_UNTRACKED_MOVING_BLOB_AVGING;
extern int HEU_FOREGROUND_THRESHOLD;
extern int HEU_FOREGROUND_MINSIZE;
extern int HEU_FOREGROUND_BLURSIZE;
extern int HEU_FOREGROUND_BUFFER;
extern int HEU_POSITION_TO_CENTER_SPEED;
extern int HEU_VELOCITY_AVERAGING;
extern int RADIO_CHANNEL;
extern int GO_AROUND_RADIUS;
extern float FISHEYE_SCALE;
extern float FISHEYE_FL;
extern float FISHEYE_Y;

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
