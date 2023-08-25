#pragma once

#include <fstream>
#include <iostream>
#include <sstream>
#include "Globals.h"

#define DEFINE_GLOBAL_VARIABLE(type, name, defaultValue) \
    extern type name;                                    \
    void load##name(const std::string &value);           \
    std::string save##name();

#define DECLARE_GLOBAL_VARIABLE(name) \
    {                                 \
        #name, save##name, load##name \
    }

extern int TURN_THRESH_1_DEG;
extern int TURN_THRESH_2_DEG;
extern int MAX_TURN_POWER_PERCENT;
extern int MIN_TURN_POWER_PERCENT;
extern int SCALE_DOWN_MOVEMENT_PERCENT;
extern int ORBIT_ANGLE_EXTRAPOLATE_MS;
extern int GTP_ANGLE_EXTRAPOLATE_MS;
extern int POSITION_EXTRAPOLATE_MS;
extern int OPPONENT_POSITION_EXTRAPOLATE_MS;
extern int ORBIT_RADIUS;
extern int PURE_PURSUIT_RADIUS;
extern int ORBIT_RADIUS_MOVAVG_SPEED;
extern int MASTER_MOVE_SCALE_PERCENT;
extern int MASTER_TURN_SCALE_PERCENT;
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
extern int MIN_OPPONENT_BLOB_SIZE;
extern int MOTION_LOW_THRESHOLD;


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
