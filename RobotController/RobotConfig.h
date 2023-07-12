#pragma once

#include <fstream>
#include <iostream>
#include <sstream>

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
extern int ANGLE_EXTRAPOLATE_MS;
extern int POSITION_EXTRAPOLATE_MS;
extern int OPPONENT_POSITION_EXTRAPOLATE_MS;
extern int ORBIT_RADIUS;
extern int ORBIT_DTHETA_DEG;
extern bool IS_RUNNING;
extern int MASTER_SPEED_SCALE_PERCENT;


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
