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

extern int NEURAL_BRIGHTNESS_ADJUST;
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
extern bool MANUAL_AUTODRIVE;
extern int SCALE_DOWN_MOVEMENT_PERCENT_ORBIT;
extern int SCALE_DOWN_MOVEMENT_PERCENT_KILL;
extern int ORBIT_KD_PERCENT;
extern int KILL_KD_PERCENT;
extern float KD_FILTER_TIME_CONSTANT;
extern int MASTER_MOVE_SCALE_PERCENT;
extern int MASTER_TURN_SCALE_PERCENT;
extern float MAX_FRONT_WEAPON_SPEED;
extern float MAX_BACK_WEAPON_SPEED;
extern float SELF_RIGHTER_IDLE_CURRENT;
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
extern int BLOB_MATCHING_DIST_THRESHOLD;
extern int MIN_OPPONENT_BLOB_SIZE;
extern int MAX_OPPONENT_BLOB_SIZE;
extern int MOTION_LOW_THRESHOLD;
extern float BLOBS_MIN_FPS;
extern bool POSITION_NET_ENABLED;
extern bool ROTATION_NET_ENABLED;
extern bool GYRO_ENABLED;
extern int HEU_FOREGROUND_RATIO;
extern int HEU_ROBOT_PROCESSORS;
// Heuristic settings
extern int HEU_BACKGROUND_AVGING;
extern int HEU_UNTRACKED_MOVING_BLOB_AVGING;
extern int HEU_FOREGROUND_THRESHOLD;
extern int HEU_FOREGROUND_THRESHOLD_INIT;
extern int HEU_FOREGROUND_MINSIZE;
extern int HEU_FOREGROUND_MAXSIZE;
extern int HEU_FOREGROUND_BLURSIZE;
extern int HEU_BACKGROUND_BLURSIZE;
extern int HEU_BACKGROUND_BLURSIZE_INIT;
extern int HEU_BLUR_COUNT;
extern int HEU_BLUR_COUNT_INIT;
extern bool HEU_HEAL_BG_INIT;
extern float HEU_BRIGHTNESS_CORR;
extern int HEU_FOREGROUND_BUFFER;
extern int HEU_POSITION_TO_CENTER_SPEED;
extern int HEU_VELOCITY_AVERAGING;
extern int HEU_VEL_TO_ANGLE_MIN;
extern int HEU_VEL_TO_ANGLE_K;
extern float HEU_ROT_GAIN;
extern int RADIO_CHANNEL;
extern int SECONDARY_RADIO_CHANNEL;
extern bool AUTO_SWITCH_CHANNEL;
extern bool FUSE_IMU;
extern bool INTEGRATE_GYRO_VEL;
extern int MAX_AVERAGE_DELAY_MS;
extern int SWITCH_COOLDOWN_MS;
extern int GO_AROUND_RADIUS;
extern float RADIUS_CURVE_X0;
extern float RADIUS_CURVE_X1;
extern float RADIUS_CURVE_X2;
extern float RADIUS_CURVE_X3;
extern float RADIUS_CURVE_Y0;
extern float RADIUS_CURVE_Y1;
extern float RADIUS_CURVE_Y2;
extern float RADIUS_CURVE_Y3;
extern float ASTAR_PP_RAD_SLOW;
extern float ASTAR_PP_RAD_FAST;
extern float ASTAR_PP_SPEED_FAST;
extern float FISHEYE_SCALE;
extern float FISHEYE_FL;
extern float FISHEYE_Y;
extern bool FISHEYE_ENABLE;
extern bool WASD_ENABLED;
extern bool ODO_BLOB_ENABLED;
extern bool ODO_HEUR_ENABLED;
extern bool ODO_IMU_ENABLED;
extern bool ODO_OPENCV_ENABLED;
extern float NN_MIN_CONFIDENCE;
extern float TRACKER_MIN_CONFIDENCE;
extern float ANGLE_FUSE_CONF_THRESH;
extern float ANGLE_FUSE_SPEED;
extern float CAMERA_GAIN;
extern float IMAGE_INTENSITY_TIME_CONSTANT;
extern int IMAGE_INTENSITY_SQR_size;
extern int IMAGE_INTENSITY_SQR_1_x, IMAGE_INTENSITY_SQR_1_y;
extern int IMAGE_INTENSITY_SQR_2_x, IMAGE_INTENSITY_SQR_2_y;
extern int IMAGE_INTENSITY_SQR_3_x, IMAGE_INTENSITY_SQR_3_y;
extern int IMAGE_INTENSITY_SQR_4_x, IMAGE_INTENSITY_SQR_4_y;
extern int STARTING_LEFT_TL_x;
extern int STARTING_LEFT_TL_y;
extern int STARTING_LEFT_BR_x;
extern int STARTING_LEFT_BR_y;
extern int STARTING_RIGHT_TL_x;
extern int STARTING_RIGHT_TL_y;
extern int STARTING_RIGHT_BR_x;
extern int STARTING_RIGHT_BR_y;
extern bool LOG_ODOMETRY_DATA;
extern bool PLAYBACK_PREPROCESS;
extern std::string VISION_TRACKING_GUI;
extern bool CONTROL_OPPONENT_ENABLED;

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
