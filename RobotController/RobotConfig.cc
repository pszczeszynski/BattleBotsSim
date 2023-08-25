#include "RobotConfig.h"

#define DEFINE_GLOBAL_VARIABLE(type, name, defaultValue) \
    type name = defaultValue; \
    void load##name(const std::string& value) { name = convertValue<type>(value); } \
    std::string save##name() { return convertValueToString(name); }

template<typename T>
std::string convertValueToString(const T& value)
{
    std::ostringstream oss;
    oss << value;
    return oss.str();
}

template<typename T>
T convertValue(const std::string& value)
{
    std::istringstream iss(value);
    T convertedValue;
    iss >> convertedValue;
    return convertedValue;
}

DEFINE_GLOBAL_VARIABLE(int, TURN_THRESH_1_DEG, 80)
DEFINE_GLOBAL_VARIABLE(int, TURN_THRESH_2_DEG, 10)
DEFINE_GLOBAL_VARIABLE(int, MAX_TURN_POWER_PERCENT, 100)
DEFINE_GLOBAL_VARIABLE(int, MIN_TURN_POWER_PERCENT, 10)
DEFINE_GLOBAL_VARIABLE(int, SCALE_DOWN_MOVEMENT_PERCENT, 100)
DEFINE_GLOBAL_VARIABLE(int, ORBIT_ANGLE_EXTRAPOLATE_MS, 150)
DEFINE_GLOBAL_VARIABLE(int, GTP_ANGLE_EXTRAPOLATE_MS, 150)
DEFINE_GLOBAL_VARIABLE(int, POSITION_EXTRAPOLATE_MS, 150)
DEFINE_GLOBAL_VARIABLE(int, OPPONENT_POSITION_EXTRAPOLATE_MS, 150)
DEFINE_GLOBAL_VARIABLE(int, ORBIT_RADIUS, 100)
DEFINE_GLOBAL_VARIABLE(int, PURE_PURSUIT_RADIUS, 40)
DEFINE_GLOBAL_VARIABLE(int, ORBIT_RADIUS_MOVAVG_SPEED, 5)
DEFINE_GLOBAL_VARIABLE(int, MASTER_MOVE_SCALE_PERCENT, 100)
DEFINE_GLOBAL_VARIABLE(int, MASTER_TURN_SCALE_PERCENT, 100)
DEFINE_GLOBAL_VARIABLE(int, preprocess_tl_x, 0)
DEFINE_GLOBAL_VARIABLE(int, preprocess_tl_y, 0)
DEFINE_GLOBAL_VARIABLE(int, preprocess_tr_x, WIDTH)
DEFINE_GLOBAL_VARIABLE(int, preprocess_tr_y, 0)
DEFINE_GLOBAL_VARIABLE(int, preprocess_bl_x, 0)
DEFINE_GLOBAL_VARIABLE(int, preprocess_bl_y, HEIGHT * 0.6667)
DEFINE_GLOBAL_VARIABLE(int, preprocess_br_x, WIDTH)
DEFINE_GLOBAL_VARIABLE(int, preprocess_br_y, HEIGHT * 0.6667)
DEFINE_GLOBAL_VARIABLE(int, MIN_INTER_SEND_TIME_MS, 4)
DEFINE_GLOBAL_VARIABLE(int, MIN_ROBOT_BLOB_SIZE, WIDTH * 0.04)
DEFINE_GLOBAL_VARIABLE(int, MAX_ROBOT_BLOB_SIZE, WIDTH * 0.08)
DEFINE_GLOBAL_VARIABLE(int, MIN_OPPONENT_BLOB_SIZE, WIDTH * 0.04)
DEFINE_GLOBAL_VARIABLE(int, MAX_OPPONENT_BLOB_SIZE, WIDTH * 0.04)
DEFINE_GLOBAL_VARIABLE(int, MOTION_LOW_THRESHOLD, 14)


GlobalVariable globalVariables[] = {
    DECLARE_GLOBAL_VARIABLE(TURN_THRESH_1_DEG),
    DECLARE_GLOBAL_VARIABLE(TURN_THRESH_2_DEG),
    DECLARE_GLOBAL_VARIABLE(MAX_TURN_POWER_PERCENT),
    DECLARE_GLOBAL_VARIABLE(MIN_TURN_POWER_PERCENT),
    DECLARE_GLOBAL_VARIABLE(SCALE_DOWN_MOVEMENT_PERCENT),
    DECLARE_GLOBAL_VARIABLE(ORBIT_ANGLE_EXTRAPOLATE_MS),
    DECLARE_GLOBAL_VARIABLE(GTP_ANGLE_EXTRAPOLATE_MS),
    DECLARE_GLOBAL_VARIABLE(POSITION_EXTRAPOLATE_MS),
    DECLARE_GLOBAL_VARIABLE(OPPONENT_POSITION_EXTRAPOLATE_MS),
    DECLARE_GLOBAL_VARIABLE(ORBIT_RADIUS),
    DECLARE_GLOBAL_VARIABLE(PURE_PURSUIT_RADIUS),
    DECLARE_GLOBAL_VARIABLE(ORBIT_RADIUS_MOVAVG_SPEED),
    DECLARE_GLOBAL_VARIABLE(MASTER_MOVE_SCALE_PERCENT),
    DECLARE_GLOBAL_VARIABLE(MASTER_TURN_SCALE_PERCENT),
    DECLARE_GLOBAL_VARIABLE(preprocess_tl_x),
    DECLARE_GLOBAL_VARIABLE(preprocess_tl_y),
    DECLARE_GLOBAL_VARIABLE(preprocess_tr_x),
    DECLARE_GLOBAL_VARIABLE(preprocess_tr_y),
    DECLARE_GLOBAL_VARIABLE(preprocess_bl_x),
    DECLARE_GLOBAL_VARIABLE(preprocess_bl_y),
    DECLARE_GLOBAL_VARIABLE(preprocess_br_x),
    DECLARE_GLOBAL_VARIABLE(preprocess_br_y),
    DECLARE_GLOBAL_VARIABLE(MIN_INTER_SEND_TIME_MS),
    DECLARE_GLOBAL_VARIABLE(MIN_ROBOT_BLOB_SIZE),
    DECLARE_GLOBAL_VARIABLE(MAX_ROBOT_BLOB_SIZE),
    DECLARE_GLOBAL_VARIABLE(MIN_OPPONENT_BLOB_SIZE),
    DECLARE_GLOBAL_VARIABLE(MAX_OPPONENT_BLOB_SIZE),
    DECLARE_GLOBAL_VARIABLE(MOTION_LOW_THRESHOLD)
};

void saveGlobalVariablesToFile(const std::string& filename)
{
    std::ofstream file(filename);

    if (file.is_open())
    {
        for (const auto& variable : globalVariables)
        {
            file << variable.name << " = " << variable.saveFunc() << '\n';
        }

        file.close();
        std::cout << "Global variables saved to file: " << filename << std::endl;
    }
    else
    {
        std::cerr << "Unable to open file for writing: " << filename << std::endl;
    }
}

void loadGlobalVariablesFromFile(const std::string& filename)
{
    std::ifstream file(filename);

    if (file.is_open())
    {
        std::string line;

        while (std::getline(file, line))
        {
            std::istringstream iss(line);
            std::string variableName;
            std::string equalsSign;
            std::string variableValue;

            if (iss >> variableName >> equalsSign >> variableValue && equalsSign == "=")
            {
                for (auto& variable : globalVariables)
                {
                    if (variable.name == variableName)
                    {
                        variable.loadFunc(variableValue);
                        break;
                    }
                }
            }
        }

        file.close();
        std::cout << "Global variables loaded from file: " << filename << std::endl;
    }
    else
    {
        std::cerr << "Unable to open file for reading: " << filename << std::endl;
    }
}
