#include <fstream>
#include <iostream>
#include <sstream>

// Macro to define a global variable and its corresponding load/save function
#define DEFINE_GLOBAL_VARIABLE(type, name, defaultValue) \
    type name = defaultValue; \
    void load##name(const std::string& value) { name = convertValue<type>(value); } \
    std::string save##name() { return convertValueToString(name); }

#define DECLARE_GLOBAL_VARIABLE(name) \
    {#name, save##name, load##name}

// Helper function to convert a value to string
template<typename T>
std::string convertValueToString(const T& value)
{
    std::ostringstream oss;
    oss << value;
    return oss.str();
}

// Helper function to convert a string to a value
template<typename T>
T convertValue(const std::string& value)
{
    std::istringstream iss(value);
    T convertedValue;
    iss >> convertedValue;
    std::cout << "converted value: " << convertedValue << std::endl;
    return convertedValue;
}

// Define a struct for global variables
struct GlobalVariable {
    std::string name;
    std::string (*saveFunc)();
    void (*loadFunc)(const std::string&);
};

// Define the global variables and their load/save functions using the macros
DEFINE_GLOBAL_VARIABLE(int, TURN_THRESH_1_DEG, 80)
DEFINE_GLOBAL_VARIABLE(int, TURN_THRESH_2_DEG, 10)
DEFINE_GLOBAL_VARIABLE(int, MAX_TURN_POWER_PERCENT, 100)
DEFINE_GLOBAL_VARIABLE(int, MIN_TURN_POWER_PERCENT, 10)
DEFINE_GLOBAL_VARIABLE(int, SCALE_DOWN_MOVEMENT_PERCENT, 100)
DEFINE_GLOBAL_VARIABLE(int, ANGLE_EXTRAPOLATE_MS, 150)
DEFINE_GLOBAL_VARIABLE(int, POSITION_EXTRAPOLATE_MS, 150)
DEFINE_GLOBAL_VARIABLE(int, OPPONENT_POSITION_EXTRAPOLATE_MS, 150)
DEFINE_GLOBAL_VARIABLE(int, ORBIT_RADIUS, 100)
DEFINE_GLOBAL_VARIABLE(int, ORBIT_DTHETA_DEG, 40)
DEFINE_GLOBAL_VARIABLE(bool, IS_RUNNING, false)
DEFINE_GLOBAL_VARIABLE(int, MASTER_SPEED_SCALE_PERCENT, 100)

// Initialize the array of global variables
GlobalVariable globalVariables[] = {
    DECLARE_GLOBAL_VARIABLE(TURN_THRESH_1_DEG),
    DECLARE_GLOBAL_VARIABLE(TURN_THRESH_2_DEG),
    DECLARE_GLOBAL_VARIABLE(MAX_TURN_POWER_PERCENT),
    DECLARE_GLOBAL_VARIABLE(MIN_TURN_POWER_PERCENT),
    DECLARE_GLOBAL_VARIABLE(SCALE_DOWN_MOVEMENT_PERCENT),
    DECLARE_GLOBAL_VARIABLE(ANGLE_EXTRAPOLATE_MS),
    DECLARE_GLOBAL_VARIABLE(POSITION_EXTRAPOLATE_MS),
    DECLARE_GLOBAL_VARIABLE(OPPONENT_POSITION_EXTRAPOLATE_MS),
    DECLARE_GLOBAL_VARIABLE(ORBIT_RADIUS),
    DECLARE_GLOBAL_VARIABLE(ORBIT_DTHETA_DEG),
    DECLARE_GLOBAL_VARIABLE(IS_RUNNING),
    DECLARE_GLOBAL_VARIABLE(MASTER_SPEED_SCALE_PERCENT)
};

// Helper function to save the global variables to a file
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

// Helper function to load the global variables from a file
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