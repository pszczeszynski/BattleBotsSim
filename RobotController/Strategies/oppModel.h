#include <vector>
#include <queue>
#include <unordered_map>
#include <set>
#include <cmath>
#include <opencv2/core.hpp>

class oppModel {
public:

    oppModel();
    void updatePos(cv::Point2f position, float angle, float moveVelocity,  float time);


private:

    // // max velocities it can hit
    // float maxMoveVelocity;
    // float maxAngularVelocity;

    // // accels
    // float moveAccel;
    // float angularAccel;

    // // current velocities
    // float moveVelocity;
    // float angularVelocity;

    
};


