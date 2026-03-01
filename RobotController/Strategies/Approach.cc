#include "Approach.h"
#include "../MathUtils.h"



// CONSTRUCTOR
Approach::Approach() {}

Approach::Approach(cv::Point2f center, float endAngle, float collisionRad, bool CW) {

    this->center = center;
    this->endAngle = endAngle;
    this->collisionRad = collisionRad;
    this->CW = CW;

    sweepRange = 270.0f*TO_RAD; if(CW) { sweepRange *= -1; }

    generateCurve();
}




// generates the point list
void Approach::generateCurve() {

    curvePoints = {}; // reset just in case
    float offsetAngle = 0.0f; // sweep up to the the specified range for this follow point

    while(abs(offsetAngle) < abs(sweepRange)) {

        float radius = radiusEquation(offsetAngle);
        float currAngle = angle_wrap(endAngle + offsetAngle);

        cv::Point2f newPoint = center + radius*cv::Point2f(cos(currAngle), sin(currAngle));
        curvePoints.emplace_back(newPoint);

        offsetAngle += 1.0f*TO_RAD * (CW? -1 : 1); // 5
    }
}



// defines the radius as a function of offset angle
float Approach::radiusEquation(float offsetAngle) {

    float radAtAngle = 120.0f;
    float angle = 120.0f*TO_RAD;
    float a = 0.65f; // 0.68 controls how sharp the curvature is at the end, lower makes it wait more to curve
    float b = (radAtAngle - collisionRad) / pow(angle, a);

    float radius = b*pow(abs(offsetAngle), a) + collisionRad;

    return radius;
}



// pure pursuit based point to follow curve
cv::Point2f Approach::ppPoint(cv::Point2f currPosition, float ppRad) {

    float closestDistance = 99999.0f;
    int closestIndex = 0;

    // list is ordered, so first point found in the pp rad is the one to follow
    for(int i = 0; i < curvePoints.size(); i++) {

        float distance = cv::norm(curvePoints[i] - currPosition);

        // first point to be in the pp rad is used
        if(distance < ppRad) { 
            if(i == 0) { return center; } // if the first point is in range, go to the center point since we don't have a proper pure pursuit intersection algorithm
            return curvePoints[i]; 
        }

        // save which point was closest to us in case there is no pp intersection
        if(distance < closestDistance || i == 0) {
            closestDistance = distance;
            closestIndex = i;
        }
    }

    // return closest point if there was no pp intersection
    return curvePoints[closestIndex];
}




// absolute angle between 2 points
float Approach::angle(cv::Point2f point1, cv::Point2f point2) {
    return std::atan2(point2.y - point1.y, point2.x - point1.x);
}



// returns follow point to follow this curve with tangency/pure pursuit rule
cv::Point2f Approach::followCurve(cv::Point2f currPosition, float ppRad, bool display) {

    cv::Point2f point = center; // default, will get set to the right point
    if(curvePoints.empty()) { return point; } // no crashy



    // target center directly if you're wrapping around
    float angleHere = angle(center, currPosition);

    float wrapOffset = (2*M_PI) - abs(sweepRange); wrapOffset *= CW? 1 : -1;
    float offsetAngle = angle_wrap(endAngle - angleHere - wrapOffset) + wrapOffset;

    if(offsetAngle*(CW? 1 : -1) < 0.0f*TO_RAD) { return point; }


    // from here we're guarenteed to not be in the 0 radius region



    float radiusHere = radiusEquation(offsetAngle);
    float distanceFromCenter = cv::norm(center - currPosition);
    bool outsideCurve = distanceFromCenter > radiusHere;



    // if we're outside the curve, go to the tangent point
    // only consider points that are more towards the beginning of the curve than our angular position
    if(outsideCurve) {

        float mostTangentAngle = 0;

        float absAngleToCurr = angle(center, currPosition); // angle of line from center to curr pos
        float wrapOffset = M_PI - abs(sweepRange); if(!CW) { wrapOffset *= -1; }
        float sweepToCurr = angle_wrap(absAngleToCurr - endAngle - wrapOffset) + wrapOffset;

        if(display) {
            std::cout << "sweepToCurr = " << sweepToCurr*TO_DEG; // << ", sweepToPoint = " << sweepToPoint*TO_DEG;
        }

        for(int i = 0; i < curvePoints.size(); i++) {

            float absAngleToPoint = angle(center, curvePoints[i]);
            float sweepToPoint = angle_wrap(absAngleToPoint - endAngle - wrapOffset) + wrapOffset;

            if(abs(sweepToPoint) > abs(sweepToCurr)) { break; }

            float angleToPoint = angle(currPosition, curvePoints[i]);
            float offset = angle_wrap(angleToPoint - mostTangentAngle);
            offset *= CW? 1 : -1;

            // set the point to the most tangent point
            if(offset < 0 || i == 0) { 
                mostTangentAngle = angleToPoint;
                point = curvePoints[i];
            }
        }

        float absAngleToPoint = angle(center, point);
        float sweepToPoint = angle_wrap(absAngleToPoint - endAngle - wrapOffset) + wrapOffset;

        if(display) {
            std::cout << ", sweepToPoint = " << sweepToPoint*TO_DEG;
        }

        // tangent point is good to use as long as it's further away than the pp rad
        if(cv::norm(point - currPosition) > ppRad) { return point; }
    }


    // set the point to the pp point, will be compared against if inside the curve
    point = ppPoint(currPosition, ppRad);


    // if we're inside the curve, drive out at specified angle
    if(!outsideCurve) {

        // angles are offset relative to line that starts at opp and connects to orb
        float angleAtRadius = 90.0f*TO_RAD; // 90
        float angleAtCollision = 30.0f*TO_RAD; // 40


        // drive angle is linearly interpolated from collision angle to at the radius
        float distanceToCollide = std::max(distanceFromCenter - collisionRad, 0.0f);
        float radiusPercent = std::clamp(distanceToCollide / (radiusHere - collisionRad), 0.0f, 1.0f);
        float driveAngle = angleAtCollision + radiusPercent*(angleAtRadius - angleAtCollision);




        // clip the drive angle if the pp angle is more extreme
        if(distanceFromCenter > radiusHere - ppRad) {

            float angleToPPGlobal = angle(currPosition, point);
            float angleToPP = angle_wrap(angleToPPGlobal - angleHere); // angle to pp point from center's perspective

            driveAngle = std::max(driveAngle, abs(angleToPP));
        }

        
        driveAngle = angle_wrap(angleHere + driveAngle*(CW? 1 : -1));
        

        // safe_circle(RobotController::GetInstance().GetDrawingImage(), ppFollow, 2, cv::Scalar(255, 0, 255), 5);

        // generate a point at the pp rad at the correct angle
        point = currPosition + ppRad*cv::Point2f(cos(driveAngle), sin(driveAngle));
    }

    return point;
}





std::vector<cv::Point2f> Approach::getCurvePoints() { return curvePoints; }