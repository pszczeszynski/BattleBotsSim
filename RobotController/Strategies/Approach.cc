#include "Approach.h"
#include "../MathUtils.h"
#include "../RobotConfig.h"



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

    curvePoints.emplace_back(center + cv::Point2f(cos(endAngle), sin(endAngle))); // center point is always included

    while(abs(offsetAngle) < abs(sweepRange)) {

        float radius = radiusEquation(offsetAngle);
        float currAngle = angle_wrap(endAngle + offsetAngle);

        cv::Point2f newPoint = center + radius*cv::Point2f(cos(currAngle), sin(currAngle));
        curvePoints.emplace_back(newPoint);

        offsetAngle += 1.0f*TO_RAD * (CW? -1 : 1); // angle increment of curve generation 5
    }
}



// defines the radius as a function of offset angle
float Approach::radiusEquation(float offsetAngle) {

    float offsetPositive = offsetAngle * (CW? -1 : 1);
    if(offsetPositive < 0) { return 0; }

    float radAtAngle = APPROACH_RAD_AT_ANGLE;
    float angle = APPROACH_REF_ANGLE_DEG * TO_RAD;
    float a = APPROACH_CURVE_EXPONENT;

    float b = (radAtAngle - collisionRad) / pow(angle, a);
    float radius = b*pow(offsetPositive, a) + collisionRad;

    return radius;
}


// closest point on path, needs to be within a radial offset of point
cv::Point2f Approach::closestPoint(cv::Point2f point, float advance) {

    float absAngle = angle(center, point);
    float radHere = radiusEquation(offsetToPoint(point));

    

    // approximate advance distance using an angular advance and curr radius
    float angleAdvance = 0;
    if(radHere > 1.0f) { angleAdvance = advance / radHere; } 

    absAngle += angleAdvance * (CW? 1 : -1);
    absAngle = angle_wrap(absAngle);

    cv::Point2f testPoint = center + cv::Point2f(cos(absAngle), sin(absAngle));
    radHere = radiusEquation(offsetToPoint(testPoint));

    return center + radHere*cv::Point2f(cos(absAngle), sin(absAngle));
}



// wrap an angle within the sweep angle
float Approach::wrapInSweep(float angle) {
    float posBound = sweepRange > 0? sweepRange : 2*M_PI + sweepRange;
    return angle_wrap_upper(angle, posBound);
}


// abs angle of the path
float Approach::absPathAngleHere(float offsetAngle) {

    float delta = 0.5f * TO_RAD; if(CW) { delta *= -1; } // small delta from offset angle used to scan for angle

    float offsetPositive = offsetAngle * (CW? -1 : 1);
    float defaultReturn = angle_wrap(endAngle + M_PI);
    if(offsetPositive < 0.001) { return defaultReturn; }

    float angle0 = angle_wrap(offsetAngle - delta + endAngle);
    float angle1 = angle_wrap(offsetAngle + delta + endAngle);

    float rad0 = radiusEquation(offsetAngle - delta);
    float rad1 = radiusEquation(offsetAngle + delta);

    if(abs(rad0) < 0.001f && abs(rad1) < 0.001f) { return angle_wrap(endAngle + M_PI); }

    cv::Point2f point0 = rad0 * cv::Point2f(cos(angle0), sin(angle0));
    cv::Point2f point1 = rad1 * cv::Point2f(cos(angle1), sin(angle1));

    return angle_wrap(angle(point0, point1) + M_PI); // technically reversed because we follow path backward
}


// calculates curvature at this offset angle
float Approach::curvatureHere(float offsetAngle, bool display) {

    float delta = 0.5f * TO_RAD; if(CW) { delta *= -1; } // small delta from offset angle used to scan for curvature

    float offsetPositive = offsetAngle * (CW? -1 : 1);
    if(offsetPositive < 0) { return 0; } // assume 0 curvature at path end



    // make 3 scan points right around the offset angle

    float angle0 = wrapInSweep(offsetAngle - delta);
    float angle1 = wrapInSweep(offsetAngle);
    float angle2 = wrapInSweep(offsetAngle + delta);

    cv::Point2f point0 = radiusEquation(angle0) * cv::Point2f(cos(angle0), sin(angle0));
    cv::Point2f point1 = radiusEquation(angle1) * cv::Point2f(cos(angle1), sin(angle1));
    cv::Point2f point2 = radiusEquation(angle2) * cv::Point2f(cos(angle2), sin(angle2));

    // calculate angle between line segments
    float segment1Angle = angle(point0, point1);
    float segment2Angle = angle(point1, point2);
    float deltaAngle = angle_wrap(segment2Angle - segment1Angle);
    
    // use arc length equation to get curvature
    float arcLength = radiusEquation(offsetAngle) * abs(delta); // s = r * theta    

    return deltaAngle / arcLength;
}



// what offset angle relative to end angle is this point at, wraps at sweep angle
float Approach::offsetToPoint(cv::Point2f point) {

    float absAngle = angle(center, point);
    return wrapInSweep(absAngle - endAngle);
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



// if a point is inside the curve shape
bool Approach::outsideCurve(cv::Point2f point) {

    float radiusHere = radiusEquation(offsetToPoint(point));
    float distanceFromCenter = cv::norm(center - point);
    return distanceFromCenter > radiusHere;
}



// tangent point relative to another point
cv::Point2f Approach::tangentPoint(cv::Point2f point) {

    float mostTangentAngle = 0;
    cv::Point mostTangentPoint = cv::Point2f(0, 0);

    // can't be tangent if the point is inside
    if(!outsideCurve(point)) { return mostTangentPoint; } 



    // from here point is guarenteed to be outside curve

    float sweepToCurr = offsetToPoint(point);
    // std::cout << "sweep to curr = " << sweepToCurr

    for(int i = 0; i < curvePoints.size(); i++) {

        float angleToPoint = angle(point, curvePoints[i]);
        float offset = angle_wrap(angleToPoint - mostTangentAngle);
        offset *= CW? 1 : -1;

        // set the point to the most tangent point
        if(offset < 0 || i == 0) { 
            mostTangentAngle = angleToPoint;
            mostTangentPoint = curvePoints[i];
        }

        float sweepToPoint = offsetToPoint(curvePoints[i]);
        if(abs(sweepToPoint) > abs(sweepToCurr)) { break; }
    }

    return mostTangentPoint;
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
cv::Point2f Approach::getCenter() { return center; }
bool Approach::getCW() { return CW; }
float Approach::getEndAngle() { return endAngle; }