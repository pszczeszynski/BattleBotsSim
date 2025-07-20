#include "Line.h"
#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>

//—we put helper functions in anonymous namespace to avoid polluting global scope
namespace {
    inline float cross(const cv::Point2f& u, const cv::Point2f& v) {
        return u.x * v.y - u.y * v.x;
    }
    inline float dot(const cv::Point2f& u, const cv::Point2f& v) {
        return u.x * v.x + u.y * v.y;
    }
}

Line::Line(const cv::Point2f& p1, const cv::Point2f& p2)
  : point1(p1),
    point2(p2),
    // precompute vector and squared length
    _d(p2 - p1),
    _len2(dot(_d, _d))
{
    length = std::sqrt(_len2);
}

cv::Point2f Line::closestLinePoint(const cv::Point2f& P) const {
    // degenerate segment?
    if (_len2 == 0.0f)
        return point1;

    // project (P - A) onto d, then clamp t in [0,1]
    float t = dot(P - point1, _d) / _len2;
    t = std::clamp(t, 0.0f, 1.0f);

    return point1 + _d * t;
}

float Line::howClosePoint(const cv::Point2f& P) const {
    cv::Point2f C = closestLinePoint(P);
    return cv::norm(P - C);
}

float Line::getLength() const {
    return length;
}

bool Line::doesIntersectLine(const Line& other) const {
    // r = B - A, s = D - C
    const cv::Point2f& A = point1;
    const cv::Point2f& B = point2;
    const cv::Point2f& C = other.point1;
    const cv::Point2f& D = other.point2;

    cv::Point2f r = B - A;
    cv::Point2f s = D - C;
    cv::Point2f Delta = C - A;

    float rxs = cross(r, s);
    const float eps = 1e-6f;

    if (std::fabs(rxs) < eps) {
        // parallel
        if (std::fabs(cross(Delta, r)) >= eps)
            return false;  // non‐collinear

        // collinear: check for overlap on projection onto r
        float rlen2 = dot(r, r);
        float t0 = dot(Delta, r) / rlen2;
        float t1 = t0 + dot(s, r) / rlen2;
        float lo = std::max(std::min(t0, t1), 0.0f);
        float hi = std::min(std::max(t0, t1), 1.0f);
        return lo <= hi;
    }

    // not parallel: compute intersection params
    float t = cross(Delta, s) / rxs;
    float u = cross(Delta, r) / rxs;
    return (t >= 0.0f && t <= 1.0f && u >= 0.0f && u <= 1.0f);
}

std::pair<cv::Point2f, cv::Point2f> Line::getLinePoints() const {
    return { point1, point2 };
}
