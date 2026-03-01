#include "MathUtils.h"

double AngleBetweenPoints(double x1, double y1, double x2, double y2) {
  return std::atan2(y2 - y1, x2 - x1);
}

double angle_wrap(double angle_rad) {
  angle_rad = fmod(angle_rad, 2 * M_PI);
  if (angle_rad <= -M_PI) {
    angle_rad += 2 * M_PI;
  } else if (angle_rad > M_PI) {
    angle_rad -= 2 * M_PI;
  }
  return angle_rad;
}

float angle_wrap(float angle_rad) {
  angle_rad = fmod(angle_rad, 2 * M_PI);
  if (angle_rad <= -M_PI) {
    angle_rad += 2 * M_PI;
  } else if (angle_rad > M_PI) {
    angle_rad -= 2 * M_PI;
  }
  return angle_rad;
}

double norm(Point2f p) { return std::sqrt(p.x * p.x + p.y * p.y); }

cv::Point2f rotate_point(cv::Point2f p, double angle) {
  return cv::Point2f(p.x * std::cos(angle) - p.y * std::sin(angle),
                     p.x * std::sin(angle) + p.y * std::cos(angle));
}

cv::Point3f rotate3dPoint(cv::Point3f p, double angle_rad) {
  // Store original values before modification to avoid using modified p.y in
  // p.z calculation
  double original_y = p.y;
  double original_z = p.z;
  p.y = original_y * cos(angle_rad) - original_z * sin(angle_rad);
  p.z = original_y * sin(angle_rad) + original_z * cos(angle_rad);
  return p;
}

float distance(cv::Point2f point1, cv::Point2f point2) {
  float dx = point1.x - point2.x;
  float dy = point1.y - point2.y;

  return std::sqrt(dx * dx + dy * dy);
}

// A function to calculate the tangent points of a circle given the center,
// radius and an external point
void CalculateTangentPoints(const cv::Point2f& center, float radius,
                            const cv::Point2f& externalPoint,
                            cv::Point2f& tangent1, cv::Point2f& tangent2) {
  // Calculate the distance between the external point and the center
  float dist = cv::norm(center - externalPoint);
  if (dist < radius) {
    // std::cerr << "ERROR CalculateTangentPoints: The external point is inside
    // the circle" << std::endl;
    return;
  }

  // Calculate the angle for the two tangent lines
  float angle = acos(radius / dist);
  float centerToExternalPointAngle =
      atan2(externalPoint.y - center.y, externalPoint.x - center.x);

  // Calculate the angles for the two tangent points
  float angle1 = centerToExternalPointAngle + angle;
  float angle2 = centerToExternalPointAngle - angle;

  // Calculate the tangent points using the angles and the radius
  tangent1 = cv::Point2f(center.x + radius * cos(angle1),
                         center.y + radius * sin(angle1));
  tangent2 = cv::Point2f(center.x + radius * cos(angle2),
                         center.y + radius * sin(angle2));
}

cv::Point2f InterpolatePoints(const cv::Point2f& p1, const cv::Point2f& p2,
                              double ratio) {
  cv::Point2f diff = p2 - p1;
  return p1 + diff * ratio;
}

Angle InterpolateAngles(Angle a1, Angle a2, double ratio) {
  // Find the difference between the angles and make sure it's the shortest path
  double diff = angle_wrap(a2 - a1);

  // Interpolate using the ratio
  return Angle(a1 + diff * ratio);
}

double Interpolate(double start, double end, double ratio) {
  double diff = end - start;
  return start + diff * std::clamp(ratio, 0.0, 1.0);
}

std::vector<cv::Point2f> CirclesIntersect(cv::Point2f center1, float r1,
                                          cv::Point2f center2, float r2) {
  std::vector<cv::Point2f> intersections;

  float x1 = center1.x, y1 = center1.y;
  float x2 = center2.x, y2 = center2.y;

  // Compute the distance between the centers of the circles
  float dist = cv::norm(center1 - center2);

  // Check if there's no solution
  if (dist > r1 + r2 || dist < std::abs(r1 - r2) || (dist == 0 && r1 == r2)) {
    return intersections;  // Return an empty vector
  }

  // Compute a, b, and c
  float a = (r1 * r1 - r2 * r2 + dist * dist) / (2 * dist);
  float h = std::sqrt(r1 * r1 - a * a);
  float cx2 = x1 + a * (x2 - x1) / dist;
  float cy2 = y1 + a * (y2 - y1) / dist;

  // Compute the intersection points
  float ix1 = cx2 + h * (y2 - y1) / dist;
  float iy1 = cy2 - h * (x2 - x1) / dist;
  float ix2 = cx2 - h * (y2 - y1) / dist;
  float iy2 = cy2 + h * (x2 - x1) / dist;

  intersections.emplace_back(ix1, iy1);
  intersections.emplace_back(ix2, iy2);

  return intersections;
}

/**
 * Finds the intersection of a circle and a line segment. Returns a vector
 * The vector may have 0, 1, or 2 points
 *
 * @param circleCenter The center of the circle
 * @param circleRadius The radius of the circle
 * @param lineStart The start of the line segment
 * @param lineEnd The end of the line segment
 * @return A vector of points of intersection
 */
std::vector<cv::Point2f> CircleLineSegmentIntersect(cv::Point2f circleCenter,
                                                    float circleRadius,
                                                    cv::Point2f lineStart,
                                                    cv::Point2f lineEnd) {
  std::vector<cv::Point2f> intersections;

  // Shift line segment and circle center so the circle's center is at the
  // origin
  cv::Point2f start = lineStart - circleCenter;
  cv::Point2f end = lineEnd - circleCenter;

  // Calculate quadratic coefficients
  cv::Point2f d = end - start;       // direction vector of the line segment
  float dr = d.x * d.x + d.y * d.y;  // dr squared
  float D = start.x * end.y - end.x * start.y;  // determinant

  // Discriminant
  float discriminant = circleRadius * circleRadius * dr - D * D;

  // Check for no intersection (discriminant < 0)
  if (discriminant < 0) {
    return intersections;
  }

  // Calculate intersection points
  float sqrtDiscriminant = std::sqrt(discriminant);
  float signDy = (d.y < 0) ? -1.0f : 1.0f;
  float x1 = (D * d.y + signDy * d.x * sqrtDiscriminant) / dr;
  float y1 = (-D * d.x + std::abs(d.y) * sqrtDiscriminant) / dr;
  float x2 = (D * d.y - signDy * d.x * sqrtDiscriminant) / dr;
  float y2 = (-D * d.x - std::abs(d.y) * sqrtDiscriminant) / dr;

  // Check if the intersection points are on the line segment
  auto isBetween = [](const cv::Point2f& start, const cv::Point2f& end,
                      const cv::Point2f& pt) {
    // Using an epsilon to handle floating-point precision issues
    float dotProduct = (pt.x - start.x) * (end.x - start.x) +
                       (pt.y - start.y) * (end.y - start.y);
    float squaredLength = (end.x - start.x) * (end.x - start.x) +
                          (end.y - start.y) * (end.y - start.y);
    return dotProduct >= 0 && dotProduct <= squaredLength;
  };

  cv::Point2f intersection1(x1, y1);
  cv::Point2f intersection2(x2, y2);

  if (discriminant == 0) {  // One intersection
    if (isBetween(start, end, intersection1)) {
      intersections.push_back(intersection1 + circleCenter);
    }
  } else {  // Two intersections
    if (isBetween(start, end, intersection1)) {
      intersections.push_back(intersection1 + circleCenter);
    }
    if (isBetween(start, end, intersection2)) {
      intersections.push_back(intersection2 + circleCenter);
    }
  }

  return intersections;
}

// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
/**
 * Finds the intersection of two lines, or returns false.
 *
 * @param o1 The first point of the first line
 * @param p1 The second point of the first line
 * @param o2 The first point of the second line
 * @param p2 The second point of the second line
 * @param r The point of intersection (returned by reference)
 */
bool SegmentsIntersect(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2,
                       cv::Point2f p2, cv::Point2f& r) {
  cv::Point2f x = o2 - o1;
  cv::Point2f d1 = p1 - o1;
  cv::Point2f d2 = p2 - o2;

  float cross = d1.x * d2.y - d1.y * d2.x;

  // if cross is zero, the lines are parallel
  if (std::abs(cross) < 1e-8) return false;

  double t1 = (x.x * d2.y - x.y * d2.x) / cross;
  double t2 =
      (x.x * d1.y - x.y * d1.x) / cross;  // Calculate t2 similarly to t1

  // Check if the intersection is within the bounds of both line segments
  if (t1 >= 0.0 && t1 <= 1.0 && t2 >= 0.0 && t2 <= 1.0) {
    r = o1 + d1 * t1;
    return true;
  }

  return false;  // Intersection does not lie within both segments
}

bool getClosestPointOnSegment(cv::Point2f start, cv::Point2f end,
                              cv::Point2f point, cv::Point2f& closestPoint) {
  cv::Point2f segment = end - start;
  cv::Point2f startToPoint = point - start;

  double segmentLength = cv::norm(segment);
  double segmentLengthSquared = segmentLength * segmentLength;

  // Project the point onto the segment
  double t = segment.dot(startToPoint) / segmentLengthSquared;

  // Check if the point is outside the segment
  if (t < 0 || t > 1) {
    return false;
  }

  closestPoint = start + t * segment;
  return true;
}
