#include "common_utils/common_utils.h"

float calcDistance(const Point& p1, const Point& p2) {
  float distance =
      sqrtf((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
  return distance;
}

float calcDistance(const Point2f& p1, const Point2f& p2) {
  float distance =
      sqrtf((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
  return distance;
}

float calcDegreeAngle(const Point& startPoint, const Point& endPoint) {
  float angle = atan2((endPoint.y - startPoint.y), (endPoint.x - startPoint.x));
  angle = RADIAN_TO_DEGREE(angle);

  return angle;
}

float calcDegreeAngle(const Point2f& startPoint, const Point2f& endPoint) {
  float angle = atan2((endPoint.y - startPoint.y), (endPoint.x - startPoint.x));
  angle = RADIAN_TO_DEGREE(angle);

  return angle;
}

float calcRadianAngle(const Point& startPoint, const Point& endPoint) {
  float angle = atan2((endPoint.y - startPoint.y), (endPoint.x - startPoint.x));

  return angle;
}

float calcPathAngle(const vector<Point>& pathPoints, int index) {
  if (index > pathPoints.size() - 2) {
    return 0;
  }

  float angle = calcDegreeAngle(pathPoints[index], pathPoints[index + 1]);

  return angle;
}

float calcAngleDiff(float angle1, float angle2) {
  // degree差值//
  if (angle1 < 0) {
    angle1 += 360.0f;
  }

  if (angle2 < 0) {
    angle2 += 360.0f;
  }

  float diff = abs(angle1 - angle2);
  if (diff > 180.0f) {
    diff = 360.0f - diff;
  }

  return diff;
}

RotateDirection calcRotateDirection(float robot_angle, float path_angle) {
  RotateDirection rotateDirection;

  if (robot_angle < 0) {
    robot_angle += 360.0f;
  }

  if (path_angle < 0) {
    path_angle += 360.0f;
  }

  if (robot_angle > path_angle) {
    rotateDirection =
        (robot_angle - path_angle) > 180.0f ? ANTICLOCKWISE : CLOCKWISE;
  } else {
    rotateDirection =
        (path_angle - robot_angle) > 180.0f ? CLOCKWISE : ANTICLOCKWISE;
  }
  return rotateDirection;
}
