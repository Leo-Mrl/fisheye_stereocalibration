#ifndef __P3P_INCLUDE__
#define __P3P_INCLUDE__

#include <Eigen/Dense>
#include <vector>
#include "config.hpp"

class Triangle;

/**
Pose from 3 Points estimator (assume the 3 points are NOT collinear)
*/
class P3P {
public:
  /**
  Given a set of three points in a reference frame, and measures in the camera frame, estimate what is the transformation between the reference and the camera frame.
  [x1,y1] ~(project) cameraTreference * reference[0]
  @param cameraTreference the transformation between the reference Frame and the camera frame
  @param reference the set of three points in the reference frame
  @param x1 the X coordinate of the first measure (in meters)
  @param y1 the Y coordinate of the first measure (in meters)
  @param x2 the X coordinate of the second measure (in meters)
  @param y2 the Y coordinate of the second measure (in meters)
  @param x3 the X coordinate of the third measure (in meters)
  @param y3 the Y coordinate of the third measure (in meters)
  */
  static bool estimate(std::vector<Eigen::Matrix4d> & cameraTreference, const Triangle & reference, double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3, double z3);
};

#endif
