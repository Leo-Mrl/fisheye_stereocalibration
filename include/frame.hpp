#ifndef __FRAME_HPP__
#define __FRAME_HPP__

#include <array>
#include <Eigen/Dense>

#include "config.hpp"
#include "point.hpp"

class Point;

/**
A Frame object defines a geometric 3D frame which is used to represent coordinates.
*/
class VS_EXPORT Frame {
public:

  /**
  Constructor.
  Initialize frame to be the identity transformation
  */
  Frame();

  /**
  Destructor
  */
  virtual ~Frame();

  /**
  Update the pose of the frame.
  @param SE3 the pose matrix
  */
  void setPose(const Eigen::Matrix4d & SE3);

  /**
  Retrieve the frame pose
  @return a pose matrix
  */
  Eigen::Matrix4d getPose();

  /**
  Retrieve the frame pose as a 6 vector (axis*angle ; translation)
  @return a pose vector
  */
  Eigen::Matrix<double, 6, 1> getPoseVector();

  /**
  Retrieve the frame parameters
  The frame parameters are the 16th elements of the 4*4 pose matrix (row major).
  @return a pointer to a 16 parameters vector
  */
  double * getPoseParametersPtr();

  /**
  Set the frame parameters using an input vector
  @param vals a 16 values input vector (A 4*4 matrix in row major)
  */
  void setPoseParameters(const double * vals);

  /**
  Transform a point using this Frame
  @param p the original point
  @return a transformed point
  */
  Point transform(const Point & p);

  /**
  Transform a point using this Frame
  @param p the original point
  @return a transformed point
  */
  Point transformInverse(const Point & p);

protected:

  /**
  Actual pose data for the frame
  */
  std::array<double, 16> posedata;

  /**
  Make a eigen matrix from the posedata buffer
  */
  Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > pose;
};

#endif
