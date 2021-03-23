#ifndef POINT_HPP_
#define POINT_HPP_

#include <array>
#include <Eigen/Dense>
#include "config.hpp"

/**
Define a 3D point and operations on points
*/
class VS_EXPORT Point {
private :
  /*Coordinates are stores as an array to make minimization easier*/
  std::array<double, 3> coords;

public :
  /**
  Default constructor
  */
  Point();

  /**
  Point Constructor
  @param X first coordinate
  @param Y second coordinate
  @param Z third coordinate
  */
  Point(double X,double Y,double Z);

  /**
  Point Constructor. Last coordinate is set to 0.
  @param X first coordinate
  @param Y second coordinate
  */
  Point(double X, double Y);

  /**
  Get a pointer to the coordinates array
  @return a pointer to a three doubles array
  */
  double * getPointParametersPtr();

  /**
  Get the original X coordinate
  @return X
  */
  double getX() const;

  /**
  Get the original Y coordinate
  @return Y
  */
  double getY() const;

  /**
  Get the original Z coordinate
  @return Z
  */
  double getZ() const;

  /**
  Update the point using an input array.
  Input array must be a 3 double array with [X,Y,Z]
  @param vals input array
  */
  void setPointParameters(const double * vals);

  /**
  Set the X coordinate of the point
  @param X the first coordinate
  */
  void setX(double X);

  /**
  Set the Y coordinate of the point
  @param Y the first coordinate
  */
  void setY(double Y);

  /**
  Set the Z coordinate of the point
  @param Z the first coordinate
  */
  void setZ(double Z);

  /**
  Substract two points. This is a nonsense, but it makes life easier.
  @param b the other point to Substract
  @return a point this - b
  */
  Point operator-(const Point & b);

  /**
  Compute euclidean distance of the points to its origin frame
  @return a distance [0; inf[
  */
  double distanceFromOrigin();

  /**
  Change the frame of a point. Uses a full solid transformation (R+t)
  @param SE3 the transformation homogeneous matrix.
  @return the transformed point.
  */
  Point transform(const Eigen::Matrix4d & SE3) const;
};



#endif
