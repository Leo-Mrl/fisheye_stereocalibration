#include "point.hpp"
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <iostream>

Point::Point()
{
  coords[0] = 0;
  coords[1] = 0;
  coords[2] = 0;
}

Point::Point(double X,double Y,double Z)
{
  coords[0] = X;
  coords[1] = Y;
  coords[2] = Z;
}

Point::Point(double X, double Y) {
  coords[0] = X;
  coords[1] = Y;
  coords[2] = 0;
}

double* Point::getPointParametersPtr() {
  return coords.data();
}

void Point::setPointParameters(const double * vals) {
  coords[0] = vals[0];
  coords[1] = vals[1];
  coords[2] = vals[2];
}

double Point::getX() const {
  return coords[0];
}

double Point::getY() const  {
  return coords[1];
}

double Point::getZ() const {
  return coords[2];
}

void Point::setX(double X) {
  coords[0] = X;
}

void Point::setY(double Y) {
  coords[1] = Y;
}

void Point::setZ(double Z) {
  coords[2] = Z;
}

Point Point::operator-(const Point & other) {
  Point ret;

  ret.coords[0] = coords[0] - other.coords[0];
  ret.coords[1] = coords[1] - other.coords[1];
  ret.coords[2] = coords[2] - other.coords[2];

  return ret;
}

double Point::distanceFromOrigin() {
  return sqrt(coords[0] * coords[0] + coords[1] * coords[1] + coords[2] * coords[2]);
}

Point Point::transform(const Eigen::Matrix4d & SE3) const {

  double X,Y,Z;

  /**
  Just multiply SE3*this...
  */
  X = SE3(0,0) * coords[0] + SE3(0,1) * coords[1] + SE3(0,2) * coords[2] + SE3(0,3);
  Y = SE3(1,0) * coords[0] + SE3(1,1) * coords[1] + SE3(1,2) * coords[2] + SE3(1,3);
  Z = SE3(2,0) * coords[0] + SE3(2,1) * coords[1] + SE3(2,2) * coords[2] + SE3(2,3);

  return Point(X,Y,Z);
}
