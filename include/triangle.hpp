#ifndef __TRIANGLE_HPP__
#define __TRIANGLE_HPP__


#include <Eigen/Dense>
#include "point.hpp"
#include "config.hpp"

/**
A Triangle definition
*/
class VS_EXPORT Triangle {
public:
  Triangle();

  /**
  Constructor
  @param p1 the first triangle point
  @param p2 the second triangle point
  @param p3 the third triangle point
  */
  Triangle(const Point & p1, const Point &p2, const Point & p3);

  /**
  Get one of the three vertices of the triangle
  @param index index of the vertices [0-2]
  @return the point vertex
  */
  Point getVertex(unsigned int index) const;

  /**
  Given a reference triangle and the current triangle, estimate the solid transformation between them
  @param thisTother the solid transformation (a 3D rotation + translation)
  @param other the reference triangle
  */
  void getRelativeTransformation(Eigen::Matrix4d  & thisTother, const Triangle & other);

  /**
  Estimate the triangle's centroid
  @return the centroid defined as a point
  */
  Point centroid() const;

  /**
  Transform the triangle such that its centroid is the origin
  @return the transformed triangle
  */
  Triangle nullcenter() const;

private:
  Point vertices[3];
};

#endif
