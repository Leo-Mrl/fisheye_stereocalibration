#include "triangle.hpp"

#include <Eigen/SVD>
#include <Eigen/Dense>
#include <iostream>

Triangle::Triangle() {
}

Triangle::Triangle(const Point & p1, const Point &p2, const Point &p3) {
  vertices[0] = p1;
  vertices[1] = p2;
  vertices[2] = p3;
}

void Triangle::getRelativeTransformation(Eigen::Matrix4d & thisTother, const Triangle & other) {

  Point c1 = centroid();
  Point c2 = other.centroid();
  Triangle t1 = nullcenter();
  Triangle t2 = other.nullcenter();

  Eigen::Matrix3d M;
  M.fill(0);

  for (int t = 0; t < 3; t++) {
    double * p1 = t1.vertices[t].getPointParametersPtr();
    double * p2 = t2.vertices[t].getPointParametersPtr();

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        M(i, j) += p1[i] * p2[j];
      }
    }
  }

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();

  /*Make sure det(R)==1*/
  double minval = svd.singularValues()(0);
  int pos = 0;
  if (svd.singularValues()(1) < minval) {
    pos = 1;
    minval = svd.singularValues()(1);
  }
  if (svd.singularValues()(2) < minval) {
    pos = 2;
    minval = svd.singularValues()(2);
  }
  Eigen::Matrix3d W = Eigen::Matrix3d::Identity();
  W(pos, pos) = 1.0 / R.determinant();
  R = svd.matrixU() * W * svd.matrixV().transpose();

  /*Estimate translation*/
  double Rc2[3];
  for (int i = 0; i < 3; i++) {
    Rc2[i] = R(i, 0) * c2.getX() + R(i, 1) * c2.getY() + R(i, 2) * c2.getZ();
  }

  thisTother.fill(0);
  thisTother(3, 3) = 1.0;
  thisTother(0, 3) = c1.getX() - Rc2[0];
  thisTother(1, 3) = c1.getY() - Rc2[1];
  thisTother(2, 3) = c1.getZ() - Rc2[2];

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      thisTother(i, j) = R(i, j);
    }
  }
}

Point Triangle::centroid() const {
  double tx = (vertices[0].getX() + vertices[1].getX() + vertices[2].getX()) / 3.0;
  double ty = (vertices[0].getY() + vertices[1].getY() + vertices[2].getY()) / 3.0;
  double tz = (vertices[0].getZ() + vertices[1].getZ() + vertices[2].getZ()) / 3.0;
  Point center(tx, ty, tz);
  return center;
}

Triangle Triangle::nullcenter() const {
  Point c = this->centroid();
  Triangle ret(Point(vertices[0]) - c, Point(vertices[1]) - c, Point(vertices[2]) - c);
  return ret;
}

Point Triangle::getVertex(unsigned int index) const {
  if (index > 2) {
    return Point();
  }

  return vertices[index];
}
