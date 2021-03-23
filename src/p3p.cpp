#include "p3p.hpp"

#include "point.hpp"
#include "triangle.hpp"
#include "polynomialQuartic.hpp"
#include <iostream>
#include <limits>

bool P3P::estimate(std::vector<Eigen::Matrix4d> & cameraTreference, const Triangle & reference, double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3, double z3) {

  cameraTreference.clear();
  /*See random sample consensus, a paradigm for model fitting with applications to image analyss and automated cartography*/

  /**
  Put measures on the unit sphere.
  */
  double n1 = sqrt(x1*x1 + y1*y1 + z1*z1);
  double n2 = sqrt(x2*x2 + y2*y2 + z2*z2);
  double n3 = sqrt(x3*x3 + y3*y3 + z3*z3);
  double nx1 = x1 / n1;
  double ny1 = y1 / n1;
  double nz1 = z1 / n1;
  double nx2 = x2 / n2;
  double ny2 = y2 / n2;
  double nz2 = z2 / n2;
  double nx3 = x3 / n3;
  double ny3 = y3 / n3;
  double nz3 = z3 / n3;

  /**
  Compute angles
  */
  double cos12 = nx1 * nx2 + ny1 * ny2 + nz1 * nz2;
  double cos13 = nx1 * nx3 + ny1 * ny3 + nz1 * nz3;
  double cos23 = nx2 * nx3 + ny2 * ny3 + nz2 * nz3;

  /**
  Compute distances in 3d
  */
  Point p1 = reference.getVertex(0);
  Point p2 = reference.getVertex(1);
  Point p3 = reference.getVertex(2);
  double l12 = (p2-p1).distanceFromOrigin();
  double l13 = (p3-p1).distanceFromOrigin();
  double l23 = (p3-p2).distanceFromOrigin();

  double l12_squared = l12 * l12;
  double l13_squared = l13 * l13;
  double l23_squared = l23 * l23;
  double K1 = l23_squared / l13_squared;
  double K2 = l23_squared / l12_squared;

  /*Polynomial coefficients*/
  double a = (K1 * K2 - K1 - K2) * (K1 * K2 - K1 - K2) - 4.0 * K1 * K2 * cos23 * cos23;
  double b = 4.0 * (K1 * K2 - K1 - K2) * K2 * (1.0 - K1) * cos12 + 4.0 * K1 * cos23 * ((K1 * K2 + K2 - K1) * cos13 + 2.0 * K2 * cos12 * cos23);
  double c = (2.0 * K2 * (1.0 - K1) * cos12) * (2.0 * K2 * (1.0 - K1) * cos12) + 2.0 * (K1 * K2 + K1 - K2) * (K1 * K2 - K1 - K2) + 4.0 * K1 * ((K1 - K2) * cos23 * cos23 + (1.0 - K2) * K1 * cos13 * cos13 - 2.0 * K2 * (1.0 + K1) * cos12 * cos13 * cos23);
  double d = 4.0 * (K1 * K2 + K1 - K2) * K2 * (1.0 - K1) * cos12 + 4.0 * K1 * ((K1 * K2 - K1 + K2) * cos13 * cos23 + 2.0 * K1 * K2 * cos12 * cos13 * cos13);
  double e = (K1 * K2 + K1 - K2) * (K1 * K2 + K1 - K2) - 4.0 * K1 * K1 * K2 * cos13 * cos13;

  PolynomialQuartic poly(a,b,c,d,e);
  if (!poly.computeRoots()) {
    return false;
  }

  for (unsigned int i = 0; i < poly.getValidRootsCount(); i++) {
    Polynomial::Root root = poly.getRoot(i);
    if (root.imag() > 1e-12) {
      continue;
    }

    double x = root.real();

    double test_val = poly.evaluate(x);
    if (test_val > 1e-12) {
      continue;
    }


    /*a and b are derivated very simply*/
    double a = l12 / sqrt(x*x  - 2.0 * x * cos12 + 1.0);
    double b = a * x;
    double c = -1;

    /*C is more complex to get and may need a quadratic solve*/
    double m0 = 1.0 - K1;
    double m1 = 1.0;
    double p0 = 2.0  * (K1 * cos13 - x * cos23);
    double p1 = 2.0 * (-x * cos23);
    double q0 = x * x  - K1;
    double q1 = x * x * (1.0 - K2) + 2.0 * x * K2 * cos12 - K2;

    double denom = m0 * q1 - m1 * q0;

    if (fabs(denom) < std::numeric_limits<double>::epsilon()) {
      double p1 = cos13;
      double a2 = a * a;
      double p2 = cos13*cos13 + (l13 * l13 - a2)/a2;
      double c1 = (p1 + p2) * a;
      double c2 = (p1 - p2) * a;

      double v1 = b * b + c1 * c1 - 2.0 * b * c1 * cos23;
      double v2 = b * b + c2 * c2 - 2.0 * b * c2 * cos23;

      /* Check that a solution is correct ...*/
      if (fabs(v1 - l23_squared) < std::numeric_limits<double>::epsilon()){
        c = c1;
      }
      else if (fabs(v2 - l23_squared) < std::numeric_limits<double>::epsilon()) {
        c = c2;
      }
      else {
        continue;
      }
    }
    else {
      double y = (p1 * q0 - p0 * q1) / denom;
      c = y * a;
    }

    /*a,b,c are the length of the "optical center to point" vectors*/
    Triangle camera_triangle(Point(nx1*a, ny1*a, nz1*a), Point(nx2*b, ny2*b, nz2*b), Point(nx3*c, ny3*c, nz3*c));

    Eigen::Matrix4d cTo;
    camera_triangle.getRelativeTransformation(cTo, reference);
    cameraTreference.push_back(cTo);
  }

  return true;
}
