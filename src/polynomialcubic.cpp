#include "polynomialcubic.hpp"
#include "polynomialquadratic.hpp"

#define _USE_MATH_DEFINES
#include <math.h>
#include <limits>

#include <iostream>

PolynomialCubic::PolynomialCubic() {
  coefficients[0] = 0.0;
  coefficients[1] = 0.0;
  coefficients[2] = 0.0;
  coefficients[3] = 0.0;
  validroots = 0;
}

PolynomialCubic::PolynomialCubic(double a, double b, double c, double d) {
  coefficients[0] = d;
  coefficients[1] = c;
  coefficients[2] = b;
  coefficients[3] = a;
  validroots = 0;
}

bool PolynomialCubic::computeRoots() {

  if (fabs(coefficients[3]) < std::numeric_limits<double>::epsilon()) {

    /*Quadratic function, fallback to quadratic root solver*/
    PolynomialQuadratic poly(coefficients[2], coefficients[1], coefficients[0]);
    if (!poly.computeRoots()) return false;

    /*Copy roots*/
    validroots = poly.getValidRootsCount();
    for (unsigned int rid = 0; rid < validroots; rid++)
    {
      roots[rid] = poly.getRoot(rid);
    }

    return true;
  }


  double a = coefficients[2] / coefficients[3];
  double b = coefficients[1] / coefficients[3];
  double c = coefficients[0] / coefficients[3];

  double Q = (a*a - 3.0 * b) / 9.0;
  double R = (2.0*a*a*a - 9.0*a*b + 27.0 * c) / 54.0;

  double Q3 = Q*Q*Q;
  double R2 = R*R;

  if (R2 < Q3) {
    double theta = acos(R/sqrt(Q3));
    roots[0].real(-2.0 * sqrt(Q) * cos((theta) / 3.0) - (a / 3.0));
    roots[1].real(-2.0 * sqrt(Q) * cos((theta + 2.0 * M_PI) / 3.0) - (a / 3.0));
    roots[2].real(-2.0 * sqrt(Q) * cos((theta - 2.0 * M_PI) / 3.0) - (a / 3.0));
    roots[0].imag(0);
    roots[1].imag(0);
    roots[2].imag(0);
  }
  else {
    double sign = (R < 0.0)?-1.0:1.0;
    double A = -sign*pow(fabs(R) + sqrt(R2 - Q3), 1.0/3.0);
    double B = 0.0;

    if (fabs(A) > std::numeric_limits<double>::epsilon()) {
      B = Q / A;
    }

    roots[0].real((A+B) - a/3.0);
    roots[0].imag(0);
    roots[1].real(-0.5*(A+B)-a/3.0);
    roots[1].imag((A-B)*sqrt(3.0)/2.0);
    roots[2].real(-0.5*(A+B)-a/3.0);
    roots[2].imag(-(A-B)*sqrt(3.0)/2.0);
    validroots = 3;
  }

  return true;
}

double PolynomialCubic::evaluate(double value) {
  return (((coefficients[3] * value) + coefficients[2]) * value + coefficients[1]) * value + coefficients[0];
}

Polynomial::Root PolynomialCubic::evaluate(Polynomial::Root value) {
  return (((coefficients[3] * value) + coefficients[2]) * value + coefficients[1]) * value + coefficients[0];
}

Polynomial::Root PolynomialCubic::getRoot(unsigned int index) {
  if (index > validroots) {
    return Root();
  }

  return roots[index];
}
