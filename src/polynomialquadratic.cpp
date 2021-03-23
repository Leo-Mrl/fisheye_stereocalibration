#include "polynomialQuadratic.hpp"

#include <math.h>
#include <limits>
#include <iostream>

PolynomialQuadratic::PolynomialQuadratic() {
  coefficients[0] = 0.0;
  coefficients[1] = 0.0;
  coefficients[2] = 1.0;
  validroots = 0;
}

PolynomialQuadratic::PolynomialQuadratic(double a, double b, double c) {
  coefficients[0] = c;
  coefficients[1] = b;
  coefficients[2] = a;
  validroots = 0;
}

bool PolynomialQuadratic::computeRoots() {

  /*Is it a degree 2 ?*/
  if (fabs(coefficients[2]) < std::numeric_limits<double>::epsilon()) {
    /*Is it a degree 1 ?*/
    if (fabs(coefficients[1]) < std::numeric_limits<double>::epsilon()) {
      validroots = 0;
      return true;
    }
    roots[0].real(-coefficients[0] / coefficients[1]);
    validroots = 1;
  }

  /*b^2 - 4ac*/
  double sq = coefficients[1] * coefficients[1] - 4.0 * coefficients[2] * coefficients[0];
  double sign = (coefficients[1] < 0.0) ? -1.0 : 1.0;

  if (sq < 0.0) {
    Root r;
    r.real(0.0);
    r.imag(sign * sqrt(-sq));
    r.real(-0.5 * (coefficients[1] + r.real()));
    r.imag(-0.5 * (coefficients[1] + r.imag()));

    roots[0].real(r.real() / coefficients[2]);
    roots[0].imag(r.imag() / coefficients[2]);

    if (fabs(r.real()) < std::numeric_limits<double>::epsilon() || fabs(r.imag()) < std::numeric_limits<double>::epsilon()) {
      validroots = 1;
      return true;
    }

    roots[1].real(coefficients[0] / r.real());
    roots[1].imag(coefficients[0] / r.imag());
    validroots = 2;

  }
  else {
    double q = -0.5 * (coefficients[1] + sign * sqrt(sq));
    roots[0].real(q / coefficients[2]);
    roots[0].imag(0.0);

    if (fabs(q) < std::numeric_limits<double>::epsilon()) {
      validroots = 1;
      return true;
    }

    roots[1].real(coefficients[0] / q);
    roots[1].imag(0.0);
    validroots = 2;
  }

  return true;
}

double PolynomialQuadratic::evaluate(double value) {
  return (coefficients[2] * value + coefficients[1]) * value + coefficients[0];
}

Polynomial::Root PolynomialQuadratic::evaluate(Root value) {
  return (coefficients[2] * value + coefficients[1]) * value + coefficients[0];
}

Polynomial::Root PolynomialQuadratic::getRoot(unsigned int index) {
  if (index > validroots) {
    return Root();
  }

  return roots[index];
}
