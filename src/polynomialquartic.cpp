#include "polynomialQuartic.hpp"
#include "polynomialCubic.hpp"

#include <math.h>
#include <limits>
#include <Eigen/Dense>
#include <unsupported/Eigen/Polynomials>
#include <iostream>

PolynomialQuartic::PolynomialQuartic() {
  coefficients[0] = 0.0;
  coefficients[1] = 0.0;
  coefficients[2] = 0.0;
  coefficients[3] = 0.0;
  coefficients[4] = 1.0;
  validroots = 0;
}

PolynomialQuartic::PolynomialQuartic(double a, double b, double c, double d, double e) {
  coefficients[0] = e;
  coefficients[1] = d;
  coefficients[2] = c;
  coefficients[3] = b;
  coefficients[4] = a;
  validroots = 0;
}

bool PolynomialQuartic::computeRoots() {

  if (fabs(coefficients[4]) < std::numeric_limits<double>::epsilon()) {

    /*Cubic function, fallback to quadratic root solver*/
    PolynomialCubic poly(coefficients[3], coefficients[2], coefficients[1], coefficients[2]);
    if (!poly.computeRoots()) return false;

    /*Copy roots*/
    validroots = poly.getValidRootsCount();
    for (unsigned int rid = 0; rid < validroots; rid++)
    {
      roots[rid] = poly.getRoot(rid);
    }

    return true;
  }


  Eigen::Matrix<double, 5, 1> list;
  list(0,0) = coefficients[0];
  list(1,0) = coefficients[1];
  list(2,0) = coefficients[2];
  list(3,0) = coefficients[3];
  list(4,0) = coefficients[4];

  Eigen::PolynomialSolver<double, 4> solver;
  solver.compute<Eigen::Matrix<double, 5, 1> >(list);
  validroots = 0;
  for (int i = 0; i < solver.roots().size(); i++) {
    roots[i] = solver.roots()[i];
    validroots++;
  }

  return true;
}

double PolynomialQuartic::evaluate(double value) {
  return ((((coefficients[4] * value + coefficients[3]) * value) + coefficients[2]) * value + coefficients[1]) * value + coefficients[0];
}

Polynomial::Root PolynomialQuartic::evaluate(Root value) {
  return ((((coefficients[4] * value + coefficients[3]) * value) + coefficients[2]) * value + coefficients[1]) * value + coefficients[0];
}

Polynomial::Root PolynomialQuartic::getRoot(unsigned int index) {
  if (index > validroots) {
    return Root();
  }

  return roots[index];
}
