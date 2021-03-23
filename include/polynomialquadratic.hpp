#ifndef __POLYNOMIAL_QUADRATIC_HPP__
#define __POLYNOMIAL_QUADRATIC_HPP__

#include "polynomial.hpp"

class PolynomialQuadratic : public Polynomial {
public:
  PolynomialQuadratic();
  PolynomialQuadratic(double a, double b, double c);
  virtual bool computeRoots();
  virtual double evaluate(double value);
  virtual Root evaluate(Root value);
  virtual Root getRoot(unsigned int index);

protected:
  /*Ordered by increasing degree*/
  double coefficients[3];
  /*Result roots*/
  Root roots[2];
};

#endif
