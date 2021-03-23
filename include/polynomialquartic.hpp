#ifndef __POLYNOMIAL_QUARTIC_HPP__
#define __POLYNOMIAL_QUARTIC_HPP__

#include "polynomial.hpp"

class PolynomialQuartic : public Polynomial {
public:
  PolynomialQuartic();
  PolynomialQuartic(double a, double b, double c, double d, double e);
  virtual bool computeRoots();
  virtual double evaluate(double value);
  virtual Root evaluate(Root value);
  virtual Root getRoot(unsigned int index);

protected:
  /*Ordered by increasing degree*/
  double coefficients[5];
  /*Result roots*/
  Root roots[4];
};

#endif
