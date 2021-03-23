#ifndef __POLYNOMIAL_CUBIC_HPP__
#define __POLYNOMIAL_UBIC_HPP__

#include "polynomial.hpp"

class PolynomialCubic : public Polynomial {
public:
  PolynomialCubic();
  PolynomialCubic(double a, double b, double c, double d);
  virtual bool computeRoots();
  virtual double evaluate(double value);
  virtual Root evaluate(Root value);
  virtual Root getRoot(unsigned int index);

protected:
  /*Ordered by increasing degree*/
  double coefficients[4];
  /*Result roots*/
  Root roots[3];
};

#endif
