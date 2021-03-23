#ifndef __POLYNOMIAL_HPP__
#define __POLYNOMIAL_HPP__

#include <complex>
#include "config.hpp"

class Polynomial {
public:
  typedef std::complex<double> Root;

  virtual bool computeRoots() = 0;
  virtual double evaluate(double value) = 0;
  virtual Root evaluate(Root value) = 0;
  virtual Root getRoot(unsigned int index) = 0;

  unsigned int getValidRootsCount() {
    return validroots;
  }

protected:
  /*Count valid roots*/
  unsigned int validroots;
};

#endif
