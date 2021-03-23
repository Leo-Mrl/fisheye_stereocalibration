#include "lens.hpp"

#define _USE_MATH_DEFINES
#include <math.h>

Lens::Lens(int image_width, int image_height, const std::array<double, 2> & focal, const std::array<double, 2> & principal_point) {
  this->generic_parameters[0] = principal_point[0];
  this->generic_parameters[1] = principal_point[1];
  this->generic_parameters[2] = focal[0];
  this->generic_parameters[3] = focal[1];
  this->image_height = image_height;
  this->image_width = image_width;
}

double * Lens::getGenericParametersPtr() {
  return this->generic_parameters.data();
}

void Lens::setGenericParameters(const double * vals) {
  generic_parameters[0] = vals[0];
  generic_parameters[1] = vals[1];
  generic_parameters[2] = vals[2];
  generic_parameters[3] = vals[3];
}

double Lens::getHorizontalFov() {
 std::array<double, 3> pleft, pright;
 bool res;

 res = liftToUnitSphere(pleft, 0);
 if (!res) return 0.0;

 res = liftToUnitSphere(pright, image_width);
 if (!res) return 0.0;

 double fov_left = atan2(-pleft[0], pleft[2]) * 2.0;
 double fov_right = atan2(pright[0], pright[2]) * 2.0;

 double fov = std::max(fov_left, fov_right);
 return fov * 180.0 / M_PI;
}

bool Lens::jacobianWrtGeneric(Eigen::Matrix<double, 2, 4> & J, const Point & cpoint) {

  std::array<double, 2> mes = this->project(cpoint, false);

  J(0,0) = 1;
  J(0,1) = 0;
  J(0,2) = mes[0];
  J(0,3) = 0;

  J(1,0) = 0;
  J(1,1) = 1;
  J(1,2) = 0;
  J(1,3) = mes[1];

  return true;
}
