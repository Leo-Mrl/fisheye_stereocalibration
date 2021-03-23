#ifndef __LENS_HPP__
#define __LENS_HPP__


#include <array>
#include <memory>
#include <math.h>
#include <Eigen/Dense>

#include "config.hpp"
#include <iostream>
class Measure;
class Point;


namespace ceres {
class CostFunction;
}


/**
Define a Lens.
A lens defines the intrinsics parameters of a camera.
It is shared by many "views" or "Camera".
*/
class VS_EXPORT Lens {
public:

  /**
  Constructor
  @param image_width the measured image width
  @param image_height the measured image height
  @param focal the focals in pixel in both image dimensions (horizontal & vertical)
  @param principal_point principal point in both image dimensions (horizontal & vertical)
  */
  Lens(int image_width, int image_height, const std::array<double, 2> & focal, const std::array<double, 2> & principal_point);

public:

  /**
  Retrieve the image width
  @return the image width
  */
  unsigned int getImageWidth() {
    return image_width;
  }

  /**
  Retrieve the image height
  @return the image height
  */
  unsigned int getImageHeight() {
    return image_height;
  }

  /**
  Set the image width
  @param width the new image width
  */
  void setImageWidth(unsigned int width) {
    image_width = width;
  }

  /**
  Set the image height
  @param height the new image height
  */
  void setImageHeight(unsigned int height) {
    image_height = height;
  }

  /**
  Set the principal point
  @param cu the u center point parameter
  @param cv the v center point parameter
  */
  void setPrincipalPoint(double u, double v) {
    generic_parameters[0] = u;
    generic_parameters[1] = v;
  }

  /**
  Retrieve the principal point u component
  @return the principal point U
  */
  double getPrincipalU() {
    return generic_parameters[0];
  }

  /**
  Retrieve the principal point v component
  @return the principal point V
  */
  double getPrincipalV() {
    return generic_parameters[1];
  }

  /**
  Set the focal of the lens
  @param fu the focal in pixels for the horizontal axis
  @param fv the focal in pixels for the vertical axis
  */
  void setFocal(double fu, double fv) {
    generic_parameters[2] = fu;
    generic_parameters[3] = fv;
  }

  /**
  Retrieve the focal in pixels for the horizontal axis
  @return the horizontal focal
  */
  double getFocalU() {
    return generic_parameters[2];
  }

  /**
  Retrieve the focal in pixels for the vertical axis
  @return the vertical focal
  */
  double getFocalV() {
    return generic_parameters[3];
  }

  /**
  Estimate the fov from computed parameters
  @return the horizontal fov in degrees
  */
  double getHorizontalFov();

  /**
  Retrieve the lens parameters pointer
  @return a pointer to the array of parameters (cu,cv,fu,fv)
  */
  double * getGenericParametersPtr();

  /**
  Set the lens parameters using an array
  @param a 4 double array with (cu,cv,fu,fv);
  */
  void setGenericParameters(const double * vals);

  /**
  Retrieve the lens additional parameters (defined by the children)
  @return a pointer to an array
  */
  virtual double * getAdditionalParametersPtr() = 0;

  /**
  Set the lens additional parameters (defined by the children)
  @param vals the input array
  */
  virtual void setAdditionalParameters(const double * vals) = 0;

  /**
  Given a measure in pixels, compute the spherical coordinates (unit sphere).
  @param result the computed spherical coordinates
  @param mes the input measure
  @return true if lift succeeded
  */
  virtual bool liftToUnitSphere(std::array<double, 3> & result, const std::shared_ptr<Measure> & mes) = 0;

  /**
  Given a measure in pixels, compute the spherical coordinates.
  @param result the computed spherical coordinates
  @param u the measured u coordinates
  @param u the measured v coordinates
  @return true if lift succeeded
  */
  virtual bool liftToUnitSphere(std::array<double, 3> & result, double u, double v) = 0;

  /**
  Given a measure in pixels and assuming vertical optical center alignment, compute the spherical coordinates.
  @param result the computed spherical coordinates
  @param u the measured u coordinates
  @return true if lift succeeded
  */
  virtual bool liftToUnitSphere(std::array<double, 3> & result, double u) = 0;

  /**
  Estimate the projection coordinates (in pixels or in meters) for a given input point
  @param point the input point to project
  @param inpixels decide if the result is in pixels (true) or in meters
  return the image coordinates
  */
  virtual std::array<double, 2> project(const Point & point, bool inpixels = true) = 0;

  /**
  Create a costfunction functor to be used by ceres solvers
  @param mes the associated measure
  @return the solver costfunction
  */
  virtual ceres::CostFunction * createIntrinsicsCostFunction(Measure * mes) = 0;

  /**
  Create a costfunction functor to be used by ceres solvers
  @param mes the associated measure
  @return the solver costfunction
  */
  virtual ceres::CostFunction * createExtrinsicsCostFunction(Measure * mes) = 0;

  /**
  Compute jacobian of a measure estimation wrt the lens basic parameters
  @param J the result jacobian
  @param cpoint the point in the camera frame
  */
  virtual bool jacobianWrtGeneric(Eigen::Matrix<double, 2, 4> & J, const Point & cpoint);

  /**
  Compute jacobian of a measure estimation wrt the lens adv parameters
  @param J the result jacobian
  @param cpoint the point in the camera frame
  */
  virtual bool jacobianWrtAdditional(Eigen::Matrix<double, 2, -1> & J, const Point & cpoint) = 0;

  /**
  Compute jacobian of a measure estimation wrt the point in camera frame parameters
  @param J the result jacobian
  @param cpoint the point in the camera frame
  */
  virtual bool jacobianWrtPoint(Eigen::Matrix<double, 2, 3> & J, const Point & cpoint) = 0;

  /**
  Display a report on the lens
  */
  virtual void display() = 0;

protected:

  unsigned int image_width;
  unsigned int image_height;

  std::array<double, 4> generic_parameters;
};

#endif
