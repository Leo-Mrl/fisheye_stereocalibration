#ifndef LENS_FISH_EYE_HPP_
#define LENS_FISH_EYE_HPP_

#include <Eigen/Dense>
#include <array>

#include "config.hpp"
#include "measure.hpp"
#include "lens.hpp"

class VS_EXPORT LensFishEye : public Lens {
public:

  /**
  Constructor
  @param image_width the measured image width
  @param image_height the measured image height
  @param focal the focals in pixel in both image dimensions (horizontal & vertical)
  @param principal_point principal point in both image dimensions (horizontal & vertical)
  @param distorsion the distorsion parameters to use
  */
  LensFishEye(int image_width, int image_height, const std::array<double, 2> & focal, const std::array<double, 2> & principal_point, const std::array<double, 3> & distorsion);

  /**
  Get the first distortion parameter (the 3th degree factor)
  @return the A factor
  */
  double getA();

  /**
  Get the first distortion parameter (the 2nd degree factor)
  @return the B factor
  */
  double getB();

  /**
  Get the first distortion parameter (the 1st degree factor)
  @return the C factor
  */
  double getC();

  /**
  Given a measure in pixels, compute the spherical coordinates.
  @param result the computed spherical coordinates
  @param mes the input measure
  @return true if lift succeeded
  */
  virtual bool liftToUnitSphere(std::array<double, 3> & result, const std::shared_ptr<Measure> & mes);

  /**
  Given a measure in pixels, compute the spherical coordinates.
  @param result the computed spherical coordinates
  @param u the measured u coordinates
  @param u the measured v coordinates
  @return true if lift succeeded
  */
  virtual bool liftToUnitSphere(std::array<double, 3> & result, double u, double v);

  /**
  Given a measure in pixels and assuming vertical optical center alignment, compute the spherical coordinates.
  @param result the computed spherical coordinates
  @param u the measured u coordinates
  @return true if lift succeeded
  */
  virtual bool liftToUnitSphere(std::array<double, 3> & result, double u);

  /**
  Estimate the projection coordinates (in pixels or in meters) for a given input point
  @param point the input point to project
  @param inpixels decide if the result is in pixels (true) or in meters
  return the image coordinates
  */
  virtual std::array<double, 2> project(const Point & point, bool inpixels = true);

  /**
  Retrieve the lens additional parameters
  @return a pointer to an array (3 distortion parameters)
  */
  virtual double * getAdditionalParametersPtr();

  /**
  Set the lens additional parameters (defined by the children)
  @param vals the input array (3 distortion parameters)
  */
  virtual void setAdditionalParameters(const double * vals);

  /**
  Compute jacobian of a measure estimation wrt the lens adv parameters
  @param J the result jacobian
  @param cpoint the point in the camera frame
  */
  virtual bool jacobianWrtAdditional(Eigen::Matrix<double, 2, -1> & J, const Point & cpoint);

  /**
  Compute jacobian of a measure estimation wrt the point in camera frame parameters
  @param J the result jacobian
  @param cpoint the point in the camera frame
  */
  virtual bool jacobianWrtPoint(Eigen::Matrix<double, 2, 3> & J, const Point & cpoint);

  /**
  Create a costfunction functor to be used by ceres solvers
  @param mes the associated measure
  @return the solver costfunction
  */
  virtual ceres::CostFunction * createIntrinsicsCostFunction(Measure * mes);

  /**
  Create a costfunction functor to be used by ceres solvers
  @param mes the associated measure
  @return the solver costfunction
  */
  virtual ceres::CostFunction * createExtrinsicsCostFunction(Measure * mes);

  /**
  Display a report on the lens
  */
  virtual void display();

private:
  std::array<double, 3> additional;
};




#endif
