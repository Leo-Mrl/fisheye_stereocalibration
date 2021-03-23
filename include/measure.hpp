#ifndef MEASURE_HPP_
#define MEASURE_HPP_

#include <memory>
#include <array>

#include "config.hpp"

class Point;
class Camera;

namespace ceres {
class CostFunction;
}

/**
A measure is the observation of a point in an image taken by a camera
*/
class VS_EXPORT Measure {
public:

  /**
  Constructor
  @param camera the measuring camera
  @param point the measured point
  @param u the measured coordinate U
  @param v the measured coordinate V
  */
  Measure(std::shared_ptr<Camera> camera, std::shared_ptr<Point> point, double u, double v);

  /**
  Create a cost function functor to use in ceres.
  @return a functor to be used by ceres
  */
  ceres::CostFunction * createIntrinsicsCostFunction();

  /**
  Create a cost function functor to use in ceres.
  @return a functor to be used by ceres
  */
  ceres::CostFunction * createExtrinsicsCostFunction();

  /**
  Get the measured U coordinate
  @return U
  */
  double getU() {
    return coords[0];
  }

  /**
  Get the measured V coordinate
  @return V
  */
  double getV() {
    return coords[1];
  }

  /**
  Set the measured U coordinate
  @param u the U coordinate
  */
  void setU(double u) {
    coords[0] = u;
  }

  /**
  Set the measured V coordinate
  @param u the V coordinate
  */
  void setV(double v) {
    coords[1] = v;
  }

  /**
  Get the coordinates as an array
  @return the measure coordinates
  */
  std::array<double, 2> getCoords();

  /**
  Retrieve a link to the measured point
  @return measured point link
  */
  std::shared_ptr<Point> getPoint();

  /**
  Retrieve a link to the measured point
  @return measuring camera link
  */
  std::shared_ptr<Camera> getCamera();

  /**
  Update the reference point
  @param point new point to use
  */
  void setPoint(std::shared_ptr<Point> point);

private:

  /**
  Link to a camera.
  Weak_ptr is used to avoid cyclic dependencies
  */
  std::weak_ptr<Camera> camera;

  /**
  Link to the measured point
  */
  std::shared_ptr<Point> point;

  /**
  Measured coordinates in pixels
  */
  std::array<double, 2> coords;
};

#endif
