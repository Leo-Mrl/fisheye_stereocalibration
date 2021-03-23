#ifndef __FISHEYEINTRISICESTIMATOR_HPP__
#define __FISHEYEINTRISICESTIMATOR_HPP__

#include "camera.hpp"
#include "config.hpp"
#include "measure.hpp"
#include "lens.hpp"
#include "measuresGrid.hpp"
#include "pointGrid.hpp"

#include <Eigen/Core>
#include <Eigen/SVD>

#include <vector>
#include <memory>
#include <iostream>

class Camera;
class PointGrid;

/**
Class to estimate coarse intrinsics for lens.
*/
class VS_EXPORT fisheyeIntrinsicEstimator {
public:
  fisheyeIntrinsicEstimator();
  virtual ~fisheyeIntrinsicEstimator();

  /**
  Estimate all lens intrinsics associated to the given views
  Assume all views share the same lens
  @param grid the associated point grid config used
  @param views a set of cameras which use the lens
  @return true if estimation succeeded.
  */
  bool estimate(const PointGrid & grid, std::vector<std::shared_ptr<Camera> > & views);

private:

  /**
  Estimate gamma parameter for a set of three 2D measures
  @param gamma estimated gamma
  @param nx estimated normal x component
  @param ny estimated normal y component
  @param nz estimated normal z component
  @param u1 first measure u component
  @param v1 first measure v component
  @param u2 second measure u component
  @param v2 second measure v component
  @param u3 third measure u component
  @param v3 third measure v component
  */
  bool estimateGamma(double & gamma, double & nx, double & ny, double & nz, double u1, double v1, double u2, double v2, double u3, double v3);

  /**
  Estimate gamma on dataset
  @param grid the associated point grid config used
  @param gamma value to score
  @return a score on dataset
  */
  double evaluateGamma(const PointGrid & grid, std::vector<std::shared_ptr<Camera> > & view, double gamma);
};

#endif
