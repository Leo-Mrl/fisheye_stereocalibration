#ifndef __EXTRINSICESTIMATOR_HPP__
#define __EXTRINSICESTIMATOR_HPP__

#include <memory>
#include "config.hpp"

class Camera;
class PointGrid;

/**
Class to estimate coarse extrinsics from scratch for a given camera.
Assume the camera measured something
*/
class VS_EXPORT ExtrinsicEstimator {
public:
  ExtrinsicEstimator();
  virtual ~ExtrinsicEstimator();

  /**
  Estimate the extrinsics on a camera.
  The extrinsics are the rotation and translation
  @param grid the grid description
  @param camera the camera to estimate extrinsics for
  @return true if estimation is successful
  */
  bool estimate(const PointGrid & grid, std::shared_ptr<Camera> & camera);
};

#endif
