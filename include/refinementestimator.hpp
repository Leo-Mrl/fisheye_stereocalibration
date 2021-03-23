#ifndef __REFINEMENT_ESTIMATOR_HPP__
#define __REFINEMENT_ESTIMATOR_HPP__

#include <vector>
#include <memory>
#include "config.hpp"

class PointGrid;
class Camera;

/**
Defines algorithm to refine the calibration parameters
*/
class VS_EXPORT RefinementEstimator {
public:
  RefinementEstimator();
  ~RefinementEstimator();

  /**
  @param grid the measured point grid definition
  @param views the set of measuring views to refine
  @return true if the refinment succeeded
  */
  bool estimate(PointGrid & grid, std::vector<std::shared_ptr<Camera> > & views);

  /**
  @param grid the measured point grid definition
  @param views the set of measuring views to refine
  @param disableTranslations boolean to disable translations in the extrinsics
  @return true if the refinment succeeded
  */
  bool estimateExtrinsics(PointGrid & grid, std::vector<std::shared_ptr<Camera> > & views, bool disableTranslations);
};

#endif
