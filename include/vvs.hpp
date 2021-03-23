#ifndef __VVS__
#define __VVS__

#include <Eigen/Dense>
#include <vector>
#include <memory>

#include "measure.hpp"

bool estimateVVS(Eigen::Matrix4d & pose, const std::vector<std::shared_ptr<Measure> > & measures);

#endif
