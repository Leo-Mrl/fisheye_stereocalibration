#ifndef  __CHECKERBOARD_HPP__
#define  __CHECKERBOARD_HPP__

#include <vector>
#include <memory.h>

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

#include "config.hpp"
#include "measure.hpp"
#include "pointGrid.hpp"
#include "lens.hpp"
#include "point.hpp"
#include "camera.hpp"

namespace cv {
  class Mat;
}

/**
Checker board corners extraction
*/
class VS_EXPORT CheckerBoard
{
public:
  CheckerBoard();
  virtual ~CheckerBoard();

  /**
  Extract corners on an image and create objects.
  @param source the input image. This image must contains a checkerboard
  @param lens the lens object which describes the image camera lens.
  @param grid the point grid which describes the 3D structure of the checkerboard
  @return true if checkerboard detection worked
  */
  bool extract(const cv::Mat & source, const std::shared_ptr<Lens> & lens, const PointGrid & grid);

  /**
  Load corners from a file and create objects.
  @param source the input file (lines of 2 reals).
  @param lens the lens object which describes the image camera lens.
  @param grid the point grid which describes the 3D structure of the checkerboard
  @return true if checkerboard detection worked
  */
  bool load(const std::string & source, const std::shared_ptr<Lens> & lens, const PointGrid & grid);

  /**
  Compute the mask of a given image of a checkerboard to detect identifiers
  @param source the image with the checkerboard observation
  @param grid the point grid which describes the 3D structure of the checkerboard
  @return a mask for each checkerboard cells
  */
  Eigen::Matrix<bool, -1, -1> computeMask(const cv::Mat & source, const PointGrid & grid);

  /**
  Get created camera link
  @return a camera pointer
  */
  std::shared_ptr<Camera> getCamera();

private:
  std::shared_ptr<Camera> camera;
};

#endif
