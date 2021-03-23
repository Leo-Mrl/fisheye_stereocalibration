#ifndef __DEMOEPIPOLAR_HPP__
#define __DEMOPIPOLAR_HPP__

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"

#include "camera.hpp"
#include "checkerBoard.hpp"
#include "lensFisheye.hpp"
#include "pointGrid.hpp"
#include "extrinsicEstimator.hpp"
#include "refinementEstimator.hpp"
#include "calibration.hpp"
#include "rectification.hpp"
#include "tools.hpp"
#include "point.hpp"
#include "epipolar.hpp"

void demonstrationEpipolarCurve(const std::vector<std::string> &imagesL, 
	const std::vector<std::string> &imagesR, const std::shared_ptr<LensFishEye> &myLensL, 
	const std::shared_ptr<LensFishEye> &myLensR, cv::Mat F, int imgSel);

#endif