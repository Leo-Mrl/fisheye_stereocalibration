#ifndef __CALIBRATION_HPP__
#define __CALIBRATION_HPP__

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"

#include "frame.hpp"
#include "checkerboard.hpp"
#include "lensFisheye.hpp"
#include "extrinsicEstimator.hpp"
#include "refinementEstimator.hpp"
#include "rectification.hpp"
#include "fisheyeIntrinsicEstimator.hpp"

std::shared_ptr<LensFishEye> calibration(const int gridwidth, const int gridheight, 
	const double squaresize, const std::vector<std::string> &images, 
	std::vector<cv::Mat> *rotationsVec, std::vector<cv::Mat> *translationsVec, 
	cv::Mat *Kam, cv::Mat *Dist) ;

std::vector<cv::Point3f> Create3DChessboardCorners(const cv::Size boardSize, const float squareSize);

cv::Mat cameraMatrixforEG(std::vector<std::string> images, std::shared_ptr<LensFishEye> myLens, 
	cv::Size boardSize, float squareSize);

void extrinsicEstimation(const std::vector<cv::Mat> &RotationsL, const std::vector<cv::Mat> &RotationsR, 
	const std::vector<cv::Mat> &TranslationsL, const std::vector<cv::Mat> &TranslationsR, 
	cv::Mat *rbmLeft, cv::Mat *rbmRight, cv::Mat *rbmMean, int num);

#endif