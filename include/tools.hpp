#ifndef TOOLS_HPP_
#define TOOLS_HPP_

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"

#include <iostream>
#include <memory>
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <unistd.h>
#include <string>
#include <math.h>
#include <iomanip>

#include "camera.hpp"
#include "checkerboard.hpp"
#include "lensFisheye.hpp"
#include "pointGrid.hpp"
#include "fisheyeIntrinsicEstimator.hpp"
#include "extrinsicEstimator.hpp"
#include "refinementEstimator.hpp"
#include "rectification.hpp"

cv::Mat crossProductOpenCV(cv::Mat A, cv::Mat B);

double eucliNormOpenCV(cv::Mat A);

void loadImages(const std::string &imgPath, std::vector<cv::String> *imagesL, 
	std::vector<cv::String> *imagesR, int *w, int *h ,float *sz);

void drawEpipoles(std::vector<cv::Point2f> Ep, const std::vector<std::string> &imagesL, 
	const std::vector<std::string> &imagesR, cv::Mat *imOut, int imNumber);

void drawEpipolarCurve(cv::Point2f inputPt, const std::shared_ptr<LensFishEye> &myLensL, 
	const std::shared_ptr<LensFishEye> &myLensR, cv::Mat F, cv::Mat *imOut, cv::Scalar Color);

void writeMatToFile(const cv::Mat &m, const std::string filename, std::string name);

#endif