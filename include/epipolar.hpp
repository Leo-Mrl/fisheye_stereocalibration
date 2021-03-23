#ifndef __EPIPOLAR_HPP__
#define __EPIPOLAR_HPP__

cv::Mat computeF(const std::vector<std::string> &imagesL, 
	const std::vector<std::string> &imagesR, const std::shared_ptr<LensFishEye> &myLensL, 
	const std::shared_ptr<LensFishEye> &myLensR, cv::Mat rbmMean, int w, int h);

std::vector<cv::Point2f> findEpipoles(const cv::Mat &F, const std::shared_ptr<LensFishEye> &myLensL, 
	const std::shared_ptr<LensFishEye> &myLensR);

#endif