#ifndef __RECTIFICATION_HPP__
#define __RECTIFICATION_HPP__

cv::Mat rectification (cv::Mat imgIn, std::shared_ptr<LensFishEye> myLens, cv::Mat *mapx, cv::Mat *mapy);

void computeHomography(cv::Size sz, cv::Mat rtFinal, cv::Mat *mapx, cv::Mat *mapy);

cv::Mat EPalignement(cv::Mat leftMean, cv::Mat rightMean, cv::Mat KrectL, cv::Mat KrectR);

cv::Mat testRectif(cv::Mat imgIn, std::shared_ptr<LensFishEye> myLens);

#endif