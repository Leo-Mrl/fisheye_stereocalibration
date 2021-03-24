#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"

#include <iostream>
#include <memory>
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
#include "point.hpp"
#include "rectification.hpp"

cv::Mat computeF(const std::vector<std::string> &imagesL, 
    const std::vector<std::string> &imagesR, const std::shared_ptr<LensFishEye> &myLensL, 
    const std::shared_ptr<LensFishEye> &myLensR, cv::Mat rbmMean, int w, int h) {
    cv::Size boardSize(w, h);

    std::vector< std::vector<cv::Point2f> > cornersL;
    std::vector< std::vector<cv::Point2f> > cornersR;

    cv::Size winSize( 5, 5 );
    cv::Size zeroZone( -1, -1 );
    cv::TermCriteria criteria( cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40, 0.001 );

    std::vector<cv::Point3d> pointsL, pointsR;

    for(int i=0; i<fabs(imagesL.size()); i++){

        cv::Mat imgL = cv::imread(imagesL[i]);
        cv::Mat imgR = cv::imread(imagesR[i]);
        std::vector<cv::Point2f> cornersTempL(w*h);
        std::vector<cv::Point2f> cornersTempR(w*h);

        cv::findChessboardCorners(imgL, boardSize, cornersTempL);
        cv::findChessboardCorners(imgR, boardSize, cornersTempR);

        cv::cvtColor(imgL, imgL, cv::COLOR_BGR2GRAY);
        cv::cvtColor(imgR, imgR, cv::COLOR_BGR2GRAY);

        cv::cornerSubPix(imgL, cornersTempL, winSize, zeroZone, criteria);
        cv::cornerSubPix(imgR, cornersTempR, winSize, zeroZone, criteria);

        cornersL.push_back(cornersTempL);
        cornersR.push_back(cornersTempR);

        for(int k=0; k<w*h; k++){

            std::array<double, 3> resultL;
            cv::Point2f tmp2DL = cornersTempL[k];
            double uL = (double) tmp2DL.x;
            double vL = (double) tmp2DL.y;
            myLensL->liftToUnitSphere(resultL, uL, vL);
            cv::Point3d tmp3DL(resultL[0], resultL[1], resultL[2]);
            pointsL.push_back(tmp3DL);

            std::array<double, 3> resultR;
            cv::Point2f tmp2DR = cornersTempR[k];
            double uR = (double) tmp2DR.x;
            double vR = (double) tmp2DR.y;
            myLensR->liftToUnitSphere(resultR, uR, vR);
            cv::Point3d tmp3DR(resultR[0], resultR[1], resultR[2]);
            pointsR.push_back(tmp3DR);
        }
    }

    //Filling A
    cv::Mat A = cv::Mat::zeros(pointsL.size(), 9, CV_64F);
    for(int i=0; i<fabs(pointsL.size()); i++){

        cv::Point3d Q = pointsL[i];
        cv::Point3d Qp = pointsR[i];

        A.at<double>(i, 0) = Qp.x * Q.x; //x'x
        A.at<double>(i, 1) = Qp.x * Q.y; //x'y
        A.at<double>(i, 2) = Qp.x * Q.z; //x'z

        A.at<double>(i, 3) = Qp.y * Q.x; //y'x
        A.at<double>(i, 4) = Qp.y * Q.y; //y'y
        A.at<double>(i, 5) = Qp.y * Q.z; //y'z

        A.at<double>(i, 6) = Qp.z * Q.x; //z'x
        A.at<double>(i, 7) = Qp.z * Q.y; //z'y
        A.at<double>(i, 8) = Qp.z * Q.z; //z'z
    }

    cv::Mat wt, u, vt;
    cv::SVD::compute(A, wt, u, vt);

    vt = vt.t();
    cv::Mat fs = vt.col(8);

    cv::Mat check = A*fs;
    check = (check.t())*check;

    cv::Mat Fs = cv::Mat::zeros(3, 3, CV_64F);
    Fs.at<double>(0,0) = fs.at<double>(0,0);
    Fs.at<double>(0,1) = fs.at<double>(1,0);
    Fs.at<double>(0,2) = fs.at<double>(2,0);

    Fs.at<double>(1,0) = fs.at<double>(3,0);
    Fs.at<double>(1,1) = fs.at<double>(4,0);
    Fs.at<double>(1,2) = fs.at<double>(5,0);

    Fs.at<double>(2,0) = fs.at<double>(6,0);
    Fs.at<double>(2,1) = fs.at<double>(7,0);
    Fs.at<double>(2,2) = fs.at<double>(8,0);

    cv::SVD::compute(Fs, wt, u, vt);

    wt.at<double>(1,0) = wt.at<double>(0,0);
    wt.at<double>(2,0) = 0;

    cv::Mat FsRecomp = u*cv::Mat::diag(wt)*vt;

    double MSE = 0;

    for(int i=0; i<fabs(pointsL.size()); i++){

        cv::Point3d Q = pointsL[i];
        cv::Point3d Qp = pointsR[i];

        cv::Mat mQp = cv::Mat(1, 3, CV_64F);
        cv::Mat mQ =  cv::Mat(3, 1, CV_64F);

        mQp.at<double>(0,0) = Qp.x;
        mQp.at<double>(0,1) = Qp.y;
        mQp.at<double>(0,2) = Qp.z;

        mQ.at<double>(0,0) = Q.x;
        mQ.at<double>(1,0) = Q.y;
        mQ.at<double>(2,0) = Q.z;

        cv::Mat out = FsRecomp*mQ;
        out = mQp*out;

        double tmp = out.at<double>(0,0);
        MSE = MSE + tmp*tmp;
    }

    MSE = MSE/pointsL.size();

    cv::Mat testE = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat testR = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat testTx = cv::Mat::zeros(3, 3, CV_64F);
    testR = rbmMean(cv::Rect(0, 0, 3, 3));

    testTx.at<double>(0,1) = -rbmMean.at<double>(2,3); //-z
    testTx.at<double>(0,2) = rbmMean.at<double>(1,3); //y
    testTx.at<double>(1,0) = rbmMean.at<double>(2,3); //z
    testTx.at<double>(1,2) = -rbmMean.at<double>(0,3); //-x
    testTx.at<double>(2,0) = -rbmMean.at<double>(1,3); //-y
    testTx.at<double>(2,1) = rbmMean.at<double>(0,3); //x

    testE = testTx * testR;
    FsRecomp = testE;

    printf("\nF computation MSE =  : %.10f \n", MSE);
    std::cout << "F : " << FsRecomp;
    return FsRecomp;
}

std::vector<cv::Point2f> findEpipoles(const cv::Mat &F, const std::shared_ptr<LensFishEye> &myLensL, 
    const std::shared_ptr<LensFishEye> &myLensR) {

    // x' * f * x = 0
    // [e' * f = 0]  &  [f * e = 0]
    // [e'1 = -e'2]  &  [e1 = -e2]
    // output = [e'1, e1, e'2, e2];

    std::vector<cv::Point2f> Ep;
    cv::Mat w, u, vt;
    cv::SVD::compute(F, w, u, vt);
    vt = vt.t();

    cv::Mat e = vt.col(2);
    cv::Mat ep = u.col(2);

    Point ePt(e.at<double>(0,0), e.at<double>(1,0), e.at<double>(2,0));
    Point epPt(ep.at<double>(0,0), ep.at<double>(1,0), ep.at<double>(2,0));

    std::array<double, 2> eIm = myLensL->project(ePt, 1);
    std::array<double, 2> epIm = myLensR->project(epPt, 1);
    cv::Point2f eCV; eCV.x = eIm[0]; eCV.y = eIm[1];
    cv::Point2f epCV; epCV.x = epIm[0]; epCV.y = epIm[1];
    Ep.push_back(epCV);
    Ep.push_back(eCV);

    ePt.setX(-ePt.getX()); ePt.setY(-ePt.getY()); ePt.setZ(-ePt.getZ());
    epPt.setX(-epPt.getX()); epPt.setY(-epPt.getY()); epPt.setZ(-epPt.getZ());
    eIm = myLensL->project(ePt, 1);
    epIm = myLensR->project(epPt, 1);
    eCV.x = eIm[0]; eCV.y = eIm[1];
    epCV.x = epIm[0]; epCV.y = epIm[1];
    Ep.push_back(epCV);
    Ep.push_back(eCV);

    return Ep;

}
