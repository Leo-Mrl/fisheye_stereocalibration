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
#include "checkerBoard.hpp"
#include "lensFisheye.hpp"
#include "pointGrid.hpp"
#include "fisheyeIntrinsicEstimator.hpp"
#include "extrinsicEstimator.hpp"
#include "refinementEstimator.hpp"
#include "point.hpp"
#include "tools.hpp"

void computeHomography(cv::Size sz, cv::Mat rtFinal, cv::Mat *mapx, cv::Mat *mapy){

    cv::Mat coord = cv::Mat::zeros(3,1,CV_64F);

    for(int i=0; i<sz.height; i++){
        for(int j=0; j<sz.width; j++){
            coord.at<double>(0,0) = j;
            coord.at<double>(1,0) = i;
            coord.at<double>(2,0) = 1;
            coord = rtFinal*coord;
            mapx->at<float>(i,j) = (float) coord.at<double>(0,0);
            mapy->at<float>(i,j) = (float) coord.at<double>(1,0);
        }
    }
}

cv::Mat rectification (cv::Mat imgIn, std::shared_ptr<LensFishEye> myLens, cv::Mat *mapx, cv::Mat *mapy){

    cv::Size imSize = imgIn.size();

    Point my3Dpoint(0,0,0);
    std::array<double, 2> my2Dpoint;

    float sf = 6; //Camera-plane distance

    float Nxc = myLens->getPrincipalU();
    float Nyc = myLens->getPrincipalV();
    float Nz = imgIn.cols/sf;

    //std::cout << "Nz = " << Nz << std::endl;

    for(int j=0; j<imgIn.rows; j++){
        for(int i=0; i<imgIn.cols; i++){
            my3Dpoint.setX(i-Nxc);
            my3Dpoint.setY(j-Nyc);
            my3Dpoint.setZ(Nz);
            my2Dpoint = myLens->project(my3Dpoint, 1);
            mapx->at<float>(j,i) = (float) my2Dpoint[0];
            mapy->at<float>(j,i) = (float) my2Dpoint[1];
        }
    }

    cv::Mat imgOut = cv::Mat(imSize, CV_32FC1);
    cv::remap(imgIn, imgOut, *mapx, *mapy, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT, cv::Scalar(0,0,0));

    return imgOut;
}

cv::Mat EPalignement(cv::Mat leftMean, cv::Mat rightMean, cv::Mat KrectL, cv::Mat KrectR){

    std::cout << "********************** Epipolar alignment **********************" << std::endl; std::cout << std::endl;

    //Compute optical centers
    cv::Mat RT_left = leftMean(cv::Rect(0, 0, 4, 3)); //[R|t]
    cv::Mat RT_right = rightMean(cv::Rect(0, 0, 4, 3));
    cv::Mat P_left = KrectL * RT_left; //K[R|t]
    cv::Mat P_right = KrectR * RT_right;
    cv::Mat QL = P_left(cv::Rect(0, 0, 3, 3)); //P=[Q|q]
    cv::Mat QR = P_right(cv::Rect(0, 0, 3, 3));
    cv::Mat qL = P_left(cv::Rect(3, 0, 1, 3));
    cv::Mat qR = P_right(cv::Rect(3, 0, 1, 3));

    cv::Mat oc_L = -(QL.inv()) * qL; //Optical centers, should be shifted according to only one vector
    cv::Mat oc_R = -(QR.inv()) * qR;

    std::cout << "New optical centers : " << std::endl; std::cout  << oc_L  << std::endl; std::cout << oc_R << std::endl; std::cout  << std::endl; std::cout << std::endl;

    //Axis definition for new coordinates
    cv::Mat V1 = oc_L - oc_R;

    cv::Mat R3 = leftMean(cv::Rect(0, 2, 3, 1));
    cv::transpose(R3, R3);
    cv::Mat V2 = crossProductOpenCV(R3, V1);

    cv::Mat V3 = crossProductOpenCV(V1, V2);

    std::cout << "New axis : " << std::endl; std::cout << V1  << std::endl; std::cout << V2 << std::endl; std::cout  << V3 << std::endl; std::cout  << std::endl; std::cout << std::endl;

    //New extrinsic parameters, translation remains unchanged
    cv::Mat Rep;
    double n1 = eucliNormOpenCV(V1);
    double n2 = eucliNormOpenCV(V2);
    double n3 = eucliNormOpenCV(V3);
    cv::transpose(V1, V1);
    cv::transpose(V2, V2);
    cv::transpose(V3, V3);
    V1 = V1/n1;
    V2 = V2/n2;
    V3 = V3/n3;

    cv::vconcat(V1, V2, Rep);
    cv::vconcat(Rep, V3, Rep);

    std::cout << "New extrinsic parameters : " << std::endl; std::cout << Rep << std::endl; std::cout << std::endl;

    //New arbitrary intrinsic parameters
    cv::Mat Kep = (KrectL + KrectR)/2;

    std::cout << "New arbitrary intrinsic parameters : " << std::endl; std::cout << Kep << std::endl; std::cout << std::endl;

    //New projection matrices
    cv::Mat RTnL, RTnR;

    hconcat(Rep, -Rep*oc_L, RTnL);
    cv::Mat PnL = Kep*RTnL;

    hconcat(Rep, -Rep*oc_R, RTnR);
    cv::Mat PnR = Kep*RTnR;

    std::cout << "New projection matrices : " << std::endl; std::cout << PnL  << std::endl; std::cout << PnR << std::endl; std::cout << std::endl;

    //Rectifying image transformations
    cv::Mat QnL = PnL(cv::Rect(0, 0, 3, 3));
    cv::Mat QnR = PnR(cv::Rect(0, 0, 3, 3));

    cv::Mat TL = QnL * QL.inv();
    cv::Mat TR = QnR * QR.inv();

    std::cout << "Rectifying image transformations : " << std::endl; std::cout << TL  << std::endl; std::cout << TR << std::endl; std::cout << std::endl;

    cv::Mat fillIn = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat output, output2;

    cv::hconcat(TL, fillIn, output);
    cv::hconcat(output, TR, output2);

    return output2;

}


cv::Mat testRectif(cv::Mat imgIn, std::shared_ptr<LensFishEye> myLens){

    cv::Size imSize = imgIn.size();

    cv::Mat imgOut = cv::Mat::zeros(imSize, CV_32F);;

    cv::Mat mapx = cv::Mat::zeros(imSize, CV_32F);
    cv::Mat mapy = cv::Mat::zeros(imSize, CV_32F);

    double cu = myLens->getPrincipalU();
    double cv = myLens->getPrincipalV();

    double fu = myLens->getFocalU();
    double fv = myLens->getFocalV();

    for(int j=0; j<imgIn.rows; j++){
        for(int i=0; i<imgIn.cols; i++){

            double x = (double) i;
            double y = (double) j;

            std::array<double, 3> TDoutput;
            myLens->liftToUnitSphere(TDoutput, x, y);

            double Xt = TDoutput[0];
            double Yt = TDoutput[1];
            double Zt = TDoutput[2];

            //x = Xt/( sqrt(pow(Xt, 2) + pow(Yt, 2) + pow(Zt, 2)) + sqrt(pow(Yt, 2) + pow(Zt, 2)) );
            //y = Yt/( sqrt(pow(Yt, 2) + pow(Zt, 2)) + Zt );

            x = Xt/(1-Zt);
            y = Yt/(1-Zt);

            x = x*fu + cu;
            y = y*fv + cv;

            //std::cout << Xt << " - " << Yt << " - " << Zt << std::endl;
            //std::cout << x << " - " << y << std::endl;

            mapx.at<float>(j,i) = (float) x;
            mapy.at<float>(j,i) = (float) y;

            imgOut.at<float>(y,x) = imgIn.at<float>(j,i);

            if((j%100) == 0 && j >= 100 && i == 0){std::cout << j << std::endl;}

        }
    }

    std::cout << "Good" << std::endl;

    //cv::remap(imgIn, imgOut, mapx, mapy, cv::INTER_LINEAR, cv::BORDER_TRANSPARENT, cv::Scalar(0,0,0));

    return imgOut;

}
