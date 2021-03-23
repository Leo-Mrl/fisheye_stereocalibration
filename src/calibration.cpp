#include "calibration.hpp"

std::vector<cv::Point3f> Create3DChessboardCorners(const cv::Size boardSize, const
    float squareSize) {
    // This function creates the 3D points of your chessboard in its own coordinate system
    std::vector<cv::Point3f> corners;
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            corners.push_back(cv::Point3f(float(j*squareSize), float(i*squareSize), 0));
        }
    }
    return corners;
}

std::shared_ptr<LensFishEye> calibration(const int gridwidth, const int gridheight,
    const double squaresize, const std::vector<std::string> &images,
    std::vector<cv::Mat> *rotationsVec, std::vector<cv::Mat> *translationsVec,
    cv::Mat *Kam, cv::Mat *Dist) {
    // Distortion parameters
    std::array<double, 3> distortion;
    distortion[0] = 0;
    distortion[1] = 0;
    distortion[2] = 0;

    // focal
    std::array<double, 2> focal;
    focal[0] = 1.0;
    focal[1] = 1.0;

    /*Create lens*/
    std::shared_ptr<LensFishEye> lens(nullptr), fail(nullptr);

    /*Initialize point grid*/
    PointGrid grid(squaresize, gridwidth, gridheight);

    /*Extraction init*/
    CheckerBoard cextractor;

    /*Measures collection*/
    std::vector<std::shared_ptr<Camera> > views;

    printf("\nCheckerboards are expected to be [ %dx%d squares | %.2f cm ]\n", gridwidth,
        gridheight, squaresize);
    for (size_t idimg = 0; idimg < images.size(); idimg++) {
        cv::Mat img;

        img = cv::imread(images[idimg]);
        if (img.data == nullptr) {
            std::cout << "Failed to load image : " << images[idimg] << std::endl;
            return fail;
        }

        int width = img.cols;
        int height = img.rows;

        if (lens == nullptr) {
            // Image center
            std::array<double, 2> principal_point;
            principal_point[0] = width / 2;
            principal_point[1] = height / 2;
            lens = std::shared_ptr<LensFishEye>(new LensFishEye(width, height, focal,
                principal_point, distortion));
        }

        /*Extract corners from image*/
        if (!cextractor.extract(img, lens, grid)) {
            std::cout << "Failed to detect checkerboard on this image : " <<
            images[idimg] << std::endl;
            return fail;
        }

        std::shared_ptr<Frame> frame(new Frame);
        std::shared_ptr<Camera> camera = cextractor.getCamera();
        camera->addFrame(frame);
        views.push_back(camera);
    }

    /*Loop over source images*/
    if (views.size() == 0) {
        std::cout << "No valid images provided." << std::endl;
        return fail;
    }

    /*Make a coarse estimation of focal length*/
    fisheyeIntrinsicEstimator iest;
    if (iest.estimate(grid, views) == false) {
        std::cout << "Estimation failed." << std::endl;
        return fail;
    } else {
        printf("\nEstimation of intrinsic parameters succeeded, focal length in u : %.2f",
            lens->getFocalU());
    }

    printf("\nEstimation of extrinsic parameters...\n");

    /*Make a coarse estimation of camera extrinsics*/
    ExtrinsicEstimator eest;
    int pos = 0;
    for (std::vector<std::shared_ptr<Camera> >::iterator it = views.begin(); it != views.end();
        it++) {
        if (eest.estimate(grid, *it) == false) {
            std::cout << "Estimation of view #" << pos << " has failed." << std::endl;
            return fail;
        }
        pos++;
    }

    /*Refine parameters*/
    RefinementEstimator rest;
    if (rest.estimate(grid, views) == false) {
        return fail;
    }

    pos = 0;
    for (std::vector<std::shared_ptr<Camera> >::iterator it = views.begin(); it != views.end();
        it++) {
        double variance = 0.0;
        double val = (*it)->computeCurrentCost(&variance);
        pos++;
        printf("Camera # %d mean reprojection error : %.5f (+/- %.5f) px\n", pos, val,
            sqrt(variance));
    }
    std::cout << std::endl;

    /*Display report*/
    lens->display();
    std::cout << std::endl;

    cv::Mat K = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat D = cv::Mat::zeros(4, 1, CV_64F);
    K.at<double>(0, 0) = lens->getFocalU();
    K.at<double>(1, 1) = lens->getFocalV();
    K.at<double>(0, 2) = lens->getPrincipalU();
    K.at<double>(1, 2) = lens->getPrincipalV();
    K.at<double>(2, 2) = 1;
    D.at<double>(0, 0) = lens->getA();
    D.at<double>(0, 1) = lens->getB();
    D.at<double>(0, 2) = lens->getC();

    for (int i = 0; i < fabs(views.size()); i++) {
        cv::Mat Rotation = cv::Mat::zeros(3, 3, CV_64F);
        cv::Mat Translation = cv::Mat::zeros(3, 1, CV_64F);

        Eigen::Matrix4d tmpRotation = views[i]->getFirstFrame()->getPose();  // Camera pose
        Rotation.at<double>(0, 0) = tmpRotation(0, 0);
        Rotation.at<double>(0, 1) = tmpRotation(0, 1);
        Rotation.at<double>(0, 2) = tmpRotation(0, 2);
        Rotation.at<double>(1, 0) = tmpRotation(1, 0);
        Rotation.at<double>(1, 1) = tmpRotation(1, 1);
        Rotation.at<double>(1, 2) = tmpRotation(1, 2);
        Rotation.at<double>(2, 0) = tmpRotation(2, 0);
        Rotation.at<double>(2, 1) = tmpRotation(2, 1);
        Rotation.at<double>(2, 2) = tmpRotation(2, 2);

        rotationsVec->push_back(Rotation);

        Translation.at<double>(0, 0) = tmpRotation(0, 3);
        Translation.at<double>(0, 1) = tmpRotation(1, 3);
        Translation.at<double>(0, 2) = tmpRotation(2, 3);

        translationsVec->push_back(Translation);
    }
    cv::destroyAllWindows();

    *Kam = K;
    *Dist = D;

    return lens;
}

cv::Mat cameraMatrixforEG(std::vector<std::string> images, std::shared_ptr<LensFishEye> myLens,
    cv::Size boardSize, float squareSize) {
    int n = 0;
    std::vector<std::string> goodMatches;
    std::vector<std::vector<cv::Point2f> > cornersAll;
    cv::Mat imgTmp;

    cv::Size winSize(5, 5);
    cv::Size zeroZone(-1, -1);
    cv::TermCriteria criteria(cv::TermCriteria::EPS, 40, 0.0001);

    cv::Mat imgTest = cv::imread(images[0]);
    cv::Size imSize = imgTest.size();
    cv::Mat mapx = cv::Mat(imSize, CV_32F);
    cv::Mat mapy = cv::Mat(imSize, CV_32F);

    for (int i = 0; i < fabs(images.size()); i++) {
        imgTmp = cv::imread(images[i]);
        imgTmp = rectification(imgTmp, myLens, &mapx, &mapy);

        cv::Mat sharp, blur;
        cv::GaussianBlur(imgTmp, blur, cv::Size(5, 5), 3);
        cv::addWeighted(imgTmp, 1.5, blur, -0.5, 0, sharp);
        sharp = imgTmp;

        std::vector<cv::Point2f> cornersTemp;
        bool found = cv::findChessboardCorners(sharp, boardSize, cornersTemp,
            cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FILTER_QUADS +
            cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found == 1) {
            n+=1;
            goodMatches.push_back(images[i]);

            cv::Mat gray;
            cv::cvtColor(imgTmp, gray, cv::COLOR_BGR2GRAY);
            cv::cornerSubPix(gray, cornersTemp, winSize, zeroZone, criteria);

            cornersAll.push_back(cornersTemp);
        }
    }

    std::cout << "Detections post-rectification : " << n << "/" << images.size() << std::endl;

    std::vector<std::vector<cv::Point3f> > objectPoints;
    for (int i = 0; i < fabs(goodMatches.size()); i++) {
        objectPoints.push_back(Create3DChessboardCorners(boardSize, squareSize));
    }

    imSize = imgTmp.size();
    cv::Mat Kbis = cv::initCameraMatrix2D(objectPoints, cornersAll, imSize);

    std::cout << "Rectified perspective projection matrix : " << std::endl;
    std::cout << Kbis << std::endl;

    return Kbis;
}

void extrinsicEstimation(const std::vector<cv::Mat> &RotationsL, const std::vector<cv::Mat>
    &RotationsR, const std::vector<cv::Mat> &TranslationsL, const std::vector<cv::Mat>
    &TranslationsR, cv::Mat *rbmLeft, cv::Mat *rbmRight, cv::Mat *rbmMean, int num) {
    cv::Mat tmpFill = cv::Mat::zeros(1, 4, CV_64F);
    tmpFill.at<double>(0, 3) = 1;

    cv::Mat leftMean = cv::Mat::zeros(4, 4, CV_64F);
    cv::Mat rightMean = cv::Mat::zeros(4, 4, CV_64F);
    cv::Mat rbmLeftTmp = cv::Mat::zeros(4, 4, CV_64F);
    cv::Mat rbmRightTmp = cv::Mat::zeros(4, 4, CV_64F);
    cv::Mat rbmMeanTmp = cv::Mat::zeros(4, 4, CV_64F);

    for (int i = 0; i < num; i++) {
        cv::hconcat(RotationsL[i], TranslationsL[i], rbmLeftTmp);
        cv::vconcat(rbmLeftTmp, tmpFill, rbmLeftTmp);

        cv::hconcat(RotationsR[i], TranslationsR[i], rbmRightTmp);
        cv::vconcat(rbmRightTmp, tmpFill, rbmRightTmp);

        rbmMeanTmp += rbmRightTmp*(rbmLeftTmp.inv());  // Right to left camera

        leftMean = leftMean + rbmLeftTmp;
        rightMean = rightMean + rbmRightTmp;
    }
    rbmMeanTmp = rbmMeanTmp/num;
    leftMean = leftMean/num;
    rightMean = rightMean/num;

    cv::Mat rtTemp, trTemp;

    rtTemp = leftMean(cv::Rect(0, 0, 3, 3));
    trTemp = leftMean(cv::Rect(3, 0, 1, 3));
    cv::Rodrigues(rtTemp, rtTemp);

    rtTemp = rightMean(cv::Rect(0, 0, 3, 3));
    trTemp = rightMean(cv::Rect(3, 0, 1, 3));
    cv::Rodrigues(rtTemp, rtTemp);

    cv::Mat rtDegrees, rtFinal, trFinal;
    rtFinal = rbmMeanTmp(cv::Rect(0, 0, 3, 3));
    trFinal = rbmMeanTmp(cv::Rect(3, 0, 1, 3));
    float distCam = sqrt(pow(trFinal.at<double>(0, 0), 2) + pow(trFinal.at<double>(1, 0), 2) +
        pow(trFinal.at<double>(2, 0), 2));

    cv::Rodrigues(rtFinal, rtDegrees);
    printf("\nRotation matrix in degrees :\n");
    std::cout << rtDegrees*180/M_PI;

    printf("\nTranslation matrix in cm :\n");
    std::cout << trFinal;
    printf("\nDistance between cameras in cm : %.2f\n", distCam);

    *rbmLeft = rbmLeftTmp;
    *rbmRight = rbmRightTmp;
    *rbmMean = rbmMeanTmp;
}
