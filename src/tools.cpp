#include "tools.hpp"

double offsetDraw = 27;

cv::Mat crossProductOpenCV(cv::Mat A, cv::Mat B){

    //Works only with double CV_64F

    cv::Mat cp = cv::Mat::zeros(3, 1, CV_64F);

    cp.at<double>(0,0) = (A.at<double>(0,1) * B.at<double>(0,2)) - (A.at<double>(0,2) * B.at<double>(0,1));
    cp.at<double>(1,0) = (A.at<double>(0,2) * B.at<double>(0,0)) - (A.at<double>(0,0) * B.at<double>(0,2));
    cp.at<double>(2,0) = (A.at<double>(0,0) * B.at<double>(0,1)) - (A.at<double>(0,1) * B.at<double>(0,0));

    /*
      cp = A x B =
      A2*B3 - A3*B2
      A3*B1 - A1*B3
      A1*B2 - A2*B1
    */

    return cp;

}

double eucliNormOpenCV(cv::Mat A){

    double n;

    n = sqrt( pow(A.at<double>(0,0),2) + pow(A.at<double>(0,1),2) + pow(A.at<double>(0,2),2) );

    return n;

}

void loadImages(const std::string &imgPath, std::vector<cv::String> *imagesL, 
    std::vector<cv::String> *imagesR, int *w, int *h ,float *sz) {
    *w = 9; *h = 6; *sz = 2.15f;

    cv::String leftPath = imgPath + "*left*";
    cv::String rightPath = imgPath + "*right*";

    cv::glob(leftPath, *imagesL, false);
    cv::glob(rightPath, *imagesR, false);
}

void drawEpipoles(std::vector<cv::Point2f> Ep, const std::vector<std::string> &imagesL, 
    const std::vector<std::string> &imagesR, cv::Mat *imOut, int imNumber) {


    cv::Mat imgR = cv::imread(imagesR[imNumber]);
    cv::Point2f A = Ep[0], B = Ep[2], C = Ep[1], D = Ep[3];

    if(A.x > 0 && A.y > 0) {
        cv::drawMarker(imgR, A, cv::Scalar(0, 255, 255), 1, 20, 2, 8);
        cv::putText(imgR, std::string("ep1"), A, 1, 3, cv::Scalar(0, 255, 255), 2, 8);
    }

    if(B.x > 0 && B.y > 0) {
        cv::drawMarker(imgR, B, cv::Scalar(0, 255, 255), 1, 20, 2, 8);
        cv::putText(imgR, std::string("ep2"), B, 1, 3, cv::Scalar(0, 255, 255), 2, 8);
    }

    cv::Mat imgL = cv::imread(imagesL[imNumber]);

    if(C.x > 0 && C.y > 0) {
        cv::drawMarker(imgL, C, cv::Scalar(0, 255, 255), 1, 20, 2, 8);
        cv::putText(imgL, std::string("e1"), C, 1, 3, cv::Scalar(0, 255, 255), 2, 8);
    }

    if(D.x > 0 && D.y > 0) {
        cv::drawMarker(imgL, D, cv::Scalar(0, 255, 255), 1, 20, 2, 8);
        cv::putText(imgL, std::string("e2"), D, 1, 3, cv::Scalar(0, 255, 255), 2, 8);
    }

    cv::hconcat(imgL, imgR, imgL);
    cv::Mat imgDisp;
    cv::resize(imgL, imgDisp, cv::Size(0, 0), 0.5, 0.5);

    *imOut = imgL;

}

void drawEpipolarCurve(cv::Point2f inputPt, const std::shared_ptr<LensFishEye> &myLensL, 
    const std::shared_ptr<LensFishEye> &myLensR, cv::Mat F, cv::Mat *imOut, cv::Scalar Color) {

    std::array<double, 3> sp; //Lift to unit sphere
    myLensL->liftToUnitSphere(sp, inputPt.x, inputPt.y);
    cv::Mat spMat = cv::Mat::zeros(3, 1, CV_64F);
    spMat.at<double>(0,0) = sp[0];
    spMat.at<double>(1,0) = sp[1];
    spMat.at<double>(2,0) = sp[2];

    cv::Mat Fp = F*spMat; //F*p matrix from p' * F * p = 0

    std::vector<cv::Point2f> s1Pt, s2Pt; //Points describing the curve

    double fu = myLensR->getFocalU(); double cu = myLensR->getPrincipalU();
    double fv = myLensR->getFocalV(); double cv = myLensR->getPrincipalV();

    double A = myLensR->getA();
    double B = myLensR->getB();
    double C = myLensR->getC();
    double D = 1.0 - A - B - C;

    //Test 2nd degree equation
    for(int i=20; i<1180; i++){

        cv::Point2f drawPt;

        double u = (i - cu) / fu; //x coordinate
        double v; //y coordinate

        double a = -Fp.at<double>(2,0); //a = -Fp(3)
        double b = 2*Fp.at<double>(1,0); //b = 2*Fp(2)
        double c = 2*u*Fp.at<double>(0,0) - Fp.at<double>(2,0)*(u*u - 1); //c = - x^2 * Fp(3) + 2x*Fp(1) - Fp(3)

        double delta = b*b - 4*a*c;

        if(delta >= 0){

            double s1 = (-b - sqrt(delta)) / (2*a);
            double s2 = (-b + sqrt(delta)) / (2*a);

            double radius = sqrt(u * u + s1 * s1);
            double scale = ((A * radius + B) * radius + C) * radius + D;
            s1 = (s1 * scale * fv) + cv;
            v = s1;
            drawPt.x = (u * scale * fu) + cu + 1200; drawPt.y = v;
            s1Pt.push_back(drawPt);

            radius = sqrt(u * u + s2 * s2);
            scale = ((A * radius + B) * radius + C) * radius + D;
            s2 = (s2 * scale * fv) + cv;
            v = s2;
            drawPt.x = (u * scale * fu) + cu + 1200; drawPt.y = v; //+offsetDraw;
            s2Pt.push_back(drawPt);

        }

    }

    cv::Mat img = *imOut;

    cv::Mat curve2(s2Pt, true);
    curve2.convertTo(curve2, CV_32S);
    cv::polylines(img, curve2, false, Color, 2, cv::LINE_AA);

    *imOut = img;

}

void writeMatToFile(const cv::Mat &m, const std::string filename, std::string name) {
    std::ofstream fout(filename.c_str(), std::ios::app);
    fout << name + " : " << std::endl;
    for(int i=0; i<m.rows; i++) {
        for(int j=0; j<m.cols; j++) {
            fout<<m.at<double>(i,j)<<"\t";
        }
        fout << std::endl;
    }
    fout << std::endl;
    fout.close();
}
