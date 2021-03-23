#include "demoepipolar.hpp"

cv::String demoTitle = "Epipolar curve PoC : click on the left image to display the corresponding epipolar curve on the right - Press ESC to quit";

struct MouseParams{
    std::shared_ptr<LensFishEye> myLensL;
    std::shared_ptr<LensFishEye> myLensR;
    cv::Mat F;
    cv::Mat img;
    double rsCoef;
};

void onMouse(int event, int x, int y, int, void *ptr){
    if(event == cv::EVENT_LBUTTONDOWN){
        MouseParams* mp = (MouseParams*) ptr;
        std::shared_ptr<LensFishEye> tmpLensL = mp->myLensL;
        std::shared_ptr<LensFishEye> tmpLensR = mp->myLensR;
        cv::Mat myF = mp->F;
        cv::Mat myImg = mp->img;
        double myCoef = mp->rsCoef;

        if(x < myImg.cols/2-1) {
            int v1 = rand() % 256; int v2 = rand() % 256; int v3 = rand() % 256;
            cv::Scalar Color = cv::Scalar(v1, v2, v3);

            cv::Point2f mPos;
            mPos.x = (double) x; mPos.y = (double) y;
            cv::drawMarker(myImg, mPos, Color, 1, 10, 2, 8);
            mPos.x = (double) x/myCoef; mPos.y = (double) y/myCoef; // Resize

            std::array<double, 3> sp; //Lift to unit sphere
            tmpLensL->liftToUnitSphere(sp, mPos.x, mPos.y);
            cv::Mat spMat = cv::Mat::zeros(3, 1, CV_64F);
            spMat.at<double>(0,0) = sp[0];
            spMat.at<double>(1,0) = sp[1];
            spMat.at<double>(2,0) = sp[2];

            cv::Mat Fp = myF*spMat; // F*p matrix from p' * F * p = 0
            std::vector<cv::Point2f> s1Pt, s2Pt; // Points describing the curve

            double fu = tmpLensR->getFocalU(); double cu = tmpLensR->getPrincipalU();
            double fv = tmpLensR->getFocalV(); double cv = tmpLensR->getPrincipalV();

            double A = tmpLensR->getA();
            double B = tmpLensR->getB();
            double C = tmpLensR->getC();
            double D = 1.0 - A - B - C;

            // Test 2nd degree equation
            for(int i=0; i<(myImg.cols/2)/myCoef; i++){
                cv::Point2f drawPt;

                double u = (i - cu) / fu; // x coordinate
                double v; //y coordinate

                double a = -Fp.at<double>(2,0); // a = -Fp(3)
                double b = 2*Fp.at<double>(1,0); // b = 2*Fp(2)
                double c = 2*u*Fp.at<double>(0,0) - Fp.at<double>(2,0)*(u*u - 1); // c = - x^2 * Fp(3) + 2x*Fp(1) - Fp(3)

                double delta = b*b - 4*a*c;

                if(delta >= 0){
                    double s1 = (-b - sqrt(delta)) / (2*a);
                    double s2 = (-b + sqrt(delta)) / (2*a);

                    double radius = sqrt(u * u + s1 * s1);
                    double scale = ((A * radius + B) * radius + C) * radius + D;
                    s1 = (s1 * scale * fv) + cv;
                    v = s1;
                    drawPt.x = (u * scale * fu) + cu; drawPt.y = v;
                    drawPt.x = myCoef*drawPt.x; drawPt.y = myCoef*drawPt.y; // Resize
                    s1Pt.push_back(drawPt);

                    radius = sqrt(u * u + s2 * s2);
                    scale = ((A * radius + B) * radius + C) * radius + D;
                    s2 = (s2 * scale * fv) + cv;
                    v = s2;
                    drawPt.x = (u * scale * fu) + cu + (myImg.cols/2)/myCoef;
                    drawPt.y = v;
                    drawPt.x = myCoef*drawPt.x; drawPt.y = myCoef*drawPt.y; // Resize

                    if(drawPt.x > myImg.cols/2) {
                        s2Pt.push_back(drawPt);
                    }
                }
            }
            cv::Mat curve2(s2Pt, true);
            curve2.convertTo(curve2, CV_32S);
            cv::polylines(myImg, curve2, false, Color, 1, cv::LINE_AA);
            cv::imshow(demoTitle, myImg);
        }
    }
}

void demonstrationEpipolarCurve(const std::vector<std::string> &imagesL, 
    const std::vector<std::string> &imagesR, const std::shared_ptr<LensFishEye> &myLensL, 
    const std::shared_ptr<LensFishEye> &myLensR, cv::Mat F, int imgSel) {
    if(imgSel >= fabs(imagesL.size())){imgSel = imagesL.size() - 1;}
    if(imgSel < 0){imgSel = 0;}
    printf("\n\n |> Epipolar curve drawing demonstration\nImage n=%d selected : %s\n", imgSel, imagesL[imgSel].c_str());

    cv::Mat img;
    std::vector<cv::Point2f> Ep = findEpipoles(F, myLensL, myLensR);
    drawEpipoles(Ep, imagesL, imagesR, &img, imgSel);

    double coef = 1800/double(img.cols); // Resizing coefficient, set at 1800 when expecting a typical 1920x1080 display
    int coefI = (int) (coef*100);
    cv::resize(img, img, cv::Size(0,0), coef, coef);
    printf("Displaying image at %d%\n\n        Press ESC to quit\n\n", coefI);

    MouseParams myParam;
    myParam.F = F;
    myParam.img = img;
    myParam.myLensL = myLensL;
    myParam.myLensR = myLensR;
    myParam.rsCoef = coef;

    cv::namedWindow(demoTitle, 1);
    cv::setMouseCallback(demoTitle, onMouse, (void*)&myParam);
    cv::imshow(demoTitle, img);
    cv::waitKey(0);
}
