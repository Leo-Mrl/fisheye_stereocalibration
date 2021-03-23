#include "opencv2/imgproc/imgproc.hpp"

#include "calibration.hpp"
#include "demoepipolar.hpp"
#include "epipolar.hpp"
#include "lensFisheye.hpp"
#include "tools.hpp"

int main(int argc, char **argv) {
    std::string img_path;
    if(argc > 1) {
        img_path = argv[1];
        printf("\nOverriding path to the calibration images : %s\n", img_path.c_str());
    }

    int imgSel = 0;
    if(argc > 2){
        imgSel = atoi(argv[2]);
    }

    int w, h;
    float sz;
    std::vector<cv::String> imagesL, imagesR;

    loadImages(img_path, &imagesL, &imagesR, &w, &h, &sz);
    int numIm = imagesL.size();

    std::shared_ptr<LensFishEye> myLensL(nullptr), myLensR(nullptr);
    std::vector<cv::Mat> RotationsL, RotationsR, TranslationsL, TranslationsR;
    cv::Mat KL, DL, KR, DR;

    printf("\n\n |> Calibration of the left camera...");
    myLensL = calibration(w, h, sz, imagesL, &RotationsL, &TranslationsL, &KL, &DL);

    printf("\n\n |> Calibration of the right camera...");
    myLensR = calibration(w, h, sz, imagesR, &RotationsR, &TranslationsR, &KR, &DR);

    printf("\n\n |> Extrinsic parameters estimation and refinement...");
    cv::Mat rbmLeft, rbmRight, rbmMean;
    extrinsicEstimation(RotationsL, RotationsR, TranslationsL, TranslationsR, &rbmLeft, &rbmRight, 
        &rbmMean, numIm);

    printf("\n\n |> Fudamental matrix computation...");
    cv::Mat F = computeF(imagesL, imagesR, myLensL, myLensR, rbmMean, w, h);

    demonstrationEpipolarCurve(imagesL, imagesR, myLensL, myLensR, F, imgSel);

    return 0;
}
