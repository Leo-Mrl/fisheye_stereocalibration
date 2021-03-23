#include "checkerBoard.hpp"

CheckerBoard::CheckerBoard() {
}


CheckerBoard::~CheckerBoard() {
}

std::shared_ptr<Camera> CheckerBoard::getCamera() {
  return camera;
}

bool CheckerBoard::load(const std::string & source, const std::shared_ptr<Lens> & lens, const PointGrid & grid) {
  std::vector<cv::Point2d> corners;
  camera.reset();

  /*Define pattern size using grid definition*/
  cv::Size patternSize;
  patternSize.width = (int)grid.getGridWidth();
  patternSize.height = (int)grid.getGridHeight();

  /*Extract corners from image*/
  FILE * f = fopen(source.c_str(), "r");
  if (!f) return false;

  /*Create associated view*/
  camera = std::shared_ptr<Camera>(new Camera(lens));

  /*Create measures*/
  int pos = 0;
  for (int i = 0; i < patternSize.height; i++) {
    for (int j = 0; j < patternSize.width; j++) {

      const std::shared_ptr<Point> p = grid.getPoint(i, j);
      cv::Point2d m;

      int read = fscanf(f, "%lf %lf\n", &m.x, &m.y);
      if (read == 0) continue;

      std::shared_ptr<Measure> mes(new Measure(camera, p, m.x, m.y));
      camera->addMeasure(mes);
      pos++;
    }
  }

  fclose(f);

  return true;
}

bool CheckerBoard::extract(const cv::Mat & source, const std::shared_ptr<Lens> & lens, const PointGrid & grid) {
  std::vector<cv::Point2f> corners;
  camera.reset();

  if (source.rows != lens->getImageHeight()) return false;
  if (source.cols != lens->getImageWidth()) return false;

  /*Define pattern size using grid definition*/
  cv::Size patternSize;
  patternSize.width = (int)grid.getGridWidth();
  patternSize.height = (int)grid.getGridHeight();

  /*Extract corners from image*/

  bool cornersFound = findChessboardCorners(source, patternSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH);
  if (!cornersFound) return false;

  /*Create associated view*/
  camera = std::shared_ptr<Camera>(new Camera(lens));

  /*Create measures*/
  int pos = 0;
  for (int i = 0; i < patternSize.height; i++) {
    for (int j = 0; j < patternSize.width; j++) {

      const std::shared_ptr<Point> p = grid.getPoint(i, j);
      const cv::Point2d m = corners[pos];

      std::shared_ptr<Measure> mes(new Measure(camera, p, m.x, m.y));
      camera->addMeasure(mes);
      pos++;
    }
  }

  return true;
}

Eigen::Matrix<bool, -1, -1> CheckerBoard::computeMask(const cv::Mat & source, const PointGrid & grid) {

  const int h = grid.getGridHeight() + 1;
  const int w = grid.getGridWidth() + 1;
  const double s = grid.getSquareDimension();

  Eigen::Matrix<bool, -1, -1> ret(h, w);

  double mean;
  double sum;
  Eigen::Matrix<double, 8, 8> vals;

  for (int i = 0; i < h; i++) {
    for (int j = 0; j < w; j++) {

      sum = 0.0;
      for (int k = 0; k < 8; k++) {
        double py = (((double)k) + 1.0) / 10.0;

        for (int l = 0; l < 8; l++) {
          double px = (((double)l) + 1.0) / 10.0;

          Point p;
          p.setX(((double)j - 1) * s + s * px);
          p.setY(((double)i - 1) * s + s * py);
          p.setZ(0.0);

          std::array<double, 2> pix = camera->project(p, true);
          vals(k,l) = source.at<unsigned char>(pix[1], pix[0]);

          sum += vals(k, l);
        }
      }

      mean = sum / 64.0;

      sum = 0.0;
      for (int k = 0; k < 8; k++) {
        for (int l = 0; l < 8; l++) {

          double centered = vals(k, l) - mean;
          sum += (centered * centered);
        }
      }

      double std = sum / 64.0;
      ret(i, j) = (std > 500.0)?true:false;
    }
  }

  return ret;
}
