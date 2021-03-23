#include "fisheyeIntrinsicEstimator.hpp"

fisheyeIntrinsicEstimator::fisheyeIntrinsicEstimator() {

}

fisheyeIntrinsicEstimator::~fisheyeIntrinsicEstimator() {

}

bool fisheyeIntrinsicEstimator::estimate(const PointGrid & grid, std::vector<std::shared_ptr<Camera> > & views) {

  double gamma, nx, ny, nz;
  double minscore;
  double bestgamma;

  minscore = std::numeric_limits<double>::max();


  for (int idimg = 0; idimg < views.size(); idimg++) {
    std::shared_ptr<Camera> cam = views[idimg];

    double cu = cam->getLens()->getPrincipalU();
    double cv = cam->getLens()->getPrincipalV();

    MeasuresGrid mgrid(cam, grid);

    for (int row = 0; row < grid.getGridHeight(); row++) {
      for (int col1 = 0; col1 < grid.getGridWidth() - 2; col1++) {
        std::shared_ptr<Measure> m1 = mgrid.at(row, col1);
        for (int col2 = col1 + 1; col2 < grid.getGridWidth() - 1; col2++) {
          std::shared_ptr<Measure> m2 = mgrid.at(row, col2);
          for (int col3 = col2 + 1; col3 < grid.getGridWidth(); col3++) {
            std::shared_ptr<Measure> m3 = mgrid.at(row, col3);

            std::array<double, 2> p1 = m1->getCoords();
            std::array<double, 2> p2 = m2->getCoords();
            std::array<double, 2> p3 = m3->getCoords();

            double u1 = p1[0] - cu;
            double v1 = p1[1] - cv;
            double u2 = p2[0] - cu;
            double v2 = p2[1] - cv;
            double u3 = p3[0] - cu;
            double v3 = p3[1] - cv;

            if (!estimateGamma(gamma, nx, ny, nz, u1, v1, u2, v2, u3, v3)) continue;
            if (gamma < 0.0) continue;

            double score = evaluateGamma(grid, views, gamma);

            if (score < minscore) {
              minscore = score;
              bestgamma = gamma;
            }
          }
        }
      }
    }
  }

  for (int idimg = 0; idimg < views.size(); idimg++) {
    std::shared_ptr<Camera> cam = views[idimg];
    cam->getLens()->setFocal(bestgamma, bestgamma);
  }


  return true;
}

bool fisheyeIntrinsicEstimator::estimateGamma(double & gamma, double & nx, double & ny, double & nz, double u1, double v1, double u2, double v2, double u3, double v3) {
  typedef Eigen::Matrix<double, 3, 4> Matrix34;
  Matrix34 M;

  M(0,0) = u1;
  M(0,1) = v1;
  M(0,2) = 0.5;
  M(0,3) = -0.5*(u1*u1 + v1*v1);

  M(1,0) = u2;
  M(1,1) = v2;
  M(1,2) = 0.5;
  M(1,3) = -0.5*(u2*u2 + v2*v2);

  M(2,0) = u3;
  M(2,1) = v3;
  M(2,2) = 0.5;
  M(2,3) = -0.5*(u3*u3 + v3*v3);

  Eigen::JacobiSVD<Matrix34> svd(M, Eigen::ComputeFullV);

  double c1 = svd.matrixV()(0,3);
  double c2 = svd.matrixV()(1,3);
  double c3 = svd.matrixV()(2,3);
  double c4 = svd.matrixV()(3,3);

  double t = c1*c1 + c2*c2 + c3*c4;
  if (fabs(t) < 1e-6) {
    return false;
  }

  double d = sqrt(1.0 / t);
  nx = c1 * d;
  ny = c2 * d;

  double thresh = nx * nx + ny * ny;
  if (thresh > 0.95) {
    return false;
  }

  nz = sqrt(1.0 - thresh);
  gamma = c3 * d / nz;

  return true;
}

double fisheyeIntrinsicEstimator::evaluateGamma(const PointGrid & grid, std::vector<std::shared_ptr<Camera> > & views, double gamma) {

  typedef Eigen::Matrix<double, 3, 3> Matrix33;
  Matrix33 M;
  double sum = 0;

  for (int idimg = 0; idimg < views.size(); idimg++) {
    std::shared_ptr<Camera> cam = views[idimg];

    double cu = cam->getLens()->getPrincipalU();
    double cv = cam->getLens()->getPrincipalV();

    /*Loop through every rows*/
    MeasuresGrid mgrid(cam, grid);

    for (int row = 0; row < grid.getGridHeight(); row++) {

      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> M(grid.getGridWidth(), 3);

      for (int col = 0; col < grid.getGridWidth(); col++) {
        std::shared_ptr<Measure> m = mgrid.at(row, col);
        std::array<double, 2> p = m->getCoords();

        double x = (p[0] - cu) / gamma;
        double y = (p[1] - cv) / gamma;

        double len = x*x + y*y + 1.0;
        double X = -(2.0 * x) / len;
        double Y = -(2.0 * y) / len;
        double Z = (x * x + y * y - 1.0) / len;

        M(col, 0) = X;
        M(col, 1) = Y;
        M(col, 2) = Z;
      }

      Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> > svd(M, Eigen::ComputeFullV);

      double a = svd.matrixV()(0,2);
      double b = svd.matrixV()(1,2);
      double c = svd.matrixV()(2,2);

      double dist = 0.0;
      for (int col = 0; col < grid.getGridWidth(); col++) {
        std::shared_ptr<Measure> m = mgrid.at(row, col);
        std::array<double, 2> p = m->getCoords();
        double x = (p[0] - cu) / gamma;
        double y = (p[1] - cv) / gamma;

        double len = x*x + y*y + 1.0;
        double X = -(2.0 * x) / len;
        double Y = -(2.0 * y) / len;
        double Z = (x * x + y * y - 1.0) / len;

        double ldist = (a*X + b*Y + c*Z);
        dist = dist + ldist * ldist;
      }

      sum += dist;
    }
  }

  return sum;
}
