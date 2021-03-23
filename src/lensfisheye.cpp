#include "lensFisheye.hpp"

#include <ceres/ceres.h>
#include "measure.hpp"
#include "camera.hpp"
#include "point.hpp"

#include <unsupported/Eigen/Polynomials>

class FishEyeIntrinsicsCostFunction : public ceres::SizedCostFunction<2, 3, 16, 4, 3> {
public:

  FishEyeIntrinsicsCostFunction(Measure * mes) {
    reference_measure = mes;
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {

    /*Read parameters from state vector */
    const double * parameters_point = parameters[0];
    const double * parameters_camera = parameters[1];
    const double * parameters_lens = parameters[2];
    const double * parameters_lens_advanced = parameters[3];

    /* Retrieve associated measure data */
    std::shared_ptr<Camera> reference_camera = reference_measure->getCamera();
    std::shared_ptr<Lens> reference_lens = reference_camera->getLens();
    LensFishEye * reference_lensfisheye = dynamic_cast<LensFishEye*>(reference_lens.get());

    /*Create lens (We want a copy to be able to update it safely)*/
    std::shared_ptr<LensFishEye> lens(new LensFishEye(*reference_lensfisheye));
    lens->setGenericParameters(parameters_lens);
    lens->setAdditionalParameters(parameters_lens_advanced);

    /*Create frame (We want a copy to be able to update it safely)*/
    std::shared_ptr<Frame> frame(new Frame);
    frame->setPoseParameters(parameters_camera);

    /*Create camera (We want a copy to be able to update it safely)*/
    Camera camera(lens);
    camera.addFrame(frame);

    /*Create point*/
    Point point;
    point.setPointParameters(parameters_point);

    /*Transform point*/
    Point tpoint;
    tpoint = frame->transform(point);

    /* Estimate measurement */
    std::array<double, 2> estimation = camera.project(point);
    residuals[0] = estimation[0] - reference_measure->getU();
    residuals[1] = estimation[1] - reference_measure->getV();

    if (jacobians) {
      Eigen::Map<Eigen::Matrix<double, 2, 16, Eigen::RowMajor> > Jcam(jacobians[1]);
      Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor> > Jlens(jacobians[2]);
      Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> > Jlensadv(jacobians[3]);

      if (jacobians[1]) {
        Eigen::Matrix<double, 2, 3> Jpoint;
        Eigen::Matrix<double, 3, 16> Jpose;
        Eigen::Matrix<double, 16, 16> Jchain;

        lens->jacobianWrtPoint(Jpoint, tpoint);
        camera.jacobianPointWrtPose(Jpose, point);
        camera.jacobianChainWrtPose(Jchain, 0);
        Jcam = Jpoint * Jpose * Jchain;
      }

      if (jacobians[2]) {
        Eigen::Matrix<double, 2, 4> J;
        lens->jacobianWrtGeneric(J, tpoint);
        Jlens = J;
      }

      if (jacobians[3]) {
        Eigen::Matrix<double, 2, -1> J;
        lens->jacobianWrtAdditional(J, tpoint);
        Jlensadv = J;
      }
    }

    return true;
  }

private:
  Measure * reference_measure;
};

class FishEyeExtrinsicsCostFunction : public ceres::SizedCostFunction<2, 16, 16, 16> {
public:

  FishEyeExtrinsicsCostFunction(Measure * mes) {
    reference_measure = mes;
  }

  virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    /*Read parameters from state vector */
    const double * parameters_camera_1 = parameters[0];
    const double * parameters_camera_2 = parameters[1];
    const double * parameters_camera_3 = parameters[2];

    /* Retrieve associated measure data */
    std::shared_ptr<Point> reference_point = reference_measure->getPoint();
    std::shared_ptr<Camera> reference_camera = reference_measure->getCamera();
    std::shared_ptr<Lens> reference_lens = reference_camera->getLens();

    /*Create frame*/
    std::shared_ptr<Frame> frame1(new Frame);
    std::shared_ptr<Frame> frame2(new Frame);
    std::shared_ptr<Frame> frame3(new Frame);

    /*Assign poses*/
    frame1->setPoseParameters(parameters_camera_1);
    frame2->setPoseParameters(parameters_camera_2);
    frame3->setPoseParameters(parameters_camera_3);

    /*Create camera*/
    Camera camera(reference_lens);
    camera.addFrame(frame1);
    camera.addFrame(frame2);
    camera.addFrame(frame3);

    /*Transform point*/
    Point tpoint;
    tpoint = frame1->transform(*reference_point);
    tpoint = frame2->transform(tpoint);
    tpoint = frame3->transform(tpoint);

    /* Estimate measurement */
    std::array<double, 2> estimation = camera.project(*reference_point);
    residuals[0] = estimation[0] - reference_measure->getU();
    residuals[1] = estimation[1] - reference_measure->getV();

    if (jacobians) {
      Eigen::Map<Eigen::Matrix<double, 2, 16, Eigen::RowMajor> > Jcam1(jacobians[0]);
      Eigen::Map<Eigen::Matrix<double, 2, 16, Eigen::RowMajor> > Jcam2(jacobians[1]);
      Eigen::Map<Eigen::Matrix<double, 2, 16, Eigen::RowMajor> > Jcam3(jacobians[2]);

      Eigen::Matrix<double, 2, 3> Jpoint;
      Eigen::Matrix<double, 3, 16> Jpose;
      reference_lens->jacobianWrtPoint(Jpoint, tpoint);
      camera.jacobianPointWrtPose(Jpose, *reference_point);

      Eigen::Matrix<double, 16, 16> Jchain;
      camera.jacobianChainWrtPose(Jchain, 0);
      Jcam1 = Jpoint * Jpose * Jchain;
      camera.jacobianChainWrtPose(Jchain, 1);
      Jcam2 = Jpoint * Jpose * Jchain;
      camera.jacobianChainWrtPose(Jchain, 2);
      Jcam3 = Jpoint * Jpose * Jchain;
    }

    return true;
  }

private:
  Measure * reference_measure;
};

LensFishEye::LensFishEye(int image_width, int image_height, const std::array<double, 2> & focal, const std::array<double, 2> & principal_point, const std::array<double, 3> & distorsion)
  : Lens(image_width, image_height, focal, principal_point) {
  this->additional[0] = distorsion[0];
  this->additional[1] = distorsion[1];
  this->additional[2] = distorsion[2];
}

double LensFishEye::getA() {
  return additional[0];
}

double LensFishEye::getB() {
  return additional[1];
}

double LensFishEye::getC() {
  return additional[2];
}

bool LensFishEye::liftToUnitSphere(std::array<double, 3> & result, double mesu) {

  double fu = getFocalU();
  double u = mesu - getPrincipalU();
  double x = u / fu;
  double y = 0.0;

  double A = getA();
  double B = getB();
  double C = getC();
  double D = 1.0 - A - B - C;

  /*
  Undistort :
  p' = s * p
  ||p'|| = s * ||p||
  s = f(||p||)
  r = ||p||
  s' = f(r)*r
  s' - s * r = 0
  f(r) = A*r^3 + B*r^2 + C*r + D
  s' - (A*r^4 + B*r^3 + C*r^2 + D*r) = 0
  */
  if (fabs(D - 1.0) > 1e-9) {
    double distorted_length = sqrt(x*x+y*y);

    Eigen::Matrix<double, 5, 1> list;
    list(4,0) = -A;
    list(3,0) = -B;
    list(2,0) = -C;
    list(1,0) = -D;
    list(0,0) = distorted_length;

    Eigen::PolynomialSolver<double, 4> solver;
    solver.compute<Eigen::Matrix<double, 5, 1> >(list);

    std::vector<double> roots;
    solver.realRoots(roots);

    /*Sort roots by increasing value*/
    std::sort(roots.begin(), roots.end());
    int index = -1;
    for (int id = 0; id < roots.size(); id++) {
      if (roots[id] < 0.0) continue;
      index = id;
      break;
    }

    if (index < 0) {
      std::cout << "Invalid polynomial root" << std::endl;
      return false;
    }

    double root = roots[index];
    x = x * root / distorted_length;
    y = y * root / distorted_length;
  }

  double len = x*x + y*y + 1.0;
  result[0] = (2.0 * x) / len;
  result[1] = (2.0 * y) / len;
  result[2] = - (x * x + y * y - 1.0) / len;

  return true;
}

bool LensFishEye::liftToUnitSphere(std::array<double, 3> & result, double mesu, double mesv) {

  double fu = getFocalU();
  double fv = getFocalV();
  double u = mesu - getPrincipalU();
  double v = mesv - getPrincipalV();
  double x = u / fu;
  double y = v / fv;

  double A = getA();
  double B = getB();
  double C = getC();
  double D = 1.0 - A - B - C;

  /*
  Undistort :
  p' = s * p
  ||p'|| = s * ||p||
  s = f(||p||)
  r = ||p||
  s' = f(r)*r
  s' - s * r = 0
  f(r) = A*r^3 + B*r^2 + C*r + D
  s' - (A*r^4 + B*r^3 + C*r^2 + D*r) = 0
  */
  if (fabs(D - 1.0) > 1e-9) {
    double distorted_length = sqrt(x*x+y*y);

    Eigen::Matrix<double, 5, 1> list;
    list(4,0) = -A;
    list(3,0) = -B;
    list(2,0) = -C;
    list(1,0) = -D;
    list(0,0) = distorted_length;

    Eigen::PolynomialSolver<double, 4> solver;
    solver.compute<Eigen::Matrix<double, 5, 1> >(list);

    std::vector<double> roots;
    solver.realRoots(roots);

    /*Sort roots by increasing value*/
    std::sort(roots.begin(), roots.end());
    int index = -1;
    for (int id = 0; id < roots.size(); id++) {
      if (roots[id] < 0.0) continue;
      index = id;
      break;
    }

    if (index < 0) {
      std::cout << "Invalid polynomial root" << std::endl;
      return false;
    }

    double root = roots[index];
    x = x * root / distorted_length;
    y = y * root / distorted_length;
  }

  double len = x*x + y*y + 1.0;
  result[0] = (2.0 * x) / len;
  result[1] = (2.0 * y) / len;
  result[2] = - (x * x + y * y - 1.0) / len;

  return true;
}

bool LensFishEye::liftToUnitSphere(std::array<double, 3> & result, const std::shared_ptr<Measure> & mes) {
  return liftToUnitSphere(result, mes->getU(), mes->getV());
}

std::array<double, 2> LensFishEye::project(const Point & point, bool inpixels) {
  std::array<double, 2> ret;

  double X = point.getX();
  double Y = point.getY();
  double Z = point.getZ();
  double len = sqrt(X*X+Y*Y+Z*Z);

  double u, v, x, y;

  /*Project point on the unit sphere*/
  X = X / len;
  Y = Y / len;
  Z = Z / len;
  x = X / (Z + 1.0);
  y = Y / (Z + 1.0);

  double radius = sqrt(x * x + y * y);
  double A = getA();
  double B = getB();
  double C = getC();
  double D = 1.0 - A - B - C;
  double scale = ((A * radius + B) * radius + C) * radius + D;

  if (inpixels) {
    u = x * scale * getFocalU() + getPrincipalU();
    v = y * scale * getFocalV() + getPrincipalV();
  }
  else {
    u = x;
    v = y;
  }

  ret[0] = u;
  ret[1] = v;

  return ret;
}


double * LensFishEye::getAdditionalParametersPtr() {
  return additional.data();
}

void LensFishEye::setAdditionalParameters(const double * vals) {
  additional[0] = vals[0];
  additional[1] = vals[1];
  additional[2] = vals[2];
}

bool LensFishEye::jacobianWrtAdditional(Eigen::Matrix<double, 2, -1> & J, const Point & cpoint) {

  std::array<double, 2> mes = this->project(cpoint, false);

  double x = mes[0];
  double y = mes[1];

  double radius = sqrt(x * x + y * y);
  double radius2 = radius * radius;
  double radius3 = radius2 * radius;

  double A = this->getA();
  double B = this->getB();
  double C = this->getC();
  double D = 1.0 - A - B - C;

  double scale = A * radius3 + B * radius2 + C * radius + D;
  double nx = scale * x;
  double ny = scale * y;

  Eigen::Matrix<double, 2, 3> Jdistort;
  double dscaleda = radius3 - 1.0;
  double dscaledb = radius2 - 1.0;
  double dscaledc = radius - 1.0;
  Jdistort(0, 0) = x * dscaleda;
  Jdistort(0, 1) = x * dscaledb;
  Jdistort(0, 2) = x * dscaledc;
  Jdistort(1, 0) = y * dscaleda;
  Jdistort(1, 1) = y * dscaledb;
  Jdistort(1, 2) = y * dscaledc;

  Eigen::Matrix<double, 2, 2> Jpix;
  double fu = getFocalU();
  double fv = getFocalV();
  Jpix(0, 0) = fu;
  Jpix(0, 1) = 0;
  Jpix(1, 0) = 0;
  Jpix(1, 1) = fv;

  J = Jpix * Jdistort;

  return true;
}

bool LensFishEye::jacobianWrtPoint(Eigen::Matrix<double, 2, 3> & J, const Point & cpoint) {

  /*Compute values*/
  double tX = cpoint.getX();
  double tY = cpoint.getY();
  double tZ = cpoint.getZ();
  double len = sqrt(tX*tX+tY*tY+tZ*tZ);
  double pX = tX / len;
  double pY = tY / len;
  double pZ = tZ / len;
  double x = pX / (pZ + 1.0);
  double y = pY / (pZ + 1.0);

  Eigen::Matrix<double, 3, 3> Jproj;
  double s1 = pow(tX*tX+tY*tY+tZ*tZ, 1.5);
  double s2 = 1.0 / len;
  double txs1 = tX / s1;
  double tys1 = tY / s1;
  double tzs1 = tZ / s1;
  Jproj(0, 0) = s2 - tX*txs1;
  Jproj(0, 1) = - tX*tys1;
  Jproj(0, 2) = - tX*tzs1;
  Jproj(1, 0) = - tY*txs1;
  Jproj(1, 1) = s2 - tY*tys1;
  Jproj(1, 2) = - tY*tzs1;
  Jproj(2, 0) = - tZ*txs1;
  Jproj(2, 1) = - tZ*tys1;
  Jproj(2, 2) = s2 - tZ*tzs1;

  Eigen::Matrix<double, 2, 3> Jpoint;
  Jpoint(0, 0) = (1.0 / (pZ + 1.0));
  Jpoint(0, 1) = 0.0;
  Jpoint(0, 2) = (-pX / ((pZ + 1.0)*(pZ + 1.0)));
  Jpoint(1, 0) = 0.0;
  Jpoint(1, 1) = (1.0 / (pZ + 1.0));
  Jpoint(1, 2) = (-pY / ((pZ + 1.0)*(pZ + 1.0)));

  /*Distortion jacobian wrt x,y*/
  Eigen::Matrix<double, 2, 2> Jdistort;
  double radius = sqrt(x * x + y * y);
  double radius2 = radius * radius;
  double radius3 = radius2 * radius;
  double A = this->getA();
  double B = this->getB();
  double C = this->getC();
  double D = 1.0 - A - B - C;
  double scale = A * radius3 + B * radius2 + C * radius + D;
  double nx = scale * x;
  double ny = scale * y;
  double drdx = x / radius;
  double drdy = y / radius;
  double dscaledr = radius * (3.0 * A * radius + 2.0 * B) + C;
  double dscaledx = dscaledr * drdx;
  double dscaledy = dscaledr * drdy;
  Jdistort(0, 0) = x * dscaledx + 1.0 * scale;
  Jdistort(0, 1) = y * dscaledy;
  Jdistort(1, 0) = x * dscaledx;
  Jdistort(1, 1) = y * dscaledy + 1.0 * scale;

  Eigen::Matrix<double, 2, 2> Jpix;
  double fu = this->getFocalU();
  double fv = this->getFocalV();
  Jpix(0, 0) = fu;
  Jpix(0, 1) = 0;
  Jpix(1, 0) = 0;
  Jpix(1, 1) = fv;

  J = Jpix * Jdistort * Jpoint * Jproj;

  return true;
}

ceres::CostFunction * LensFishEye::createIntrinsicsCostFunction(Measure * mes) {
  FishEyeIntrinsicsCostFunction * functor = new FishEyeIntrinsicsCostFunction(mes);
  return (ceres::CostFunction *)functor;
}

ceres::CostFunction * LensFishEye::createExtrinsicsCostFunction(Measure * mes) {
  FishEyeExtrinsicsCostFunction * functor = new FishEyeExtrinsicsCostFunction(mes);
  return (ceres::CostFunction *)functor;
}

/**
Display a report on the lens
*/
void LensFishEye::display() {
  double hwidth = 0.5 * (double)getImageWidth();
  double hheight = 0.5 * (double)getImageHeight();

  std::cout << "Lens parameters : " << std::endl;
  std::cout << "Image width and height : (" << getImageWidth() << "," << getImageHeight() << ")" << std::endl;
  std::cout << "Focal : (" << getFocalU() << "," << getFocalV() << ")" << std::endl;
  std::cout << "Principal point : (" << getPrincipalU() << "," << getPrincipalV() << ")" << std::endl;
  std::cout << "Principal point shift : (" << getPrincipalU() - hwidth << "," << getPrincipalV()  - hheight << ")" << std::endl;
  std::cout << "Distortion : [" << getA() << "," << getB() << "," << getC() << "]" << std::endl;
  std::cout << "Estimated FOV : " << getHorizontalFov() << std::endl;
}
