#include "camera.hpp"

/** Jacobian tools */
void computedABdA(Eigen::Matrix<double, 16, 16> & J, const Eigen::Matrix4d & A, const Eigen::Matrix4d & B) {
  J.fill(0.0);
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      J(j * 4 + i, 0 + i) = B(0, j);
      J(j * 4 + i, 4 + i) = B(1, j);
      J(j * 4 + i, 8 + i) = B(2, j);
      J(j * 4 + i, 12 + i) = B(3, j);
    }
  }
}

void computedABdB(Eigen::Matrix<double, 16, 16> & J, const Eigen::Matrix4d & A, const Eigen::Matrix4d & B) {
  J.fill(0.0);
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      J(i + 0 * 4, j + 0 * 4) = A(i, j);
      J(i + 1 * 4, j + 1 * 4) = A(i, j);
      J(i + 2 * 4, j + 2 * 4) = A(i, j);
      J(i + 3 * 4, j + 3 * 4) = A(i, j);
    }
  }
}

Camera::Camera(std::shared_ptr<Lens> lens) {
  this->lens = lens;
  uid = 0;
}

std::shared_ptr<Lens> Camera::getLens() {
  return lens;
}

std::array<double, 2> Camera::project(const Point & p, bool inpixels) {
  Point tp = p;
  for (int i = 0; i < frames.size(); i++) {
    tp = frames[i]->transform(tp);
  }
  std::array<double, 2> ret = lens->project(tp, inpixels);
  return ret;
}

double Camera::computeCurrentCost(double * variance) {

  double ret = 0.0;
  int count = 0;

  /**
  Loop through all measures to compute mean
  */
  for (MeasuresVector::iterator itmes = measures_list.begin(); itmes != measures_list.end(); itmes++) {
    std::shared_ptr<Point> pt = (*itmes)->getPoint();
    std::array<double, 2> mes = (*itmes)->getCoords();
    std::array<double, 2> est = project(*pt);
    double dx = est[0] - mes[0];
    double dy = est[1] - mes[1];

    /*Get L2 distance*/
    ret += sqrt(dx*dx+dy*dy);
    count++;
  }

  double mean = ret / (double)count;

  if (variance) {
    /**
    Loop through all measures to compute sigma
    */
    count = 0;
    for (MeasuresVector::iterator itmes = measures_list.begin(); itmes != measures_list.end(); itmes++) {
      std::shared_ptr<Point> pt = (*itmes)->getPoint();
      std::array<double, 2> mes = (*itmes)->getCoords();
      std::array<double, 2> est = project(*pt);
      double dx = est[0] - mes[0];
      double dy = est[1] - mes[1];

      /*Get L2 distance*/
      ret += sqrt(dx*dx+dy*dy) - mean;
      count++;
    }

    *variance = ret / (double)count;
  }

  return mean;
}

void Camera::setId(unsigned int id) {
  uid = id;
}

unsigned int Camera::getId() {
  return uid;
}

std::vector<std::shared_ptr<Frame> > Camera::getKinematicChain() {
  return frames;
}

std::shared_ptr<Frame> Camera::getFirstFrame() {
  return frames[0];
}

void Camera::clearFrames() {
  frames.clear();
}

void Camera::addFrame(std::shared_ptr<Frame> & frame) {
  frames.push_back(frame);
}

void Camera::addMeasure(std::shared_ptr<Measure> & mes) {
  measures_list.push_back(mes);
}

bool Camera::jacobianChainWrtPose(Eigen::Matrix<double, 16, 16> & J, unsigned int poseIndex) {
  Eigen::Matrix4d A, B, C;

  if (poseIndex >= frames.size()) {
    return false;
  }

  /**
  Let chain be M6*M5*M4*M3*M2*M1
  index = 2
  A = M6*M5
  B = I
  C = M4*M3*M2*M1
  */

  A.setIdentity();
  B.setIdentity();
  C.setIdentity();

  for (int i = 0; i <= poseIndex; i++) {
    C = frames[i]->getPose() * C;
  }


  for (int i = poseIndex + 1; i < frames.size(); i++) {
    A = frames[i]->getPose() * A;
  }

  /*
  J = d(A*B*C)/dB
  d(M*C)/dM * dM/dB with M=A*B
  */
  Eigen::Matrix4d M = A * B;
  Eigen::Matrix<double, 16, 16> J1, J2;
  computedABdA(J1, M, C);
  computedABdB(J2, A, B);

  J = J1 * J2;

  return true;
}

bool Camera::jacobianPointWrtPose(Eigen::Matrix<double, 3, 16> & Jpose, const Point & point) {

  double X = point.getX();
  double Y = point.getY();
  double Z = point.getZ();

  /*
  X' = M11 * X + M12 * Y + M13 * Z + M14 * 1
  Y' = M21 * X + M22 * Y + M23 * Z + M24 * 1
  Z' = M31 * X + M32 * Y + M33 * Z + M34 * 1
  0 =  M41 * 0 + M42 * 0 + M43 * 0 + M44 * 1
  */

  Jpose(0, 0) = X;
  Jpose(0, 1) = 0.0;
  Jpose(0, 2) = 0.0;
  Jpose(0, 3) = 0.0;
  Jpose(0, 4) = Y;
  Jpose(0, 5) = 0.0;
  Jpose(0, 6) = 0.0;
  Jpose(0, 7) = 0.0;
  Jpose(0, 8) = Z;
  Jpose(0, 9) = 0.0;
  Jpose(0, 10) = 0.0;
  Jpose(0, 11) = 0.0;
  Jpose(0, 12) = 1.0;
  Jpose(0, 13) = 0.0;
  Jpose(0, 14) = 0.0;
  Jpose(0, 15) = 0.0;

  Jpose(1, 0) = 0.0;
  Jpose(1, 1) = X;
  Jpose(1, 2) = 0.0;
  Jpose(1, 3) = 0.0;
  Jpose(1, 4) = 0.0;
  Jpose(1, 5) = Y;
  Jpose(1, 6) = 0.0;
  Jpose(1, 7) = 0.0;
  Jpose(1, 8) = 0.0;
  Jpose(1, 9) = Z;
  Jpose(1, 10) = 0.0;
  Jpose(1, 11) = 0.0;
  Jpose(1, 12) = 0.0;
  Jpose(1, 13) = 1.0;
  Jpose(1, 14) = 0.0;
  Jpose(1, 15) = 0.0;

  Jpose(2, 0) = 0.0;
  Jpose(2, 1) = 0.0;
  Jpose(2, 2) = X;
  Jpose(2, 3) = 0.0;
  Jpose(2, 4) = 0.0;
  Jpose(2, 5) = 0.0;
  Jpose(2, 6) = Y;
  Jpose(2, 7) = 0.0;
  Jpose(2, 8) = 0.0;
  Jpose(2, 9) = 0.0;
  Jpose(2, 10) = Z;
  Jpose(2, 11) = 0.0;
  Jpose(2, 12) = 0.0;
  Jpose(2, 13) = 0.0;
  Jpose(2, 14) = 1.0;
  Jpose(2, 15) = 0.0;

  return true;
}
