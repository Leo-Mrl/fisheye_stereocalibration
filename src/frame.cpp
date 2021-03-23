#include "frame.hpp"

Frame::Frame() : pose(posedata.data()) {
  pose.setIdentity();
}

Frame::~Frame() {
}

void Frame::setPose(const Eigen::Matrix4d & SE3) {

  pose = SE3;
}

Eigen::Matrix4d Frame::getPose() {
  return pose;
}

Eigen::Matrix<double, 6, 1> Frame::getPoseVector() {
  Eigen::Matrix3d R = pose.block<3,3>(0,0);
  Eigen::AngleAxisd aa(R);

  Eigen::Matrix<double, 6, 1> ret;
  Eigen::Vector3d axis = aa.axis();
  double angle = aa.angle();

  ret(0, 0) = axis(0) * angle;
  ret(1, 0) = axis(1) * angle;
  ret(2, 0) = axis(2) * angle;
  ret(3, 0) = pose(0, 3);
  ret(4, 0) = pose(1, 3);
  ret(5, 0) = pose(2, 3);

  return ret;
}

double * Frame::getPoseParametersPtr() {
  return posedata.data();
}

void Frame::setPoseParameters(const double * vals) {
  for (int i = 0; i < 16; i++) {
    posedata[i] = vals[i];
  }
}

Point Frame::transform(const Point & p) {
  return p.transform(pose);
}

Point Frame::transformInverse(const Point & p) {
  return p.transform(pose.inverse());
}
