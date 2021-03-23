#include "measure.hpp"
#include "lens.hpp"
#include "camera.hpp"

Measure::Measure(std::shared_ptr<Camera> camera, std::shared_ptr<Point> point, double u, double v) {
  this->camera = camera;
  this->point = point;
  this->coords[0] = u;
  this->coords[1] = v;
}

std::array<double, 2> Measure::getCoords() {
  return this->coords;
}

std::shared_ptr<Point> Measure::getPoint() {
  return this->point;
}

std::shared_ptr<Camera> Measure::getCamera() {
  return this->camera.lock();
}

void Measure::setPoint(std::shared_ptr<Point> point) {
  this->point = point;
}

ceres::CostFunction * Measure::createIntrinsicsCostFunction() {
  return this->getCamera()->getLens()->createIntrinsicsCostFunction(this);
}

ceres::CostFunction * Measure::createExtrinsicsCostFunction() {
  return this->getCamera()->getLens()->createExtrinsicsCostFunction(this);
}
