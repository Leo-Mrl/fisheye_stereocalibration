#include "refinementEstimator.hpp"

//#define MAX_LOG_LEVEL 0

#include "camera.hpp"
#include "lens.hpp"
#include "point.hpp"
#include "measure.hpp"
#include "pointGrid.hpp"

#include <map>
#include <ceres/ceres.h>
#include <iomanip>
#include <csignal>

#include "pointGrid.hpp"
#include "LensFishEye.hpp"

class SE3Parameterization : public ceres::LocalParameterization {
 public:

  typedef Eigen::Transform<double, 3, Eigen::Affine> MatrixSE3;

  SE3Parameterization(bool disableTranslations = false) : m_disableTranslations(disableTranslations)
  {
  }

  virtual ~SE3Parameterization()
  {
  }

  virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const {

    double * ptrBase = (double*)x;
    double * ptrResult = (double*)x_plus_delta;
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > pose(ptrBase);
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > poseResult(ptrResult);

    Eigen::Vector3d axis;
    axis(0) = delta[0];
    axis(1) = delta[1];
    axis(2) = delta[2];
    double angle = axis.norm();
    axis.normalize();

    Eigen::Vector3d translation;
    translation(0) = delta[3];
    translation(1) = delta[4];
    translation(2) = delta[5];

    Eigen::AngleAxisd aa(angle, axis);
    MatrixSE3 T;
    T.matrix().block<3,3>(0,0) = aa.toRotationMatrix();
    T.matrix().block<3,1>(0,3) = translation;

    poseResult = T.matrix() * pose;


    return true;
  }

  virtual bool ComputeJacobian(const double* x, double* jacobian) const {

    // double * ptrCurrentPose = (double*)x;
    // Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor> > pose(ptrCurrentPose);

    double * row[16];
    for (int i = 0; i < 16; i++) {
      row[i] = &jacobian[i * 6];
      for (int j = 0; j < 6; j++) {
        row[i][j] = 0;
      }
    }

    row[1][2] = 1;
    row[2][1] = -1;
    row[6][0] = 1;
    row[8][1] = 1;
    row[9][0] = -1;
    // if translations are not disabled, set up the Jacobian for translations
    // not sure about where the translations are : 12, 13, 14 in the original code, why not 3, 7 and 11 if the jabobians are row-major ???
    // update: does not work with 3, 7 and 11
    if ( ! m_disableTranslations ) {
      row[12][3] = 1;
      row[13][4] = 1;
      row[14][5] = 1;
    }
    row[4][2] = -1;


    return true;
  }

  virtual int GlobalSize() const {
    return 16;
  }

  virtual int LocalSize() const {
    return 6;
  }

private:
  bool m_disableTranslations;
};


RefinementEstimator::RefinementEstimator() {

}

RefinementEstimator::~RefinementEstimator() {

}

bool RefinementEstimator::estimate(PointGrid & grid, std::vector<std::shared_ptr<Camera> > & views) {

  ceres::Problem problem;

  for (int i = 0; i < grid.getGridHeight(); i++) {
    for (int j = 0; j < grid.getGridWidth(); j++) {
      const std::shared_ptr<Point> pt = grid.getPoint(i, j);
      problem.AddParameterBlock(pt->getPointParametersPtr(), 3);
      problem.SetParameterBlockConstant(pt->getPointParametersPtr());
    }
  }

  std::vector<std::shared_ptr<Camera> >::iterator it;
  for (it = views.begin(); it != views.end(); it++) {

    std::shared_ptr<Camera> camera = *it;
    std::vector<std::shared_ptr<Frame> > frames = camera->getKinematicChain();
    if (frames.size() == 0) return false;

    problem.AddParameterBlock(frames[0]->getPoseParametersPtr(), 16);
    problem.SetParameterization(frames[0]->getPoseParametersPtr(), new SE3Parameterization);

    std::vector<std::shared_ptr<Measure> > measures = camera->getMeasuresList();

    for (std::vector<std::shared_ptr<Measure> >::iterator itmes = measures.begin(); itmes != measures.end(); itmes++) {
      std::shared_ptr<Measure> mes = *itmes;
      double * ppt = mes->getPoint()->getPointParametersPtr();
      double * pcam = frames[0]->getPoseParametersPtr();
      double * plens = camera->getLens()->getGenericParametersPtr();
      double * plensadd = camera->getLens()->getAdditionalParametersPtr();

      ceres::CostFunction * cost = mes->createIntrinsicsCostFunction();
      problem.AddResidualBlock(cost, nullptr, ppt, pcam, plens, plensadd);
    }
  }

  ceres::Solver::Options options;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.use_nonmonotonic_steps = true;
  options.max_num_iterations = 1000;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.num_threads = 1;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  return true;
}

bool RefinementEstimator::estimateExtrinsics(PointGrid & grid, std::vector<std::shared_ptr<Camera> > & views, bool disableTranslations) {

  ceres::Problem problem;

  std::map<double*, bool> map;

  std::vector<std::shared_ptr<Camera> >::iterator it;
  for (it = views.begin(); it != views.end(); it++) {

    std::shared_ptr<Camera> camera = *it;

    std::vector<std::shared_ptr<Frame> > frames = camera->getKinematicChain();
    if (frames.size() != 3) return false;

    problem.AddParameterBlock(frames[0]->getPoseParametersPtr(), 16);
    if (map.find(frames[0]->getPoseParametersPtr()) == map.end()) {
      problem.SetParameterization(frames[0]->getPoseParametersPtr(), new SE3Parameterization(false));
      map[frames[0]->getPoseParametersPtr()] = true;
      //problem.SetParameterBlockConstant(frames[0]->getPoseParametersPtr());
    }

    problem.AddParameterBlock(frames[1]->getPoseParametersPtr(), 16);
    if (map.find(frames[1]->getPoseParametersPtr()) == map.end()) {
      problem.SetParameterization(frames[1]->getPoseParametersPtr(), new SE3Parameterization(false));
      map[frames[1]->getPoseParametersPtr()] = true;
      //problem.SetParameterBlockConstant(frames[1]->getPoseParametersPtr());
    }

    problem.AddParameterBlock(frames[2]->getPoseParametersPtr(), 16);
    if (map.find(frames[2]->getPoseParametersPtr()) == map.end()) {
      problem.SetParameterization(frames[2]->getPoseParametersPtr(), new SE3Parameterization(disableTranslations));
      map[frames[2]->getPoseParametersPtr()] = true;
    }

    std::vector<std::shared_ptr<Measure> > measures = camera->getMeasuresList();

    for (std::vector<std::shared_ptr<Measure> >::iterator itmes = measures.begin(); itmes != measures.end(); itmes++) {
      std::shared_ptr<Measure> mes = *itmes;

      double * pcam1 = frames[0]->getPoseParametersPtr();
      double * pcam2 = frames[1]->getPoseParametersPtr();
      double * pcam3 = frames[2]->getPoseParametersPtr();

      // set the translations of the rig-to-sensor frame to zero 
      if (disableTranslations) {
        pcam3[3] = pcam3[7] = pcam3[11] = 0.;
      }

      ceres::CostFunction * cost = mes->createExtrinsicsCostFunction();
      problem.AddResidualBlock(cost, nullptr, pcam1, pcam2, pcam3);
    }
  }

  ceres::Solver::Options options;
  options.trust_region_strategy_type = ceres::DOGLEG;
  options.use_nonmonotonic_steps = true;
  options.max_num_iterations = 500;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.num_threads = 1;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << "\n";

  return true;
}
