#include "ExtrinsicEstimator.hpp"
#include "camera.hpp"
#include "lens.hpp"
#include "measure.hpp"
#include "triangle.hpp"
#include "p3p.hpp"
#include "vvs.hpp"
#include "pointGrid.hpp"
#include "measuresGrid.hpp"
#include <limits>
#include <iostream>

ExtrinsicEstimator::ExtrinsicEstimator() {

}

ExtrinsicEstimator::~ExtrinsicEstimator() {

}

bool ExtrinsicEstimator::estimate(const PointGrid & grid, std::shared_ptr<Camera> & camera) {

  std::vector<Eigen::Matrix4d> cameraTreferences;
  std::shared_ptr<Lens> lens = camera->getLens();

  std::array<double, 3> sph[4];
  double bestcost;
  bool res;
  Eigen::Matrix4d bestpose;

  bestcost = std::numeric_limits<double>::max();
  bestpose.setIdentity();

  std::vector<std::shared_ptr<Frame> > frames = camera->getKinematicChain();
  if (frames.size() == 0) return false;

  MeasuresGrid mgrid(camera, grid);

  /* For each possible quadrangles of non zero size */
  for (int i1 = 0; i1 < grid.getGridHeight(); i1++) {

    for (int i2 = i1 + 1; i2 < grid.getGridHeight(); i2++) {

      for (int pos1 = 0; pos1 < grid.getGridWidth() - 1; pos1++) {

        std::shared_ptr<Measure> mes1 = mgrid.at(i1, pos1);
        std::shared_ptr<Measure> mes2 = mgrid.at(i2, pos1);

        res = lens->liftToUnitSphere(sph[0], mes1);
        if (!res) {
          continue;
        }

        res = lens->liftToUnitSphere(sph[1], mes2);
        if (!res) {
          continue;
        }

        Point p1 = *mes1->getPoint();
        Point p2 = *mes2->getPoint();


        for (int pos2 = pos1 + 1; pos2 < grid.getGridWidth(); pos2++) {

          std::shared_ptr<Measure> mes3 = mgrid.at(i1, pos2);
          std::shared_ptr<Measure> mes4 = mgrid.at(i2, pos2);

          res = lens->liftToUnitSphere(sph[2], mes3);
          if (!res) {
            continue;
          }

          res = lens->liftToUnitSphere(sph[3], mes4);
          if (!res) {
            continue;
          }

          Point p3 = *mes3->getPoint();
          Point p4 = *mes4->getPoint();

          Triangle ref(p1, p2, p3);

          /*Estimate pose using p3p*/
          bool res = P3P::estimate(cameraTreferences, ref, sph[0][0], sph[0][1], sph[0][2], sph[1][0], sph[1][1], sph[1][2], sph[2][0], sph[2][1], sph[2][2]);
          if (!res) {
            continue;
          }

          /*We have at most 4 poses, retrieve the good one using the 4th point*/
          double mindiff = std::numeric_limits<double>::max();
          int mindiffidx = -1;
          for (int idpose = 0; idpose < cameraTreferences.size(); idpose++) {

            frames[0]->setPose(cameraTreferences[idpose]);
            std::array<double,2> est = camera->project(p4);

            double dx = est[0] - mes4->getCoords()[0];
            double dy = est[1] - mes4->getCoords()[1];
            double diff = sqrt(dx*dx+dy*dy);

            if (diff < mindiff) {
              mindiff = diff;
              mindiffidx = idpose;
            }
          }

          //Keep the best one from the possible set of points
          if (mindiffidx < 0) continue;

          frames[0]->setPose(cameraTreferences[mindiffidx]);
          double cost = camera->computeCurrentCost();
          if (cost < bestcost) {
            bestcost = cost;
            bestpose = cameraTreferences[mindiffidx];
          }
        }
      }
    }
  }

  if (bestcost == std::numeric_limits<double>::max()) {
    return false;
  }

  /*Refinment using non linear estimation in 2D*/
  if (!estimateVVS(bestpose, camera->getMeasuresList())) {
    return false;
  }

  frames[0]->setPose(bestpose);

  return true;
}
