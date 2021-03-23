#include "vvs.hpp"
#include "lens.hpp"

#include "camera.hpp"
#include "point.hpp"

bool estimateVVS(Eigen::Matrix4d & pose, const std::vector<std::shared_ptr<Measure> > & measures) {
  if (measures.size() < 4) return false;

  double sum;
  double J[3][6];
  double M[3][3];
  Eigen::Matrix<double, 6, 6> JtJ;
  Eigen::Matrix<double, 6, 1> Jtd;
  Eigen::Matrix<double, 6, 1> sol;

  std::shared_ptr<Lens> lens = measures[0]->getCamera()->getLens();

  for (int iter = 0; iter < 10; iter++) {

    JtJ.fill(0.0);
    Jtd.fill(0.0);
    sum = 0.0;

    for (int id = 0; id < measures.size(); id++) {
      /*Get 3D coordinates of point*/
      double oX = measures[id]->getPoint()->getX();
      double oY = measures[id]->getPoint()->getY();
      double oZ = measures[id]->getPoint()->getZ();

      /*Transform this 3D point in the camera frame*/
      double cX = pose(0, 0) * oX + pose(0, 1) * oY + pose(0, 2) * oZ + pose(0, 3);
      double cY = pose(1, 0) * oX + pose(1, 1) * oY + pose(1, 2) * oZ + pose(1, 3);
      double cZ = pose(2, 0) * oX + pose(2, 1) * oY + pose(2, 2) * oZ + pose(2, 3);

      /*Estimate spherical coordinates*/
      double length = sqrt(cX * cX + cY * cY + cZ * cZ);
      double cx = cX / length;
      double cy = cY / length;
      double cz = cZ / length;

      /*Transform measure to unit sphere*/
      std::array<double, 3> sph;
      lens->liftToUnitSphere(sph, measures[id]);

      double ex = sph[0] - cx;
      double ey = sph[1] - cy;
      double ez = sph[2] - cz;
      sum += sqrt(ex*ex+ey*ey+ez*ez);

      /*Jacobian estimation*/

      double sig1 = length * length * length;
      double invsig1 = 1.0 / sig1;
      double cX2 = cX * cX;
      double cY2 = cY * cY;
      double cZ2 = cZ * cZ;
      double cXcY = cX * cY;
      double cXcZ = cX * cZ;
      double cYcZ = cY * cZ;

      M[0][0] = (cY2 + cZ2) * invsig1;
      M[0][1] = (-cXcY) * invsig1;
      M[0][2] = (-cXcZ) * invsig1;
      M[1][0] = (-cXcY) * invsig1;
      M[1][1] = (cX2 + cZ2) * invsig1;
      M[1][2] = (-cYcZ) * invsig1;
      M[2][0] = (-cXcZ) * invsig1;
      M[2][1] = (-cYcZ) * invsig1;
      M[2][2] = (cX2 + cY2) * invsig1;

      J[0][0] = -cZ * M[0][1] + cY * M[0][2];
      J[0][1] = cZ * M[0][0] - cX * M[0][2];
      J[0][2] = -cY * M[0][0] + cX * M[0][1];
      J[0][3] = M[0][0];
      J[0][4] = M[0][1];
      J[0][5] = M[0][2];

      J[1][0] = -cZ * M[1][1] + cY * M[1][2];
      J[1][1] = cZ * M[1][0] - cX * M[1][2];
      J[1][2] = -cY * M[1][0] + cX * M[1][1];
      J[1][3] = M[1][0];
      J[1][4] = M[1][1];
      J[1][5] = M[1][2];

      J[2][0] = -cZ * M[2][1] + cY * M[2][2];
      J[2][1] = cZ * M[2][0] - cX * M[2][2];
      J[2][2] = -cY * M[2][0] + cX * M[2][1];
      J[2][3] = M[2][0];
      J[2][4] = M[2][1];
      J[2][5] = M[2][2];

      for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
          JtJ(i,j) += J[0][i] * J[0][j] + J[1][i] * J[1][j] + J[2][i] * J[2][j];
        }
        Jtd(i,0) += J[0][i] * ex + J[1][i] * ey + J[2][i] * ez;
      }
    }

    /*x=(J^+)*err*/
    sol = -(JtJ.inverse() * Jtd);

    Eigen::Vector3d axis;
    axis(0) = sol(0);
    axis(1) = sol(1);
    axis(2) = sol(2);
    double angle = axis.norm();
    axis.normalize();

    /*x -> pose*/
    Eigen::Vector3d translation;
    translation(0) = sol(3);
    translation(1) = sol(4);
    translation(2) = sol(5);

    Eigen::AngleAxisd aa(angle, axis);
    Eigen::Matrix4d T;
    T.setIdentity();
    T.block<3,3>(0,0) = aa.toRotationMatrix();
    T.block<3,1>(0,3) = translation;

    pose = T.inverse() * pose;
  }


  return true;
}
