#ifndef CAMERA_HPP_
#define CAMERA_HPP_

#include <array>
#include <memory>
#include <vector>

#include "frame.hpp"
#include "point.hpp"
#include "lens.hpp"
#include "measure.hpp"

#include <Eigen/Geometry>
#include <iostream>

class Measure;
class Lens;
class Point;

/**
Defines a camera viewpoint.
A camera defines the camera parameters (both extrinsics through the pose and intrinsics through the lens).
Each camera has its own extrinsics chain but shares its intrinsics with other "camera"
*/
class VS_EXPORT Camera {

public:
  /**Vector of measures type*/
  typedef std::vector<std::shared_ptr<Measure> > MeasuresVector;

public:
  /**
  Constructor with lens. A camera without a lens is not possible.
  @param lens the lens to use
  */
  Camera(std::shared_ptr<Lens> lens);

  /**
  Set the unique id
  @param id the new unique id
  */
  void setId(unsigned int id);

  /**
  Get the unique id
  @return id the unique id
  */
  unsigned int getId();

  /**
  Retrieve a link to the lens used
  */
  std::shared_ptr<Lens> getLens();

  /**
  Retrieve the kinematic chain
  @return the kinematic chain
  */
  std::vector<std::shared_ptr<Frame> > getKinematicChain();

  /**
  Retrieve the first pose
  @return the default pose
  */
  std::shared_ptr<Frame> getFirstFrame();

  /**
  Remove all frames
  */
  void clearFrames();

  /**
  Add a frame to the kinematic chain
  p' = framen *  framen-1 * ... * frame0 * p
  @param frame the frame to add at the end of the kinematic chain
  */
  void addFrame(std::shared_ptr<Frame> & frame);

  /**
  Add a measure to the Camera
  @param measure new measure to add
  */
  void addMeasure(std::shared_ptr<Measure> & mes);

  /**
  Project a point on the camera image plane
  @param p point to project
  @param inpixels flag to decide if the projected coordinates are transformed to pixel space
  @return the projected coordinates
  */
  std::array<double, 2> project(const Point & p, bool inpixels = true);

  /**
  Compute the projection error (SSD) of the points with respect to the measured coordinates.
  @param variance the result variance
  @return a global score for all measures of this camera
  */
  double computeCurrentCost(double * variance = nullptr);

  /**
  Return a copy of the measures list
  @return list of measures
  */
  MeasuresVector getMeasuresList() {
    return measures_list;
  }

  /**
  Return an iterator of the measures list
  @return list of measures iterator
  */
  MeasuresVector::iterator measures_begin() {
    return measures_list.begin();
  }

  /**
  Return an iterator of the measures list
  @return list of measures iterator
  */
  MeasuresVector::iterator measures_end() {
    return measures_list.end();
  }

  /**
  Compute jacobian of the kinematic total chain wrt a pose of the chain
  @param J the result jacobian
  @param poseindex the index of the chain element
  */
  bool jacobianChainWrtPose(Eigen::Matrix<double, 16, 16> & J, unsigned int poseIndex);

  /**
  Compute jacobian of the transformed point wrt kinematic total chain
  @param J the result jacobian
  @param point the point in the reference frame
  */
  bool jacobianPointWrtPose(Eigen::Matrix<double, 3, 16> & J, const Point & point);

protected:
  /**
  A link to the lens.
  By construction, lens can not be nullptr
  */
  std::shared_ptr<Lens> lens;

  /**
  A list of measures links
  */
  MeasuresVector measures_list;

  /**
  Kinematic chain (in hierarchical order)
  */
  std::vector<std::shared_ptr<Frame> > frames;

  /**
  Camera unique id (used when many cameras are in fact the same object)
  */
  unsigned int uid;
};

#endif
