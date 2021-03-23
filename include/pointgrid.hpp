#ifndef POINTGRID_HPP_
#define POINTGRID_HPP_


#include <memory>
#include <vector>
#include "config.hpp"

class Point;

class VS_EXPORT PointGrid
{
private:
  double squareDimension;
  std::vector<std::shared_ptr<Point> > grid;
  int nbCornerWidth;
  int nbCornerHeight;

public:
  PointGrid(double squareDimension, int nbCornerWidth, int nbCornerHeight);
  virtual ~PointGrid();

  const std::shared_ptr<Point> getPoint(int i, int j) const;
  const size_t getGridWidth() const;
  const size_t getGridHeight() const;
  const double getSquareDimension() const;
};

#endif
