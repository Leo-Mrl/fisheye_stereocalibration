#include "pointgrid.hpp"
#include "point.hpp"
#include <iostream>

PointGrid::PointGrid(double squareDimension, int nbCornerWidth, int nbCornerHeight) {

  this->squareDimension = squareDimension;
  this->nbCornerWidth = (nbCornerWidth > nbCornerHeight) ? nbCornerWidth : nbCornerHeight;
  this->nbCornerHeight = (nbCornerWidth > nbCornerHeight) ? nbCornerHeight : nbCornerWidth;

  for (int i = 0; i < nbCornerHeight; i++) {
    for (int j = 0; j < nbCornerWidth; j++) {

      double ri = i;
      double rj = j;

      /*Point grid is finally just a holder for points*/
      Point * p = new Point(squareDimension * rj, squareDimension * ri);
      grid.push_back(std::shared_ptr<Point>(p));
    }
  }
}

PointGrid::~PointGrid() {
}

const std::shared_ptr<Point> PointGrid::getPoint(int i, int j) const {
  return grid[i * nbCornerWidth + j];
}

const size_t PointGrid::getGridWidth() const {
  return nbCornerWidth;
}

const size_t PointGrid::getGridHeight() const {
  return nbCornerHeight;
}

const double PointGrid::getSquareDimension() const {
  return squareDimension;
}
