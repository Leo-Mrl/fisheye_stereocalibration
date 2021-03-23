#ifndef __MEASURES_GRID_ITERATOR__
#define __MEASURES_GRID_ITERATOR__

class Camera;
class PointGrid;
class Measure;

#include <memory>
#include <vector>
#include "config.hpp"

class VS_EXPORT MeasuresGrid {
public:
  MeasuresGrid(std::shared_ptr<Camera> & cam, const PointGrid & grid);

  std::shared_ptr<Measure> at(unsigned int row, unsigned int col);

private:
  std::vector<std::shared_ptr<Measure> >::iterator measures_iterator;

  unsigned int width;
  unsigned int height;
};

#endif
