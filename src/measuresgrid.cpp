#include "measuresGrid.hpp"

#include "camera.hpp"
#include "pointgrid.hpp"
#include "measure.hpp"

#include <memory>

MeasuresGrid::MeasuresGrid(std::shared_ptr<Camera> & cam, const PointGrid & grid) {
  width = grid.getGridWidth();
  height = grid.getGridHeight();
  measures_iterator = cam->measures_begin();
}

std::shared_ptr<Measure> MeasuresGrid::at(unsigned int row, unsigned int col) {
  return *(measures_iterator + (row * width) + col);
}
