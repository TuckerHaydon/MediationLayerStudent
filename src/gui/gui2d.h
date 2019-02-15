// Author: Tucker Haydon

#pragma once

#include <vector>
#include <string>
#include <utility>

#include "gnuplot-iostream.h"
#include "map2d.h"
#include "occupancy_grid2d.h"
#include "polygon.h"

namespace mediation_layer {

  class Gui2D {
    public:
      struct CommandUnit {
        std::vector<std::pair<double, double>> data_;
        std::string command_;

        CommandUnit(const std::string command, 
                    const std::vector<std::pair<double, double>> data)
          : data_(data),
            command_(command) {}
      };

      bool LoadMap(const Map2D& map);
      bool LoadOccupancyGrid(const OccupancyGrid2D* occupancy_grid);
      bool LoadPath(const std::vector<Point2D>& path);
      bool LoadSafeFlightCorridor(const std::vector<Polygon>& safe_flight_corridor);
      bool Display();

    private:

      Gnuplot gp_;
      std::vector<CommandUnit> command_units_;

  };
}
