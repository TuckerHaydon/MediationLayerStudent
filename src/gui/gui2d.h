// Author: Tucker Haydon

#ifndef PATH_PLANNING_GUI_GUI2D_H
#define PATH_PLANNING_GUI_GUI2D_H

#include <vector>
#include <string>
#include <utility>

#include "gnuplot-iostream.h"
#include "map2d.h"
#include "occupancy_grid2d.h"
#include "point2d.h"
#include "polygon.h"

namespace path_planning {

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
      bool LoadOccupancyGrid(const OccupancyGrid2D& occupancy_grid);
      bool LoadPath(const std::vector<geometry::Point2D>& path);
      bool LoadSafeFlightCorridor(const std::vector<geometry::Polygon>& safe_flight_corridor);
      bool Display();

    private:

      Gnuplot gp_;
      std::vector<CommandUnit> command_units_;

  };
}

#endif
