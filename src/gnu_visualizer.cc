// Author: Tucker Haydon

#include "gnu_visualizer.h"
#include "cpp_plot.h"

namespace path_planning {
  namespace {
    void DrawGrid(const CppPlot::Plot& plt,
                  const OccupancyGrid& occupancy_grid) {
    }

    void DrawPath(const CppPlot::Plot& plt, 
                  const std::vector<Node>& path) {

    }
  }

  GNUVisualizer::Run(const OccupancyGrid& occupancy_grid,
                     const std::vector<Node>& path) {
    CppPlot::Plot plt;

    // # color definitions
    // set border linewidth 1.5
    // set style line 1 lc rgb '#0060ad' lt 1 lw 2 pt 7 pi -1 ps 1.5
    // set pointintervalbox 3
    // 
    // # Set grid on
    // set grid
    // 
    // # Turn key off
    // unset key
    // 
    // # set ytics 1
    // # set tics scale 0.75
    // 
    // # Set plot bounds
    // set xrange [0:5]
    // set yrange [0:5]
    // 
    // # Draw rectangle
    // # set object 1 rect from 0,1 to 1,2 fc lt 2 front
    // set obj rect from 1, graph 0 to 2, graph 1
    // 
    // # Draw path
    // plot 'data' with linespoints ls 1

    // Set plot bounds
    plt.Forward("set xrange [0:5]")
    plt.Forward("set yrange [0:5]")

    // Turn grid on
    plt.Forward("set grid")


    DrawGrid(plt, occupancy_grid);
    DrawPath(plt, path);
  }

}
