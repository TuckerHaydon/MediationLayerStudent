// Author: Tucker Haydon

#include <sstream>
#include <memory>
#include <chrono>
#include <thread>
#include <iostream>

#include "gnu_visualizer.h"
#include "cpp_plot.h"
#include "raii_data.h"

namespace path_planning {
  namespace {
    struct PathDataAdapter {
      const std::vector<Node>& path_;

      PathDataAdapter(const std::vector<Node>& path) 
        : path_(path) {}

      std::shared_ptr<CppPlot::RAIIData> operator()() {
        constexpr size_t SIZE_DOUBLE_BYTES = sizeof(double);

        const size_t size = 2 * path_.size() * SIZE_DOUBLE_BYTES;
        std::shared_ptr<CppPlot::RAIIData> data = std::make_shared<CppPlot::RAIIData>(size);

        for(size_t idx = 0; idx < path_.size(); ++idx) {
          reinterpret_cast<double*>(data->Data())[2*idx + 0] 
            = 0.5 + static_cast<double>(path_[idx].id_[1]);
          reinterpret_cast<double*>(data->Data())[2*idx + 1] 
            = 0.5 + static_cast<double>(path_[idx].id_[0]);  
        }

        return data;

      };
    };
  }

  void GNUVisualizer::Run(const OccupancyGrid& occupancy_grid,
                          const std::vector<Node>& path) {
    CppPlot::Plot plt;

    // Unset key
    plt.Forward("unset key");

    // Set plot bounds
    plt.Forward("set xrange [0:" + std::to_string(occupancy_grid.cols_) + "]");
    plt.Forward("set yrange [0:" + std::to_string(occupancy_grid.rows_) + "]");

    // Turn grid on
    plt.Forward("set grid");


    // Draw the occupancy grid
    // # set object 1 rect from 0,1 to 1,2 fc lt 2 front
    size_t counter = 1;
    for(size_t row = 0; row < occupancy_grid.rows_; ++row) {
      for(size_t col = 0; col < occupancy_grid.cols_; ++col) {
        if(occupancy_grid.occupancy_grid_[row][col]) {
          std::stringstream ss; 
          ss << "set object " << counter++ << " rectangle from " 
            << col << "," << row << " to " << col+1 << "," << row+1
            << " fillcolor rgb \"black\" fillstyle solid 1.0 front";
          plt.Forward(ss.str()); 
        }
      }
    }

    // Draw the path
    // # color definitions
    // set border linewidth 1.5
    // set style line 1 lc rgb '#0060ad' lt 1 lw 2 pt 7 pi -1 ps 1.5
    // set pointintervalbox 3
    plt.Forward("set border linewidth 1.5");
    plt.Forward("set style line 1 linecolor rgb '#0060ad' linetype 1 linewidth 2 pointtype 7 pointinterval -1 pointsize 1.5");
    plt.Forward("set pointintervalbox 3");

    // # Draw path
    // plot 'data' with linespoints ls 1
    plt.LoadData(PathDataAdapter(path)());
    const std::string filename = plt.GetFileName();
    plt.Forward("plot '" + filename + "' binary format='%double%double' using 1:2 with linespoints ls 1");
    std::this_thread::sleep_for (std::chrono::seconds(1));
  }

}
