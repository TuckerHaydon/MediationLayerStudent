// Author: Tucker Haydon

#include "trajectory_vetter.h"

#include <Eigen/Core>
#include <iostream>

namespace game_engine {

  bool TrajectoryVetter::Vet(
      const Trajectory& trajectory,
      const Map3D& map,
      const std::shared_ptr<QuadStateWarden> quad_state_warden,
      const std::string& quad_name
      ) const {

    const size_t trajectory_size = trajectory.Size();

    // Initial position constraints
    QuadState current_quad_state;
    quad_state_warden->Read(quad_name, current_quad_state);

    const Eigen::Vector3d initial_position = trajectory.Position(0);
    const Eigen::Vector3d current_position = current_quad_state.Position();
    if(this->options_.max_distance_from_current_position
        < (initial_position - current_position).norm()) {
      std::cerr 
        << "Specified trajectory start point "
        << initial_position.transpose() 
        << " deviates from the current quad position"
        << current_position.transpose()
        << " by more than the maximum distance of "
        << this->options_.max_distance_from_current_position
        << " meters" 
        << std::endl;
      return false;
    }

    // Position constraints 
    for(size_t idx = 0; idx < trajectory_size; ++idx) {
      const Eigen::Vector3d point = trajectory.Position(idx);
      if(false == map.Contains(point)) {
        std::cerr 
          << "Specified trajectory point " 
          << point.transpose() 
          << " exceeded map bounds" 
          << std::endl; 
        return false;
      }
      if(false == map.IsFreeSpace(point)) {
        std::cerr 
          << "Specified trajectory point " 
          << point.transpose() 
          << " is contained within an obstacle" 
          << std::endl; 
        return false;
      }   
    }

    // Velocity constraints
    for(size_t idx = 0; idx < trajectory_size; ++idx) {
      const Eigen::Vector3d vel = trajectory.Velocity(idx);
      if(this->options_.max_velocity_magnitude < vel.norm()) {
        std::cerr 
          << "Specified trajectory velocity " 
          << " exceeds maximum velocity constraint of "
          << this->options_.max_velocity_magnitude 
          << " m/s" 
          << std::endl; 
        return false;
      }   
    }

    // Mean value theorem for velocity constraints
    for(size_t idx = 0; idx < trajectory_size-1; ++idx) {
      const Eigen::Vector3d current_pos = trajectory.Position(idx);
      const Eigen::Vector3d next_pos = trajectory.Position(idx+1);
      const double current_time = trajectory.Time(idx);
      const double next_time = trajectory.Time(idx+1);
      const double mean_value_velocity 
        = ((next_pos - current_pos) / (next_time - current_time)).norm();
      if(this->options_.max_velocity_magnitude < mean_value_velocity) {
        std::cerr 
          << "Specified mean-value trajectory velocity " 
          << " exceeds maximum velocity constraint of "
          << this->options_.max_velocity_magnitude 
          << " m/s" 
          << std::endl; 
        return false;
      }   
    }


    // Acceleration constraints
    for(size_t idx = 0; idx < trajectory_size; ++idx) {
      const Eigen::Vector3d acc = trajectory.Acceleration(idx);
      if(this->options_.max_acceleration_magnitude < acc.norm()) {
        std::cerr 
          << "Specified trajectory acceleration " 
          << " exceeds maximum acceleration constraint of "
          << this->options_.max_acceleration_magnitude 
          << " m/s^2" 
          << std::endl; 
        std::cout << acc.transpose() << std::endl;
        std::cout << acc.norm() << std::endl;
        return false;
      }   
    }

    // Mean value theorem for acceleration constraints
    for(size_t idx = 0; idx < trajectory_size-1; ++idx) {
      const Eigen::Vector3d current_vel = trajectory.Velocity(idx);
      const Eigen::Vector3d next_vel = trajectory.Velocity(idx+1);
      const double current_time = trajectory.Time(idx);
      const double next_time = trajectory.Time(idx+1);
      const double mean_value_acceleration 
        = ((next_vel - current_vel) / (next_time - current_time)).norm();
      if(this->options_.max_acceleration_magnitude < mean_value_acceleration) {
        std::cerr 
          << "Specified mean-value trajectory acceleration " 
          << " exceeds maximum acceleration constraint of "
          << this->options_.max_acceleration_magnitude 
          << " m/s" 
          << std::endl; 
        return false;
      }   
    }

    // Time constraints
    for(size_t idx = 0; idx < trajectory_size-1; ++idx) {
      const double current_time = trajectory.Time(idx);
      const double next_time = trajectory.Time(idx+1);
      const double delta_time = next_time - current_time;

      if( true == (delta_time < 0.0) ) {
        std::cerr
          << "Trajectory timestamps must be monotonically increasing"
          << std::endl;
        return false;
      }

      if(this->options_.max_delta_t < delta_time) {
        std::cerr
          << "Time between adjacent trajectory samples "
          << " exceeds maximum time of "
          << this->options_.max_delta_t 
          << " seconds."
          << std::endl;
        return false;
      }
    }

    return true;
  }
}
