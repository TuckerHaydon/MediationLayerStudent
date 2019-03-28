// Author: Tucker Haydon

#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <memory>
#include <string>
#include <functional>

#include "trajectory.h"
#include "publisher_guard.h"

namespace mediation_layer {
  // Trajectory publisher acts as an adapter between the ROS ecosystem and the
  // internal mediation layer ecosystem. Transforms internal trajectory into a
  // ROS trajectory and publishes it.
  template <size_t T>
  class TrajectoryPublisherNode {
    private:
      // A publisher guard ensures that the Publish() function may be called in
      // a thread-safe manner
      std::shared_ptr<PublisherGuard<std_msgs::String>> publisher_guard_;
      
    public:
      // Constructor.
      TrajectoryPublisherNode(const std::string& topic);

      // Publishes the message
      void Publish(const Trajectory<T>& msg);
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  template <size_t T>
  inline TrajectoryPublisherNode<T>::TrajectoryPublisherNode(
      const std::string& topic) {
    this->publisher_guard_ = std::make_shared<PublisherGuard<std_msgs::String>>(topic);
  }

  template <size_t T>
  inline void TrajectoryPublisherNode<T>::Publish(const Trajectory<T>& msg) {
    std_msgs::String m;
    m.data = "";
    this->publisher_guard_->Publish(m);
  }

  using TrajectoryPublisherNode2D = TrajectoryPublisherNode<2>;
  using TrajectoryPublisherNode3D = TrajectoryPublisherNode<3>;

};
