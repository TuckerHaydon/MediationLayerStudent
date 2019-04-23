// Author: Tucker Haydon

#pragma once 

#include "types.h"

namespace mediation_layer {
  // Implementation of a 3D line
  class Line3D {
    private:
      // Start point
      Point3D start_;

      // End point
      Point3D end_;

      // Forward-declare parser
      friend class YAML::convert<Line3D>;

    public:
      // Constructor
      Line3D(const Point3D& start = Point3D(), 
             const Point3D& end = Point3D())
        : start_(start),
          end_(end) {}

      // Start point accessor
      const Point3D& Start() const;

      // End point accessor
      const Point3D& End() const;

      // Expresses the line as a 3D vector
      Vec3D AsVector() const;

  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline const Point3D& Line3D::Start() const {
    return this->start_;
  }

  inline const Point3D& Line3D::End() const {
    return this->end_;
  }

  inline Point3D Line3D::AsVector() const {
    return this->end_ - this->start_;
  }
}

namespace YAML {
  template<>
  struct convert<mediation_layer::Line3D> {
    static Node encode(const mediation_layer::Line3D& rhs) {
      Node node;
      node.push_back(rhs.start_.x());
      node.push_back(rhs.start_.y());
      node.push_back(rhs.start_.z());
      node.push_back(rhs.end_.x());
      node.push_back(rhs.end_.y());
      node.push_back(rhs.end_.z());
      return node;
    }
  
    static bool decode(const Node& node, mediation_layer::Line3D& rhs) {
      if(!node.IsSequence() || node.size() != 6) {
        return false;
      }
  
      rhs.start_.x() = node[0].as<double>();
      rhs.start_.y() = node[1].as<double>();
      rhs.start_.z() = node[2].as<double>();
      rhs.end_.x()   = node[3].as<double>();
      rhs.end_.y()   = node[4].as<double>();
      rhs.end_.z()   = node[5].as<double>();
      return true;
    }
  };
}
