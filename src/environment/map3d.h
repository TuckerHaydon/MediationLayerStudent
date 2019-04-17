// Author: Tucker Haydon

#pragma once 

#include <vector>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <utility>

#include "polyhedron.h"
#include "yaml-cpp/yaml.h"

namespace mediation_layer {
  class Map3D {
    private:
      Polyhedron boundary_;
      std::vector<Polyhedron> obstacles_;
      friend class YAML::convert<Map3D>;

    public:
      Map3D(const Polyhedron& boundary = Polyhedron(),
            const std::vector<Polyhedron>& obstacles = {})
        : boundary_(boundary),
          obstacles_(obstacles) {}

      const Polyhedron& Boundary() const;

      const std::vector<Polyhedron>& Obstacles() const;

      bool Contains(const Point3D& point) const;
      bool IsFreeSpace(const Point3D& point) const;

      // Determines the extents of the map. Returns a list of tuples that
      // contain the {min,max} coordinates for the XYZ dimensions.
      std::vector<std::pair<double, double>> Extents() const;

      // Returns the plane with the smallest average z-coordinate
      Plane3D Ground() const;

      // Returns the list of planes minus the ground plane
      std::vector<Plane3D> Walls() const;

      // Inflates a map by a set distance. Map boundaries are shrunk and
      // obstacles are expanded. Shrinking and expanding affects both x and y
      // directions equally, therefore object aspect ratios are not guaranteed
      // to stay the same.
      Map3D Inflate(const double distance) const;
  };

  //  ******************
  //  * IMPLEMENTATION *
  //  ******************
  inline const Polyhedron& Map3D::Boundary() const {
    return this->boundary_;
  } 

  inline const std::vector<Polyhedron>& Map3D::Obstacles() const {
    return this->obstacles_;
  }

  inline bool Map3D::Contains(const Point3D& point) const {
    return this->boundary_.Contains(point);
  }

  inline Plane3D Map3D::Ground() const {
    size_t min_idx;
    double min_z = std::numeric_limits<double>::max();
    const std::vector<Plane3D>& faces = this->boundary_.Faces();
    for(size_t idx = 0; idx < faces.size(); ++idx) {
      double z = 0;
      for(const Line3D edge: faces[idx].Edges()) {
        z+=edge.Start().z();
      }
      z /= faces[idx].Edges().size();

      if(z < min_z) {
        min_idx = idx;
        min_z = z;
      } 
    }

    return faces[min_idx];
  }

  inline std::vector<Plane3D> Map3D::Walls() const {
    size_t min_idx;
    double min_z = std::numeric_limits<double>::max();
    std::vector<Plane3D> faces = this->boundary_.Faces();
    for(size_t idx = 0; idx < faces.size(); ++idx) {
      double z = 0;
      for(const Line3D edge: faces[idx].Edges()) {
        z+=edge.Start().z();
      }
      z /= faces[idx].Edges().size();

      if(z < min_z) {
        min_idx = idx;
        min_z = z;
      } 
    }

    faces.erase(faces.begin() + min_idx);
    return faces;

  }

  inline bool Map3D::IsFreeSpace(const Point3D& point) const {
    for(const Polyhedron& obstacle: this->obstacles_)  {
      if(true == obstacle.Contains(point)) {
        return false;
      }
    }
    return true;
  }

  inline Map3D Map3D::Inflate(const double distance) const {
    std::cerr << "Map3D::Inflate not yet implemented!" << std::endl;
    std::exit(EXIT_FAILURE);

    return *this;
  }

  inline std::vector<std::pair<double, double>> Map3D::Extents() const {
    double 
      min_x{std::numeric_limits<double>::max()}, max_x{-std::numeric_limits<double>::max()},
      min_y{std::numeric_limits<double>::max()}, max_y{-std::numeric_limits<double>::max()},
      min_z{std::numeric_limits<double>::max()}, max_z{-std::numeric_limits<double>::max()};

    for(const Plane3D& face: this->boundary_.Faces()) {
      for(const Line3D& edge: face.Edges()) {
        const std::vector<Point3D> vertices = {edge.Start(), edge.End()};
        for(const Point3D& vertex: vertices) {
          if(vertex.x() < min_x) { min_x = vertex.x(); }
          if(vertex.y() < min_y) { min_y = vertex.y(); }
          if(vertex.z() < min_z) { min_z = vertex.z(); }
          if(vertex.x() > max_x) { max_x = vertex.x(); }
          if(vertex.y() > max_y) { max_y = vertex.y(); }
          if(vertex.z() > max_z) { max_z = vertex.z(); }
        }
      }
    }

    return {
      std::make_pair(min_x, max_x),
      std::make_pair(min_y, max_y),
      std::make_pair(min_z, max_z)
    };
  }
}

namespace YAML {
template<>
struct convert<mediation_layer::Map3D> {
  static Node encode(const mediation_layer::Map3D& rhs) {
    Node node;
    node["boundary"] = rhs.boundary_;
    node["obstacles"] = rhs.obstacles_;
    return node;
  }

  static bool decode(const Node& node, mediation_layer::Map3D& rhs) {
    if(!node.IsMap() || !node["boundary"]) {
      return false;
    }

    rhs.boundary_ = node["boundary"].as<mediation_layer::Polyhedron>();

    if(node["obstacles"]) {
      rhs.obstacles_ = node["obstacles"].as<std::vector<mediation_layer::Polyhedron>>();
    }

    return true;
  }
};
}
