// Author: Tucker Haydon

#include "map3d.h"

namespace game_engine {
  const Polyhedron& Map3D::Boundary() const {
    return this->boundary_;
  } 

  const std::vector<Polyhedron>& Map3D::Obstacles() const {
    return this->obstacles_;
  }

  bool Map3D::Contains(const Point3D& point) const {
    return this->boundary_.Contains(point);
  }

  Plane3D Map3D::Ground() const {
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

  std::vector<Plane3D> Map3D::Walls() const {
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

  bool Map3D::IsFreeSpace(const Point3D& point) const {
    for(const Polyhedron& obstacle: this->obstacles_)  {
      if(true == obstacle.Contains(point)) {
        return false;
      }
    }
    return true;
  }

  Map3D Map3D::Inflate(const double distance) const {
    std::cerr << "Map3D::Inflate not yet implemented!" << std::endl;
    std::exit(EXIT_FAILURE);

    return *this;
  }

  std::vector<std::pair<double, double>> Map3D::Extents() const {
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
