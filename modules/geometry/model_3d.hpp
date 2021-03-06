// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_GEOMETRY_MODEL_3D_HPP_
#define MODULES_GEOMETRY_MODEL_3D_HPP_

#include "modules/geometry/polygon.hpp"

namespace modules {
namespace geometry {

class Model3D {
 public:
  typedef enum Type {
    NONE = 0,
    ROAD = 1,
    LIMOUSINE = 2,
    PEDESTRIAN = 3
  } Type;

  Model3D() : scale_x_(0.0),
              scale_y_(0.0),
              scale_z_(0.0),
              type_(NONE) {}

  explicit Model3D(Type type) : scale_x_(0.0),
                                scale_y_(0.0),
                                scale_z_(0.0),
                                type_(type) {}
  ~Model3D() {}

  Type GetType() { return type_; }
  void UpdateScale(const Polygon bounding_box) {}

 private:
  float scale_x_;
  float scale_y_;
  float scale_z_;
  Type type_;
};

}  // namespace geometry
}  // namespace modules

#endif  // MODULES_GEOMETRY_MODEL_3D_HPP_
