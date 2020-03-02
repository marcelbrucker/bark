// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_CONTINUOUS_ACTIONS_HPP_
#define MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_CONTINUOUS_ACTIONS_HPP_

#include "modules/models/behavior/motion_primitives/motion_primitives.hpp"

#include <numeric>

namespace modules {
namespace models {
namespace behavior {

class BehaviorMPContinuousActions : public BehaviorMotionPrimitives {
 public:
  BehaviorMPContinuousActions(const DynamicModelPtr& dynamic_model,
                             const commons::ParamsPtr& params)
      : BehaviorMotionPrimitives(dynamic_model, params), motion_primitives_() {}

  ~BehaviorMPContinuousActions() override {}

  Trajectory Plan(float delta_time, const ObservedWorld& observed_world) override;

  std::vector<MotionIdx> GetValidMotionPrimitives(const ObservedWorldPtr& observed_world) const override {
    std::vector<MotionIdx> valid_mp(motion_primitives_.size());
    std::iota(valid_mp.begin(), valid_mp.end(), 0);
    return valid_mp;
  }
  virtual Input GetAction() const { return motion_primitives_[active_motion_]; }
  MotionIdx AddMotionPrimitive(const Input& dynamic_input);

  virtual std::shared_ptr<BehaviorModel> Clone() const;

 private:
  std::vector<Input> motion_primitives_;
};

inline std::shared_ptr<BehaviorModel> BehaviorMPContinuousActions::Clone()
    const {
  std::shared_ptr<BehaviorMPContinuousActions> model_ptr =
      std::make_shared<BehaviorMPContinuousActions>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_CONTINUOUS_ACTIONS_HPP_
