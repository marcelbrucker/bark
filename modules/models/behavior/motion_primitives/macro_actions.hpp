// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_MACRO_ACTIONS_HPP_
#define MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_MACRO_ACTIONS_HPP_

#include "modules/models/behavior/motion_primitives/motion_primitives.hpp"
#include "modules/models/behavior/motion_primitives/primitives.hpp"

namespace modules {
namespace models {
namespace behavior {

class BehaviorMPMacroActions : public BehaviorMotionPrimitives {
 public:
  BehaviorMPMacroActions(const DynamicModelPtr& dynamic_model,
                         const commons::ParamsPtr& params)
      : BehaviorMotionPrimitives(dynamic_model, params) {}

  ~BehaviorMPMacroActions() override {}

  Trajectory Plan(float delta_time,
                  const ObservedWorld& observed_world) override;

  std::vector<MotionIdx> GetValidMotionPrimitives(
      const ObservedWorldPtr& observed_world) const override {
    MotionIdx count = 0;
    std::vector<MotionIdx> valid_mp;
    for (auto const& p : motion_primitives_) {
      if (p->IsPreConditionSatisfied(observed_world)) {
        valid_mp.emplace_back(count);
      }
      count++;
    }
    return valid_mp;
  }

  MotionIdx AddMotionPrimitive(const primitives::PrimitivePtr& primitive);

  virtual std::shared_ptr<BehaviorModel> Clone() const;

 private:
  std::vector<primitives::PrimitivePtr> motion_primitives_;
};

inline std::shared_ptr<BehaviorModel> BehaviorMPMacroActions::Clone() const {
  std::shared_ptr<BehaviorMPMacroActions> model_ptr =
      std::make_shared<BehaviorMPMacroActions>(dynamic_model_, this->GetParams());

  for(const auto& mp : motion_primitives_) {
    model_ptr->AddMotionPrimitive(mp->ClonePrimitive());
  }
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_MOTION_PRIMITIVES_MACRO_ACTIONS_HPP_
