#include "indy7_command_sender.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace manipulation {
namespace indy7 {

using drake::systems::kVectorValued;

Indy7CommandSender::Indy7CommandSender(int num_joints, int num_fingers)
    : num_joints_(num_joints),
      num_fingers_(num_fingers) {
  position_input_ = &DeclareInputPort(
      "position", kVectorValued, num_joints_ + num_fingers_);
  velocity_input_ = &DeclareInputPort(
      "velocity", kVectorValued, num_joints_ + num_fingers_);
  time_input_ = &DeclareInputPort("time", kVectorValued, 1);

  this->DeclareAbstractOutputPort(
      "lcmt_indy7_command", &Indy7CommandSender::CalcOutput);
}

void Indy7CommandSender::CalcOutput(
    const systems::Context<double>& context, lcmt_indy7_command* output) const {

  if (time_input_->HasValue(context)) {
    output->utime = time_input_->Eval(context)[0] * 1e6;
  } else {
    output->utime = context.get_time() * 1e6;
  }

  output->num_joints = num_joints_;
  output->joint_position.resize(num_joints_);
  output->joint_velocity.resize(num_joints_);

  output->num_fingers = num_fingers_;
  output->finger_position.resize(num_fingers_);
  output->finger_velocity.resize(num_fingers_);

  const auto& position = position_input_->Eval(context);
  const auto& velocity = velocity_input_->Eval(context);
  for (int i = 0; i < num_joints_; ++i) {
    output->joint_position[i] = position(i);
    output->joint_velocity[i] = velocity(i);
  }

  for (int i = 0; i < num_fingers_; ++i) {
    output->finger_position[i] =
        position(num_joints_ + i) * kFingerUrdfToSdk;
    output->finger_velocity[i] =
        velocity(num_joints_ + i) * kFingerUrdfToSdk;
  }
}

}  // namespace indy7
}  // namespace manipulation
}  // namespace drake