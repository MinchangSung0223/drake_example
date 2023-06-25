#pragma once

#include "drake/common/drake_copyable.h"
#include "lcmt_indy7_status.hpp"
#include "indy7_constants.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace indy7 {

/// Creates and outputs lcmt_indy7_status messages.
///
/// Note that this system does not actually send the message to an LCM
/// channel. To send the message, the output of this system should be
/// connected to a systems::lcm::LcmPublisherSystem::Make<lcmt_indy7_status>().
///
/// This system has many vector-valued input ports.  Most input ports are of
/// size num_joints + num_fingers. The exception is `time_measured` which is
/// the one-dimensional time in seconds to set as the message timestamp
/// (i.e. the time inputted will be converted to microseconds and sent to the
/// hardware). It is optional and if unset, the context time is used.  The
/// elements in the ports are the joints of the arm from the base to the tip,
/// followed by the fingers in the same order as used by the Kinova SDK
/// (consult the URDF model for a visual example).  If the torque,
/// torque_external, or current input ports are not connected, the output
/// message will use zeros.  Finger velocities will be translated to the
/// values used by the Kinova SDK from values appropriate for the finger
/// joints in the Indy7 description (see indy7_constants.h).
///
/// This system has one abstract-valued output port of type lcmt_indy7_status.
///
/// This system is presently only used in simulation. The robot hardware drivers
/// publish directly to LCM and do not make use of this system.
///
/// @system
/// name: Indy7StatusSender
/// input_ports:
/// - position
/// - velocity
/// - torque (optional)
/// - torque_external (optional)
/// - current (optional)
/// - time_measured (optional)
/// output_ports:
/// - lcmt_indy7_status
/// @endsystem
///
/// @see `lcmt_indy7_status.lcm` for additional documentation.
class Indy7StatusSender : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Indy7StatusSender)

  Indy7StatusSender(int num_joints = kIndy7DefaultArmNumJoints,
                   int num_fingers = kIndy7DefaultArmNumFingers);

  /// @name Named accessors for this System's input and output ports.
  //@{
  const systems::InputPort<double>& get_time_measured_input_port() const {
    return *time_measured_input_;
  }
  const systems::InputPort<double>& get_position_input_port() const {
    return *position_input_;
  }
  const systems::InputPort<double>& get_velocity_input_port() const {
    return *velocity_input_;
  }
  const systems::InputPort<double>& get_torque_input_port() const {
    return *torque_input_;
  }
  const systems::InputPort<double>& get_torque_external_input_port() const {
    return *torque_external_input_;
  }
  const systems::InputPort<double>& get_current_input_port() const {
    return *current_input_;
  }
  //@}

 private:
  void CalcOutput(const systems::Context<double>&, lcmt_indy7_status*) const;

  const int num_joints_;
  const int num_fingers_;
  const systems::InputPort<double>* time_measured_input_{};
  const systems::InputPort<double>* position_input_{};
  const systems::InputPort<double>* velocity_input_{};
  const systems::InputPort<double>* torque_input_{};
  const systems::InputPort<double>* torque_external_input_{};
  const systems::InputPort<double>* current_input_{};
};

}  // namespace indy7
}  // namespace manipulation
}  // namespace drake