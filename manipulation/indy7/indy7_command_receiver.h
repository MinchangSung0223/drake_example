#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "lcmt_indy7_command.hpp"
#include "indy7_constants.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace indy7 {

/// Handles lcmt_indy7_command message from a LcmSubscriberSystem.
///
/// Note that this system does not actually subscribe to an LCM channel. To
/// receive the message, the input of this system should be connected to a
/// LcmSubscriberSystem::Make<drake::lcmt_indy7_command>().
///
/// It has one required input port, "lcmt_indy7_command".
///
/// This system has three output ports: one each for the commanded position
/// and velocity of the arm+finger joints, and one for the timestamp in the
/// most recently received message.  Finger velocities will be translated from
/// the values used by the Kinova SDK to values appropriate for the finger
/// joints in the Indy7 description (see indy7_constants.h).
///
/// @system
/// name: Indy7CommandReceiver
/// input_ports:
/// - lcmt_indy7_command
/// - position_measured (optional)
/// output_ports:
/// - position
/// - velocity
/// - time
///
/// @par Output prior to receiving a valid lcmt_indy7_command message: The
/// "position" output initially feeds through from the "position_measured"
/// input port -- or if not connected, outputs zero.  When discrete update
/// events are enabled (e.g., during a simulation), the system latches the
/// "position_measured" input into state during the first event, and the
/// "position" output comes from the latched state, no longer fed through from
/// the "position" input.  Alternatively, the LatchInitialPosition() method is
/// available to achieve the same effect without using events. The "time"
/// output will be a vector of a single zero.
///
/// @endsystem
class Indy7CommandReceiver : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Indy7CommandReceiver)

  Indy7CommandReceiver(int num_joints = kIndy7DefaultArmNumJoints,
                      int num_fingers = kIndy7DefaultArmNumFingers);

  /// (Advanced) Copies the current "position_measured" input (or zero if not
  /// connected) into Context state, and changes the behavior of the "position"
  /// output to produce the latched state if no message has been received yet.
  /// The latching already happens automatically during the first discrete
  /// update event (e.g., when using a Simulator); this method exists for use
  /// when not already using a Simulator or other special cases.
  void LatchInitialPosition(systems::Context<double>* context) const;

  /// @name Named accessors for this System's input and output ports.
  //@{
  const systems::InputPort<double>& get_message_input_port() const {
    return *message_input_;
  }
  const systems::InputPort<double>& get_position_measured_input_port() const {
    return *position_measured_input_;
  }
  const systems::OutputPort<double>& get_commanded_position_output_port()
      const {
    return *commanded_position_output_;
  }
  const systems::OutputPort<double>& get_commanded_velocity_output_port()
      const {
    return *commanded_velocity_output_;
  }
  const systems::OutputPort<double>& get_time_output_port() const {
    return *time_output_;
  }
//@}

 private:
  void CalcInput(const systems::Context<double>&, lcmt_indy7_command*) const;

  void DoCalcNextUpdateTime(
      const systems::Context<double>&,
      systems::CompositeEventCollection<double>*, double*) const final;
  void CalcPositionMeasuredOrZero(
      const systems::Context<double>&, systems::BasicVector<double>*) const;

  // Copies the current "position measured" input (or zero if not connected)
  // into the @p result.
  void LatchInitialPosition(
      const systems::Context<double>&,
      systems::DiscreteValues<double>*) const;
  void CalcPositionOutput(
      const systems::Context<double>&, systems::BasicVector<double>*) const;
  void CalcVelocityOutput(
      const systems::Context<double>&, systems::BasicVector<double>*) const;
  void CalcTimeOutput(
      const systems::Context<double>&, systems::BasicVector<double>*) const;

  const int num_joints_;
  const int num_fingers_;
  const systems::InputPort<double>* message_input_{};
  const systems::InputPort<double>* position_measured_input_{};
  const systems::CacheEntry* position_measured_or_zero_{};
  systems::DiscreteStateIndex latched_position_measured_is_set_;
  systems::DiscreteStateIndex latched_position_measured_;
  const systems::CacheEntry* groomed_input_{};
  const systems::OutputPort<double>* commanded_position_output_{};
  const systems::OutputPort<double>* commanded_velocity_output_{};
  const systems::OutputPort<double>* time_output_{};
};

}  // namespace indy7
}  // namespace manipulation
}  // namespace drake