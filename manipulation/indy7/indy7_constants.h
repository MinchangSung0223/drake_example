#pragma once

namespace drake {
namespace manipulation {
namespace indy7 {

/// The LCM system classes for the Indy7 default to a 7dof model with 3
/// fingers.  Different configurations are supported by passing the
/// proper arguments to the system constructors.
constexpr int kIndy7DefaultArmNumJoints = 6;
constexpr int kIndy7DefaultArmNumFingers = 0;

/// The Indy7 URDF models the fingers as having a single revolute joint, but
/// the SDK uses the motor position for the actuator driving the adaptive
/// fingers.  This doesn't really convert well, so we use a scaling factor to
/// approximate a reasonable behavior for simulation/visualization.
constexpr double kFingerSdkToUrdf = 1.34 / 118.68;
constexpr double kFingerUrdfToSdk = 1. / kFingerSdkToUrdf;

/// Kinova says 100Hz is the proper frequency for joint velocity
/// updates.  See
/// https://github.com/Kinovarobotics/kinova-ros#velocity-control-for-joint-space-and-cartesian-space
constexpr double kIndy7LcmStatusPeriod = 0.010;

}  // namespace indy7
}  // namespace manipulation
}  // namespace drake