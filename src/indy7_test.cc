

#include <gflags/gflags.h>

#include <drake/common/text_logging.h>
#include <limits>
#include <memory>

#include <gflags/gflags.h>

#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "indy7_command_receiver.h"
#include "indy7_constants.h"
#include "indy7_status_sender.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/visualization/visualization_config_functions.h"
using Eigen::VectorXd;

using drake::geometry::SceneGraph;
using drake::lcm::DrakeLcm;
using drake::manipulation::indy7::Indy7CommandReceiver;
using drake::manipulation::indy7::Indy7StatusSender;
using drake::manipulation::indy7::kIndy7DefaultArmNumJoints;
using drake::manipulation::indy7::kIndy7DefaultArmNumFingers;
using drake::manipulation::indy7::kIndy7LcmStatusPeriod;
using drake::multibody::Body;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::controllers::InverseDynamicsController;
using drake::systems::Demultiplexer;
using drake::visualization::AddDefaultVisualization;

const char kUrdfPath[] ="../models/indy7/indy7.urdf";
    // "package://drake/manipulation/models/jaco_description/urdf/"
    // "j2s7s300_sphere_collision.urdf";

namespace drake {
namespace examples {
namespace hello {
DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");
DEFINE_double(realtime_rate, 1.0, "");
DEFINE_double(time_step, 1e-3,
              "The time step to use for MultibodyPlant model "
              "discretization.  0 uses the continuous version of the plant.");

void DoMain() {
  drake::log()->info("Indy7 Simulation Start");
  drake::log()->info("INPUT PARAMS --time_step :{} ",FLAGS_time_step);


  systems::DiagramBuilder<double> builder;
  auto [indy7_plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, FLAGS_time_step);
  const multibody::ModelInstanceIndex indy7_id =
      Parser(&indy7_plant).AddModelFromFile(kUrdfPath);
  indy7_plant.WeldFrames(indy7_plant.world_frame(),
                        indy7_plant.GetFrameByName("base_link"));
  indy7_plant.Finalize();
  const int num_positions = indy7_plant.num_positions();
  drake::log()->info("num_positions :{} ",num_positions);  
  VectorXd kp = VectorXd::Constant(num_positions, 100);
  VectorXd kd = 2.0 * kp.array().sqrt();
  VectorXd ki = VectorXd::Zero(num_positions);

  auto indy7_controller = builder.AddSystem<InverseDynamicsController>(
      indy7_plant, kp, ki, kd, false);
  builder.Connect(indy7_plant.get_state_output_port(indy7_id),
                  indy7_controller->get_input_port_estimated_state());
  AddDefaultVisualization(&builder);

   systems::lcm::LcmInterfaceSystem* lcm =
       builder.AddSystem<systems::lcm::LcmInterfaceSystem>();
   auto command_sub = builder.AddSystem(
    systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_indy7_command>(
           "INDY7_COMMAND", lcm));
   auto command_receiver = builder.AddSystem<Indy7CommandReceiver>();
   builder.Connect(command_sub->get_output_port(),
                   command_receiver->get_message_input_port());
 auto mux = builder.AddSystem<systems::Multiplexer>(
      std::vector<int>({kIndy7DefaultArmNumJoints + kIndy7DefaultArmNumFingers,
              kIndy7DefaultArmNumJoints + kIndy7DefaultArmNumFingers}));
  builder.Connect(command_receiver->get_commanded_position_output_port(),
                  mux->get_input_port(0));
  builder.Connect(command_receiver->get_commanded_velocity_output_port(),
                  mux->get_input_port(1));
  builder.Connect(mux->get_output_port(),
                  indy7_controller->get_input_port_desired_state());
  builder.Connect(indy7_controller->get_output_port_control(),
                  indy7_plant.get_actuation_input_port(indy7_id));

  auto status_pub =
      builder.AddSystem(
          systems::lcm::LcmPublisherSystem::Make<drake::lcmt_indy7_status>(
          "INDY7_STATUS", lcm, kIndy7LcmStatusPeriod));
  // TODO(sammy-tri) populate joint torque (and external torques).  External
  // torques might want to wait until after #12631 is fixed or it could slow
  // down the simulation significantly.
  auto status_sender = builder.AddSystem<Indy7StatusSender>();
  auto demux = builder.AddSystem<systems::Demultiplexer>(
      std::vector<int>({kIndy7DefaultArmNumJoints + kIndy7DefaultArmNumFingers,
              kIndy7DefaultArmNumJoints + kIndy7DefaultArmNumFingers}));
  builder.Connect(indy7_plant.get_state_output_port(indy7_id),
                  demux->get_input_port());
  builder.Connect(demux->get_output_port(0),
                  status_sender->get_position_input_port());
  builder.Connect(demux->get_output_port(0),
                  command_receiver->get_position_measured_input_port());
  builder.Connect(demux->get_output_port(1),
                  status_sender->get_velocity_input_port());
  builder.Connect(status_sender->get_output_port(),
                  status_pub->get_input_port());

  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  systems::Context<double>& root_context = simulator.get_mutable_context();

  // Set the initial position to something similar to where the indy7 moves to
  // when starting teleop.
  VectorXd initial_position = VectorXd::Zero(num_positions);
  initial_position(0) = 0;
  initial_position(1) = -1.5707;
  initial_position(2) = 0;
  initial_position(3) = 0;
  initial_position(4) = 0;
  initial_position(5) = 0;

  indy7_plant.SetPositions(
      &diagram->GetMutableSubsystemContext(indy7_plant, &root_context),
      initial_position);

  simulator.Initialize();
  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_realtime_rate);
  simulator.AdvanceTo(FLAGS_simulation_sec);


}

}  // namespace hello
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple hello Drake example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::hello::DoMain();
  return 0;
}