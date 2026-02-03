/**
 * EGM node: ROS 2 node that runs an EGM server (UDP) and executes a built-in
 * "one joint move" trajectory. Interfaces: abb_libegm (UDP + protobuf), rclcpp
 * (node, parameters). Robot controller is the UDP client; this node is the server.
 */

#include <boost/thread.hpp>

#include <rclcpp/rclcpp.hpp>

#include <abb_libegm/egm_common.h>
#include <abb_libegm/egm_controller_interface.h>

#include <abb_libegm/egm_wrapper.pb.h>

namespace abb
{
namespace egm
{

/**
 * \brief Build reference output for "one joint move" trajectory (ported from win_pkg).
 * Joint 5 moves from 90 to ~0 deg; joints 1-4 = 0, joint 6 = -90.
 * \param p_output output message to fill.
 * \param degrees current angle [deg]; incremented by 0.1 each call.
 * \return true if trajectory continues, false when degrees >= 90 (task finished).
 */
static bool buildOneJointMoveOutput(
  abb::egm::wrapper::Output* p_output,
  double* p_degrees)
{
  if (*p_degrees >= 90.0)
  {
    return false;
  }

  *p_degrees += 0.1;

  // EGM Output layout: robot -> joints -> position (6 values, degrees).
  // Only position references are sent; velocity_outputs are disabled in config.
  abb::egm::wrapper::Robot* robot = p_output->mutable_robot();
  abb::egm::wrapper::JointSpace* joint_space = robot->mutable_joints();
  abb::egm::wrapper::Joints* position = joint_space->mutable_position();

  position->clear_values();
  position->add_values(0.0);
  position->add_values(0.0);
  position->add_values(0.0);
  position->add_values(0.0);
  position->add_values(90.0 - *p_degrees);
  position->add_values(-90.0);

  return true;
}

}  // namespace egm
}  // namespace abb

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("egm_node");

  // Port must match the EGM port configured on the robot controller (default 6510).
  const int port = node->declare_parameter<int>("port", 6510);
  const unsigned short port_number = static_cast<unsigned short>(port);

  abb::egm::BaseConfiguration config;
  config.axes = abb::egm::Six;
  config.use_demo_outputs = false;   // We send real joint targets, not demo data.
  config.use_velocity_outputs = false;  // Position-only; no velocity in Output.

  boost::asio::io_service io_service;
  abb::egm::EGMControllerInterface interface(io_service, port_number, config);

  if (!interface.isInitialized())
  {
    RCLCPP_ERROR(node->get_logger(), "EGM interface failed to initialize.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "EGM server listening on port %u", port_number);

  // Run io_service in a separate thread so main thread can block on waitForMessage.
  // The robot is the UDP client; it sends first. This thread handles receive/send I/O.
  boost::thread server_thread(boost::bind(&boost::asio::io_service::run, &io_service));

  abb::egm::wrapper::Input inputs;
  double degrees = 0.0;
  const unsigned int timeout_ms = 1000;  // Max wait for one EGM message from robot.

  while (rclcpp::ok())
  {
    // Blocks until one EGM packet from robot or timeout. If timeout, we just retry
    // (no log); first run may delay up to timeout_ms until robot starts sending.
    if (!interface.waitForMessage(timeout_ms))
    {
      continue;
    }

    interface.read(&inputs);

    abb::egm::wrapper::Output outputs;
    bool running = abb::egm::buildOneJointMoveOutput(&outputs, &degrees);

    if (!running)
    {
      RCLCPP_INFO(node->get_logger(), "EGM task finished!");
      break;
    }

    // Send target positions back to robot; EGM protocol expects reply after each input.
    interface.write(outputs);
  }

  // Stop the I/O thread so we can join it; required for clean shutdown.
  io_service.stop();
  if (server_thread.joinable())
  {
    server_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}
