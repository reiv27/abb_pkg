/**
 * EGM node: ROS 2 node that runs an EGM server (UDP) and executes a built-in
 * "one joint move" trajectory. Interfaces: abb_libegm (UDP + protobuf), rclcpp
 * (node, parameters). Robot controller is the UDP client; this node is the server.
 */

#include <boost/thread.hpp>
#include <cmath>
#include <cstdint>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <abb_libegm/egm_common.h>
#include <abb_libegm/egm_common_auxiliary.h>
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
static bool buildOneJointMoveOutput(abb::egm::wrapper::Output* p_output,
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

static bool buildCartesianMoveOutput(abb::egm::wrapper::Output* p_output,
                                     const abb::egm::wrapper::Input& inputs)
{
  abb::egm::wrapper::Robot* robot = p_output->mutable_robot();
  abb::egm::wrapper::CartesianSpace* cartesian = robot->mutable_cartesian();
  abb::egm::wrapper::CartesianPose* pose = cartesian->mutable_pose();

  const auto& feedback_pose = inputs.feedback().robot().cartesian().pose();
  if (std::abs(feedback_pose.position().z() - 1000.0) > 0.01) {
    pose->mutable_position()->set_x(770.0);
    pose->mutable_position()->set_y(0.0);
    pose->mutable_position()->set_z(1000.0);
  } else {
    return false;
  }

  pose->mutable_euler()->set_x(180.0);
  pose->mutable_euler()->set_y(0.0);
  pose->mutable_euler()->set_z(-90.0);

  // pose->mutable_quaternion()->set_u0(1.0);
  // pose->mutable_quaternion()->set_u1(0.0);
  // pose->mutable_quaternion()->set_u2(0.0);
  // pose->mutable_quaternion()->set_u3(0.0);

  // if (feedback_pose.has_euler()) {
  //   pose->mutable_euler()->CopyFrom(feedback_pose.euler());
  // } else if (feedback_pose.has_quaternion()) {
  //   pose->mutable_quaternion()->CopyFrom(feedback_pose.quaternion());
  // }

  return true;
}

}  // namespace egm
}  // namespace abb

bool moveToPoint(double x, double y, double z, double rx, double ry, double rz,
                 abb::egm::wrapper::Output* p_output,
                 const abb::egm::wrapper::Input& inputs,
                 double tolerance_length = 0.01,
                 double tolerance_angle = 0.01)
{
  abb::egm::wrapper::Robot* robot = p_output->mutable_robot();
  abb::egm::wrapper::CartesianSpace* cartesian = robot->mutable_cartesian();
  abb::egm::wrapper::CartesianPose* pose = cartesian->mutable_pose();

  if (std::abs(x - inputs.feedback().robot().cartesian().pose().position().x()) > tolerance_length ||
      std::abs(y - inputs.feedback().robot().cartesian().pose().position().y()) > tolerance_length ||
      std::abs(z - inputs.feedback().robot().cartesian().pose().position().z()) > tolerance_length ||
      std::abs(rx - inputs.feedback().robot().cartesian().pose().euler().x()) > tolerance_angle ||
      std::abs(ry - inputs.feedback().robot().cartesian().pose().euler().y()) > tolerance_angle ||
      std::abs(rz - inputs.feedback().robot().cartesian().pose().euler().z()) > tolerance_angle) {
    pose->mutable_position()->set_x(x);
    pose->mutable_position()->set_y(y);
    pose->mutable_position()->set_z(z);
    pose->mutable_euler()->set_x(rx);
    pose->mutable_euler()->set_y(ry);
    pose->mutable_euler()->set_z(rz);
    return false;
  }

  return true;
}

std::vector<double> trajectoryGenerator(
  double x0, double y0, double z0,
  double rx0, double ry0, double rz0,
  double R, double* theta, double dtheta)
{
  std::vector<double> result;
  result.reserve(6);

  *theta += dtheta;
  result.push_back(x0 + R * std::cos(*theta));
  result.push_back(y0 + R * std::sin(*theta));
  result.push_back(z0);
  result.push_back(rx0);
  result.push_back(ry0);
  result.push_back(rz0);

  return result;
}

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
  config.use_velocity_outputs = true;  // Position-only; no velocity in Output.

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
  const unsigned int timeout_ms = 1000;  // Max wait for one EGM message from robot.

  double x0 = 770.0;
  double y0 = 0.0;
  double z0 = 1000.0;
  double rx0 = 180.0;
  double ry0 = 0.0;
  double rz0 = -90.0;

  double R = 200.0;
  double theta = 0.0;
  double dtheta = 0.1;
  // bool task_finished = false;

  std::vector<double> target_point ={x0+R*std::cos(theta), y0+R*std::sin(theta), z0, rx0, ry0, rz0};
  
  while (rclcpp::ok())
  {
    // Blocks until one EGM packet from robot or timeout. If timeout, we just retry
    // (no log); first run may delay up to timeout_ms until robot starts sending.
    if (!interface.waitForMessage(timeout_ms))
    {
      continue;
    }

    interface.read(&inputs);
    // double x = inputs.feedback().robot().cartesian().pose().position().x();
    // double y = inputs.feedback().robot().cartesian().pose().position().y();
    // double z = inputs.feedback().robot().cartesian().pose().position().z();
    // double rx = inputs.feedback().robot().cartesian().pose().euler().x();
    // double ry = inputs.feedback().robot().cartesian().pose().euler().y();
    // double rz = inputs.feedback().robot().cartesian().pose().euler().z();
    
    abb::egm::wrapper::Output outputs;

    // bool running = abb::egm::buildOneJointMoveOutput(&outputs, &degrees);
    // bool running = abb::egm::buildCartesianMoveOutput(&outputs, inputs);

    bool running = moveToPoint(target_point[0], target_point[1], target_point[2], target_point[3], target_point[4], target_point[5], &outputs, inputs);
    if (running) {
      target_point = trajectoryGenerator(x0, y0, z0, rx0, ry0, rz0, R, &theta, dtheta);
      RCLCPP_INFO(node->get_logger(), "New target point");
      RCLCPP_INFO(node->get_logger(), "Theta: %f", theta);
    }

    if (theta >= 2 * M_PI)
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
