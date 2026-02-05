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
                                     const abb::egm::wrapper::Input& inputs,
                                     double* z)
{
  abb::egm::wrapper::Robot* robot = p_output->mutable_robot();
  abb::egm::wrapper::CartesianSpace* cartesian = robot->mutable_cartesian();
  abb::egm::wrapper::CartesianPose* pose = cartesian->mutable_pose();

  const auto& feedback_pose = inputs.feedback().robot().cartesian().pose();
  if (feedback_pose.position().z() > 1000.0) {
    *z += 1.0;
    pose->mutable_position()->set_x(776.0);
    pose->mutable_position()->set_y(-16.0);
    pose->mutable_position()->set_z(1120.07 - *z);
  } else {
    return false;
  }

  pose->mutable_euler()->set_x(180.0);
  pose->mutable_euler()->set_y(0.0);
  pose->mutable_euler()->set_z(-90.0);

  return true;
}

}  // namespace egm
}  // namespace abb

double sign(double x) {
  return x / std::abs(x);
}

bool moveToPoint(double x, double y, double z, double rx, double ry, double rz,
                 abb::egm::wrapper::Output* p_output,
                 const abb::egm::wrapper::Input& inputs,
                 double dlin,
                 double dang,
                 double tolerance_length = 1.0,
                 double tolerance_angle = 0.5)
{
  abb::egm::wrapper::Robot* robot = p_output->mutable_robot();
  abb::egm::wrapper::CartesianSpace* cartesian = robot->mutable_cartesian();
  abb::egm::wrapper::CartesianPose* pose = cartesian->mutable_pose();

  double dx = x - inputs.feedback().robot().cartesian().pose().position().x();
  double dy = y - inputs.feedback().robot().cartesian().pose().position().y();
  double dz = z - inputs.feedback().robot().cartesian().pose().position().z();
  double drx = rx - inputs.feedback().robot().cartesian().pose().euler().x();
  double dry = ry - inputs.feedback().robot().cartesian().pose().euler().y();
  double drz = rz - inputs.feedback().robot().cartesian().pose().euler().z();

  if (std::abs(dx) > tolerance_length ||
      std::abs(dy) > tolerance_length ||
      std::abs(dz) > tolerance_length ||
      std::abs(drx) > tolerance_angle ||
      std::abs(dry) > tolerance_angle ||
      std::abs(drz) > tolerance_angle) {
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

/*
 * -----------------------------------------------------------------------------
 * Алгоритм дискретизированного движения к целевой точке (set = следующая цель)
 * -----------------------------------------------------------------------------
 *
 * Роботу нельзя подавать сразу целевую точку (x,y,z,rx,ry,rz) — он не успевает.
 * Нужно разбивать путь на маленькие шаги и в каждом цикле EGM отправлять
 * следующую промежуточную цель (set).
 *
 * Шаги алгоритма в каждом цикле (например в while после interface.read):
 *
 * 1. Целевая точка (финальная) задана: target = (x, y, z, rx, ry, rz).
 *
 * 2. Текущая поза робота берётся из feedback: current = (cx, cy, cz, crx, cry, crz).
 *
 * 3. Направление к цели:
 *    - по позиции:  dx = x - cx,  dy = y - cy,  dz = z - cz;
 *    - по углам:    drx = rx - crx, dry = ry - cry, drz = rz - crz.
 *
 * 4. Ограничение шага (дискретизация):
 *    - линейный шаг не больше dlin [мм]:
 *      dist_lin = sqrt(dx^2 + dy^2 + dz^2);
 *      если dist_lin > dlin, то (dx, dy, dz) масштабируем: *= dlin / dist_lin;
 *    - угловой шаг не больше dang [град]:
 *      dist_ang = sqrt(drx^2 + dry^2 + drz^2);
 *      если dist_ang > dang, то (drx, dry, drz) *= dang / dist_ang;
 *
 * 5. Следующая целевая точка (то, что подаём в set):
 *    next = current + (dx, dy, dz, drx, dry, drz)  // уже ограниченные шаги
 *    В output записываем: set_x(next_x), set_y(next_y), ... — именно следующую
 *    целевую точку, а не приращение.
 *
 * 6. Отправляем output роботу (interface.write). В следующем цикле current
 *    обновится по feedback, и мы снова вычислим next к той же цели target.
 *
 * 7. Критерий «достигли цели»: |x - cx| <= tol_lin и аналогично для y, z
 *    и для углов (tol_ang). Тогда можно переходить к следующей целевой точке
 *    (например из траектории) или завершать движение.
 *
 * Параметры:
 *   dlin   — макс. линейный шаг за один цикл [мм]; меньше = плавнее и медленнее.
 *   dang   — макс. угловой шаг за один цикл [град].
 *   tol_lin, tol_ang — допуски для признания «достигнуто».
 *
 * Реализация: функция stepTowardPoint() ниже выполняет шаги 2–6 и возвращает
 * true, когда цель достигнута (шаг 7).
 * -----------------------------------------------------------------------------
 */

/**
 * \brief Discretized move toward target pose: computes direction and limited
 *        step, then writes the next target point (current + step) into output,
 *        not the step vector itself.
 * \param x, y, z Final target position [mm].
 * \param rx, ry, rz Target orientation (Euler) [deg]; sent as-is every cycle (TCP
 *        orientation is not changed during move).
 * \param p_output EGM output to fill with next target point (pose).
 * \param inputs Current EGM feedback (robot state).
 * \param dlin Max linear step per call [mm].
 * \param dang Unused; kept for API compatibility.
 * \param tolerance_length Position tolerance to consider "reached" [mm].
 * \param tolerance_angle Unused; kept for API compatibility.
 * \return true when position reached within tolerance_length, false otherwise.
 */
bool stepTowardPoint(double x, double y, double z, double rx, double ry, double rz,
                     abb::egm::wrapper::Output* p_output,
                     const abb::egm::wrapper::Input& inputs,
                     double dlin,
                     double dang,
                     double tolerance_length = 0.1,
                     double tolerance_angle = 0.1)
{
  const auto& feedback = inputs.feedback().robot().cartesian().pose();
  double cx = feedback.position().x();
  double cy = feedback.position().y();
  double cz = feedback.position().z();

  double dx = x - cx;
  double dy = y - cy;
  double dz = z - cz;

  double dist_lin = std::sqrt(dx * dx + dy * dy + dz * dz);
  double scale_lin = 1.0;
  if (dist_lin > dlin && dist_lin > 1e-9)
  {
    scale_lin = dlin / dist_lin;
  }
  double next_target_x = cx + dx * scale_lin;
  double next_target_y = cy + dy * scale_lin;
  double next_target_z = cz + dz * scale_lin;

  abb::egm::wrapper::Robot* robot = p_output->mutable_robot();
  abb::egm::wrapper::CartesianSpace* cartesian = robot->mutable_cartesian();
  abb::egm::wrapper::CartesianPose* pose = cartesian->mutable_pose();
  pose->mutable_position()->set_x(next_target_x);
  pose->mutable_position()->set_y(next_target_y);
  pose->mutable_position()->set_z(next_target_z);
  pose->mutable_euler()->set_x(rx);
  pose->mutable_euler()->set_y(ry);
  pose->mutable_euler()->set_z(rz);

  bool reached_lin = (std::abs(dx) <= tolerance_length &&
                      std::abs(dy) <= tolerance_length &&
                      std::abs(dz) <= tolerance_length);
  return reached_lin;
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
  const int port = node->declare_parameter<int>("port", 6515);
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
  double rz0 = 90.0;

  double R = 200.0;
  double theta = 0.0;
  double dtheta = 0.05;

  double dlin = 15.0;
  double dang = 0.5;
  std::vector<double> d_vector{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // bool task_finished = false;

  std::vector<double> target_point{x0+R*std::cos(theta), y0+R*std::sin(theta), z0, rx0, ry0, rz0};
  
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
    // bool running = abb::egm::buildCartesianMoveOutput(&outputs, inputs, &z);

    // bool running = moveToPoint(target_point[0], target_point[1], target_point[2], target_point[3], target_point[4], target_point[5], &outputs, inputs);
    bool running = stepTowardPoint(target_point[0], target_point[1], target_point[2], target_point[3], target_point[4], target_point[5], &outputs, inputs, dlin, dang);
    if (running) {
      target_point = trajectoryGenerator(x0, y0, z0, rx0, ry0, rz0, R, &theta, dtheta);
        // RCLCPP_INFO(node->get_logger(), "Theta: %f", theta);
      // RCLCPP_INFO(node->get_logger(), "New target point");
      // RCLCPP_INFO(node->get_logger(), "Theta: %f", theta);
    }

    if (theta >= 2 * M_PI)
    // if (!running)
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
