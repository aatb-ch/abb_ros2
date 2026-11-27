#include <rclcpp/rclcpp.hpp>

#include <abb_rws_client/rws_service_provider_ros.hpp>
#include <abb_rws_client/rws_state_publisher_ros.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr client_node = rclcpp::Node::make_shared("rws");

  client_node->declare_parameter("robot_nickname", std::string{});
  client_node->declare_parameter("no_connection_timeout", false);
  std::string robot_ip = client_node->declare_parameter<std::string>("robot_ip", "127.0.0.1");
  int robot_port = client_node->declare_parameter<int>("robot_port", 65535);

  client_node->get_parameter<std::string>("robot_ip", robot_ip);
  client_node->get_parameter<int>("robot_port", robot_port);

  // One RWS manager per role. The state publisher's poll holds the manager's
  // interface mutex for the whole RWS round trip, and every service only
  // try_locks it (RWSManager::verifyRWSManagerReady), so a shared manager makes
  // any service call that lands during a poll fail at once with "Server is
  // busy" (stop_egm included). Separate managers keep polling and services
  // independent; librws logs each manager's sessions out when it is destroyed,
  // i.e. on a clean shutdown (SIGINT/SIGTERM end spin()).
  auto make_manager = [&]() {
    return std::make_shared<abb::robot::RWSManager>(
        robot_ip, static_cast<unsigned short>(robot_port), abb::rws::SystemConstants::General::DEFAULT_USERNAME,
        abb::rws::SystemConstants::General::DEFAULT_PASSWORD);
  };
  abb_rws_client::RWSServiceProviderROS srv_provider(client_node, make_manager());
  abb_rws_client::RWSStatePublisherROS state_publisher(client_node, make_manager());

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(client_node);

  exec.spin();

  return 0;
}
