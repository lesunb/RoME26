#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"

#include <string>
#include <fstream>
#include <chrono>


class BTExecutor : public rclcpp::Node
{
public:
  explicit BTExecutor(const std::string &node_name);
  void setup();
  void create_behavior_tree();
  void update_behavior_tree();
  void halt_callback(const std_msgs::msg::Bool::SharedPtr new_msg);
  void halt_behavior_tree();
	void node_logger();

  void register_nav2_plugins();

private:
  BT::BehaviorTreeFactory factory_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr str_publisher_ptr_;
	rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr end_publisher_ptr_;

	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr halt_bt_sub_ptr_;
	std::vector<BT::TreeNode::StatusChangeSubscriber> status_change_;
  BT::Tree tree_;
};
