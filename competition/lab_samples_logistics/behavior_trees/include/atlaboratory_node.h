#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include "nav_msgs/msg/odometry.hpp"

#include "behaviortree_cpp_v3/condition_node.h"

#include <string>

class AtLaboratoryNode: public BT::ConditionNode {
	public:
		explicit AtLaboratoryNode(const std::string &name,
														const BT::NodeConfiguration &config,
														rclcpp::Node::SharedPtr node_ptr);

		BT::NodeStatus tick() override;

		void check_pose(const nav_msgs::msg::Odometry::SharedPtr msg);

	private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_pose_subscriber_ptr_;
		bool correct_pose_;
};
