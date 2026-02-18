#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include "nav_msgs/msg/odometry.hpp"

#include "behaviortree_cpp_v3/condition_node.h"

#include <string>

class AtRobotPoseNode: public BT::ConditionNode {
	public:
		explicit AtRobotPoseNode(const std::string &name,
														const BT::NodeConfiguration &config,
														rclcpp::Node::SharedPtr node_ptr);

		BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() {
        return BT::PortsList {BT::OutputPort<double>("x"),
                              BT::InputPort<double>("y")};
        }

		void check_pose(const nav_msgs::msg::Odometry::SharedPtr msg);

	private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_pose_subscriber_ptr_;
		bool correct_pose_;
};
