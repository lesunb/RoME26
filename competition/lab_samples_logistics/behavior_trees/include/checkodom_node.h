#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include "nav_msgs/msg/odometry.hpp"

#include "behaviortree_cpp_v3/condition_node.h"

#include <string>

class CheckOdomNode: public BT::ConditionNode {
	public:
		explicit CheckOdomNode(const std::string &name,
																const BT::NodeConfiguration &config,
																rclcpp::Node::SharedPtr node_ptr);

		BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() {
			return BT::PortsList {BT::InputPort<std::string>("target"),
														BT::InputPort<double>("x"),
														BT::InputPort<double>("y"),
                            BT::InputPort<int>("wait_duration_sec")};
        }

		void update_pose(const nav_msgs::msg::Odometry::SharedPtr msg);

		bool check_pose(std::string target);

		bool wait_for_message(int wait_duration);

	private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_pose_subscriber_ptr_;
		rclcpp::Node::SharedPtr node_ptr_;
		nav_msgs::msg::Odometry msg_;
		bool correct_pose_;
};
