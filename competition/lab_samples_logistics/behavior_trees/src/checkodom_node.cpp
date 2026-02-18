#include "checkodom_node.h"


CheckOdomNode::CheckOdomNode(const std::string &xml_tag_name,
                             const BT::NodeConfiguration &conf,
                             rclcpp::Node::SharedPtr node_ptr): BT::ConditionNode(xml_tag_name, conf){
	node_ptr_ = node_ptr;
}

bool CheckOdomNode::check_pose(std::string target){
		auto robot_pose = msg_.pose.pose.position;
		double target_x;
		double target_y;
		double tolerance = 0.5;

		if (target == "nurse_room"){
			target_x = 7.0;
			target_y = -2.0;
		} else if (target == "laboratory") {
			target_x = -3.0;
			target_y = 6.0;
		} else {
			target_x = getInput<double>("x").value();
			target_y = getInput<double>("y").value();
		}
		return (std::abs(robot_pose.x - target_x) <= tolerance && std::abs(robot_pose.y - target_y) <= tolerance);
}

void CheckOdomNode::update_pose(const nav_msgs::msg::Odometry::SharedPtr new_msg){
    msg_ = *new_msg;
}

// Waits for given duration or until message is received
bool CheckOdomNode::wait_for_message(int wait_duration){
    rclcpp::WaitSet wait_set;
    wait_set.add_subscription(robot_pose_subscriber_ptr_);
    auto wait_result = wait_set.wait(std::chrono::seconds(wait_duration));
    if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
				nav_msgs::msg::Odometry msg;
        rclcpp::MessageInfo info;
        robot_pose_subscriber_ptr_->take(msg, info);
				msg_ = msg;
        return true;
    }
    return false;
}

BT::NodeStatus CheckOdomNode::tick(){
    auto qos = rclcpp::SystemDefaultsQoS();
    qos.best_effort();
		std::string topic = "odom";
		std::string target = getInput<std::string>("target").value();
    robot_pose_subscriber_ptr_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(topic, qos,
                                                                               std::bind(&CheckOdomNode::update_pose, this, std::placeholders::_1));

    auto wait_duration_sec = getInput<int>("wait_duration_sec");
    int wait_duration = wait_duration_sec.value();
    bool received_msg = wait_for_message(wait_duration);

    if (!received_msg)
        return BT::NodeStatus::FAILURE;

    
		if (check_pose(target)){
			return BT::NodeStatus::SUCCESS;
		} else
			return BT::NodeStatus::FAILURE;
}
