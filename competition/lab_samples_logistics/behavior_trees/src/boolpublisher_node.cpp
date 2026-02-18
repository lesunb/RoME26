#include "boolpublisher_node.h"


BoolPublisherNode::BoolPublisherNode(const std::string &xml_tag_name,
									 const BT::NodeConfiguration &conf,
									 rclcpp::Node::SharedPtr node_ptr): BT::SyncActionNode(xml_tag_name, conf){
    node_ptr_ = node_ptr;
}

BT::NodeStatus BoolPublisherNode::tick(){
    bool value = getInput<bool>("value").value();

	std::string topic = getInput<std::string>("topic").value();
    bool_publisher_ptr_ = node_ptr_->create_publisher<std_msgs::msg::Bool>(topic, 10);

    int times_to_send = 1;
	std_msgs::msg::Bool bool_msg;
	bool_msg.data = value;

    for(int i=0; i<times_to_send; i++){
        bool_publisher_ptr_->publish(bool_msg);
        sleep(1);
    }
    return BT::NodeStatus::SUCCESS;
}

