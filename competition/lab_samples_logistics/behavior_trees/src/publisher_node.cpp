#include "publisher_node.h"


PublisherNode::PublisherNode(const std::string &xml_tag_name,
                             const BT::NodeConfiguration &conf,
                             rclcpp::Node::SharedPtr node_ptr): BT::SyncActionNode(xml_tag_name, conf){
		node_ptr_ = node_ptr;
}

BT::NodeStatus PublisherNode::tick(){
    std_msgs::msg::String str_msg;
    auto msg = getInput<std::string>("message");
    str_msg.data = msg.value();

		std::string topic = getInput<std::string>("topic").value();
    str_publisher_ptr_ = node_ptr_->create_publisher<std_msgs::msg::String>(topic, 10);

    int times_to_send = 1;

    for(int i=0; i<times_to_send; i++){
        str_publisher_ptr_->publish(str_msg);
        sleep(1);
    }
    return BT::NodeStatus::SUCCESS;
}

