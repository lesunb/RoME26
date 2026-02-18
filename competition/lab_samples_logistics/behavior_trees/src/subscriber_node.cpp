#include "subscriber_node.h"


SubscriberNode::SubscriberNode(const std::string &xml_tag_name,
                             const BT::NodeConfiguration &conf,
                             rclcpp::Node::SharedPtr node_ptr): BT::ConditionNode(xml_tag_name, conf){
	node_ptr_ = node_ptr;
}

void SubscriberNode::update_msg(const std_msgs::msg::String::SharedPtr new_msg){
    msg_ = new_msg->data;
}

// Waits for given duration or until message is received
bool SubscriberNode::wait_for_message(int wait_duration){
    rclcpp::WaitSet wait_set;
    wait_set.add_subscription(str_subscriber_ptr_);
    auto wait_result = wait_set.wait(std::chrono::seconds(wait_duration));
    if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
        std_msgs::msg::String msg;
        rclcpp::MessageInfo info;
        str_subscriber_ptr_->take(msg, info);
				msg_ = msg.data;
        return true;
    }
    return false;
}

BT::NodeStatus SubscriberNode::tick(){
    auto qos = rclcpp::SystemDefaultsQoS();
    qos.best_effort();
		std::string topic = "/" + getInput<std::string>("topic").value(); //absolute namespace
    str_subscriber_ptr_ = node_ptr_->create_subscription<std_msgs::msg::String>(topic, qos,
                                                                               std::bind(&SubscriberNode::update_msg, this, std::placeholders::_1));

    auto wait_duration_sec = getInput<int>("wait_duration_sec");
    int wait_duration = wait_duration_sec.value();
    bool received_msg = wait_for_message(wait_duration);

    if (msg_.empty() || !received_msg)
        return BT::NodeStatus::FAILURE;

    setOutput("message", msg_);
		std::cout << "Received message: " << msg_ << std::endl;
    return BT::NodeStatus::SUCCESS;
}
