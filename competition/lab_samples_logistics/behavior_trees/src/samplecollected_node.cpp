#include "samplecollected_node.h"

SampleCollectedNode::SampleCollectedNode(const std::string &xml_tag_name,
																				const BT::NodeConfiguration &conf,
																				rclcpp::Node::SharedPtr node_ptr): BT::ConditionNode(xml_tag_name, conf){
	(void)node_ptr;
}

BT::NodeStatus SampleCollectedNode::tick(){
	auto sample = getInput<std::string>("sample");
	if(!sample)
		return BT::NodeStatus::FAILURE;
	else if(sample.value() == "success")
		return BT::NodeStatus::SUCCESS;

	return BT::NodeStatus::FAILURE;
}
