#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include "std_msgs/msg/bool.hpp"

#include "behaviortree_cpp_v3/action_node.h"

#include <string>


class BoolPublisherNode: public BT::SyncActionNode{
    public:
        explicit BoolPublisherNode(const std::string &name,
                               const BT::NodeConfiguration &config,
                               rclcpp::Node::SharedPtr node_ptr);

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts() {
            return BT::PortsList {BT::InputPort<bool>("value"),
                                  BT::InputPort<std::string>("topic")};
        }


    private:
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bool_publisher_ptr_;
		rclcpp::Node::SharedPtr node_ptr_;
};
