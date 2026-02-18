#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include "std_msgs/msg/string.hpp"

#include "behaviortree_cpp_v3/action_node.h"

#include <string>


class PublisherNode: public BT::SyncActionNode{
    public:
        explicit PublisherNode(const std::string &name,
                               const BT::NodeConfiguration &config,
                               rclcpp::Node::SharedPtr node_ptr);

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts() {
            return BT::PortsList {BT::InputPort<std::string>("message"),
                                  BT::InputPort<std::string>("topic")};
        }


    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr str_publisher_ptr_;
				rclcpp::Node::SharedPtr node_ptr_;
};
