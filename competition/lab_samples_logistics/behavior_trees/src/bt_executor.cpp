#include "bt_executor.h"
#include "publisher_node.cpp"
#include "subscriber_node.cpp"
#include "gotopose_node.cpp"
#include "checkodom_node.cpp"
#include "samplecollected_node.cpp"
#include "boolpublisher_node.cpp"


BTExecutor::BTExecutor(const std::string &node_name): rclcpp::Node(node_name) {
    this->declare_parameter<std::string>("bt", "");
    this->declare_parameter<int>("tick_rate_ms", 500);
    RCLCPP_INFO(get_logger(), "Started BT Executor");
}

void BTExecutor::setup(){
    RCLCPP_INFO(get_logger(), "Started BT Setup");
    create_behavior_tree();
    RCLCPP_INFO(get_logger(), "Created BT Successfully");

    rclcpp::Parameter int_param = this->get_parameter("tick_rate_ms");
    int tick_rate_ms = int_param.as_int();
    auto timer_period = std::chrono::duration<int, std::ratio<1, 1000>>(tick_rate_ms);
    timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&BTExecutor::update_behavior_tree, this));

		str_publisher_ptr_ = this->create_publisher<std_msgs::msg::String>("active_task", 10);
		end_publisher_ptr_ = this->create_publisher<std_msgs::msg::Empty>("finish", 10);


		halt_bt_sub_ptr_ = this->create_subscription<std_msgs::msg::Bool>("halt", 10, std::bind(&BTExecutor::halt_callback, this, std::placeholders::_1));

    rclcpp::spin(shared_from_this());
}

void BTExecutor::halt_callback(const std_msgs::msg::Bool::SharedPtr new_msg){
		if(new_msg->data)
			halt_behavior_tree();
}

void BTExecutor::halt_behavior_tree(){
    RCLCPP_INFO(get_logger(), "BT halted");
    timer_->cancel();
		tree_.haltTree();
		shared_from_this().reset();
    //rclcpp::shutdown();
}

void BTExecutor::update_behavior_tree()
{
    BT::NodeStatus tree_status = tree_.tickRoot();
		if (tree_status == BT::NodeStatus::RUNNING)
			return;

    if (tree_status == BT::NodeStatus::FAILURE)
      RCLCPP_INFO(get_logger(), "BT Ended with FAILURE");
    else if (tree_status == BT::NodeStatus::SUCCESS)
      RCLCPP_INFO(get_logger(), "BT Ended with SUCCESS");

		std_msgs::msg::Empty end_msg;
		end_publisher_ptr_->publish(end_msg);

    halt_behavior_tree();

}

void BTExecutor::register_nav2_plugins(){
    const std::string plugins_path = ament_index_cpp::get_package_share_directory("behavior_trees") + "/behavior_trees/plugins.txt";
    std::ifstream plugin_file;
    plugin_file.open(plugins_path);

    BT::SharedLibrary loader;

    if (plugin_file.is_open())
        for(std::string plugin; std::getline(plugin_file, plugin);)
            factory_.registerFromPlugin(loader.getOSName(plugin));

    plugin_file.close();
}

void BTExecutor::create_behavior_tree(){
    rclcpp::Parameter str_param = this->get_parameter("bt");
    std::string tree_xml = str_param.as_string();

    // Registering Custom BT Nodes
    RCLCPP_INFO(get_logger(), "Registering Nodes");
    BT::NodeBuilder builder = [=](const std::string &name, const BT::NodeConfiguration &config){
        return std::make_unique<PublisherNode>(name, config, shared_from_this());
    };
    factory_.registerBuilder<PublisherNode>("Publisher", builder);
    builder = [=](const std::string &name, const BT::NodeConfiguration &config){
        return std::make_unique<BoolPublisherNode>(name, config, shared_from_this());
    };
    factory_.registerBuilder<BoolPublisherNode>("BoolPublisher", builder);
    builder = [=](const std::string &name, const BT::NodeConfiguration &config){
        return std::make_unique<SubscriberNode>(name, config, shared_from_this());
    };
    factory_.registerBuilder<SubscriberNode>("Subscriber", builder);

    builder = [=](const std::string &name, const BT::NodeConfiguration &config){
        return std::make_unique<CheckOdomNode>(name, config, shared_from_this());
    };
    factory_.registerBuilder<CheckOdomNode>("CheckOdomNode", builder);

    builder = [=](const std::string &name, const BT::NodeConfiguration &config){
        return std::make_unique<SampleCollectedNode>(name, config, shared_from_this());
    };
    factory_.registerBuilder<SampleCollectedNode>("SampleCollected", builder);

    builder = [=](const std::string &name, const BT::NodeConfiguration &config){
        return std::make_unique<GoToPose>(name, config, shared_from_this());
    };
    factory_.registerBuilder<GoToPose>("GoToPose", builder);

    // Registering nav2 nodes
    RCLCPP_INFO(get_logger(), "Registering Nav2 Plugins");
    register_nav2_plugins();


    // Creating tree from xml
    RCLCPP_INFO(get_logger(), "Creating Tree %s", tree_xml.c_str());
    tree_ = factory_.createTreeFromFile(tree_xml);

		node_logger();
}

void BTExecutor::node_logger(){
		for (auto &node_ptr : tree_.nodes) {
			auto status_sub = node_ptr->subscribeToStatusChange([this](BT::TimePoint timestamp,
			                                                           const BT::TreeNode &node,
																       BT::NodeStatus prev_status,
																	   BT::NodeStatus new_status){
					(void) timestamp;
					RCLCPP_INFO(get_logger(), "Ticking node: %s", node.name().c_str());
					RCLCPP_INFO(get_logger(), "Previous Status: %s", toStr(prev_status).c_str());
					RCLCPP_INFO(get_logger(), "Current Status: %s\n", toStr(new_status).c_str());

					std_msgs::msg::String str_msg;
					str_msg.data = node.name() + "/" + toStr(new_status);
					str_publisher_ptr_->publish(str_msg);
			});
			status_change_.push_back(status_sub);
		}

}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BTExecutor>("bt_executor");
  node->setup();

  return 0;
}
