#include "gotopose_node.h"


GoToPose::GoToPose(const std::string &name,
                   const BT::NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
	node_ptr_ = node_ptr;
  done_flag_ = false;
  name_ = name;
}


BT::NodeStatus GoToPose::onStart(){
  // Make pose
  auto goal_msg = NavigateToPose::Goal();
  auto x_in = getInput<double>("x");
  auto y_in = getInput<double>("y");
  auto yaw_in = getInput<double>("yaw");
  double x, y, yaw;
  std::string action_client_name = "navigate_to_pose";
	
  action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, action_client_name);

  if(name_ == "SampleSwap"){
    const char* env_x = std::getenv("ROBOT1_X");
    const char* env_y = std::getenv("ROBOT1_Y");
    x = std::stod(env_x);
    y = std::stod(env_y);
  }else
    x = x_in.value(), y = y_in.value();
  yaw = yaw_in.value();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.pose.position.x = x;
  goal_msg.pose.pose.position.y = y;

  // Calculate quaternion from yaw
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  q.normalize();
  goal_msg.pose.pose.orientation = tf2::toMsg(q);

  // Setup action client
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&GoToPose::nav_to_pose_callback, this, std::placeholders::_1);

  // Send pose
  done_flag_ = false;
  for(int i=0; i<5; i++){
    action_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    sleep(1);
  }

  RCLCPP_INFO(node_ptr_->get_logger(), "Sent Goal to Nav2\n");
  RCLCPP_INFO(node_ptr_->get_logger(), "Navigating to (%f, %f)\n", x, y);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPose::onRunning(){
  if (done_flag_){
    RCLCPP_INFO(node_ptr_->get_logger(), "Goal reached\n");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void GoToPose::onHalted(){
	action_client_ptr_->async_cancel_all_goals();
}

void GoToPose::nav_to_pose_callback(const GoalHandleNav::WrappedResult &result){
  // If there is a result, we consider navigation completed.
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    done_flag_ = true;
}
