#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <simple_ugv_sim/ugv_motion_node.h>
#include <simple_ugv_sim/UGVState.h>

class UGVMotion {
private:
  ros::NodeHandle nh_;
  ros::Subscriber goal_sub_;
  ros::Publisher state_pub_;
  tf2_ros::TransformBroadcaster broadcaster_;
  double map_size_;
  
  // 无人车状态
  simple_ugv_sim::UGVState1 ugv_state_;
  
  // 运动控制参数
  bool movement_started_;
  bool reached_boundary_;
  bool has_goal_;
  double start_x_, start_y_;
  double goal_x_, goal_y_;
  double total_distance_;
  double traveled_distance_;
  
  // 发布频率
  double publish_rate_;

public:
  UGVMotion() : has_goal_(false),reached_boundary_(false),movement_started_(false) {
    // 从参数服务器获取初始位置和速度参数
    double init_x, init_y, init_z, init_yaw;
    double init_velocity, init_direction;
    
    // 直接使用nh_.param()读取全局参数
    nh_.getParam("/ugv/init_x", init_x);
    nh_.getParam("/ugv/init_y", init_y);
    nh_.getParam("/ugv/init_z", init_z);
    nh_.getParam("/ugv/init_yaw", init_yaw);//
    nh_.getParam("/ugv/init_velocity", init_velocity);  // 默认0.3m/s
    nh_.getParam("/ugv/init_direction", init_direction); // 默认沿y轴正方向
    nh_.getParam("/ugv/publish_rate", publish_rate_);  // 默认50Hz
    nh_.getParam("/map_size", map_size_);
    ROS_INFO("Map size: %.2f x %.2f", map_size_, map_size_);
    // 初始化无人车状态
    ugv_state_.x = init_x;
    ugv_state_.y = init_y;
    ugv_state_.z = init_z;
    ugv_state_.yaw = init_direction;//ugv_state_.yaw = init_yaw;
    ugv_state_.v_linear = init_velocity;
    ugv_state_.dir_x = cos(init_direction);
    ugv_state_.dir_y = sin(init_direction);
    
    // 初始化当前位置
    start_x_ = init_x;
    start_y_ = init_y;
    
    // 订阅2D Nav Goal话题
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, 
                              &UGVMotion::goalCallback, this);
    
    // 发布无人车状态话题
    state_pub_ = nh_.advertise<simple_ugv_sim::UGVState>("/ugv/state", 10);
    
    ROS_INFO("UGV motion node initialized at position (%.2f, %.2f, %.2f)", 
             ugv_state_.x, ugv_state_.y, ugv_state_.z);
    ROS_INFO("Initial velocity: %.2f m/s, Direction: %.2f rad", 
             ugv_state_.v_linear, init_direction);
    ROS_INFO("Waiting for 2D Nav Goal...");
    ROS_INFO("Please use the '2D Nav Goal' button in RViz to set a destination.");
  }

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // // 记录起始位置
    // start_x_ = ugv_state_.x;
    // start_y_ = ugv_state_.y;
    
    // // 记录目标位置
    // goal_x_ = msg->pose.position.x;
    // goal_y_ = msg->pose.position.y;
    
    // // 计算运动方向（弧度）
    // double direction = atan2(goal_y_ - start_y_, goal_x_ - start_x_);
    // ugv_state_.yaw = direction;
    // ugv_state_.dir_x = cos(direction);
    // ugv_state_.dir_y = sin(direction);
    
    // // 计算总距离
    // total_distance_ = sqrt(pow(goal_x_ - start_x_, 2) + pow(goal_y_ - start_y_, 2));
    // traveled_distance_ = 0.0;
    
    // has_goal_ = true;
    // reached_boundary_ = false;  // 重置边界标志
    // ROS_INFO("Received new goal: x=%.2f, y=%.2f", goal_x_, goal_y_);
    // ROS_INFO("Starting movement from: x=%.2f, y=%.2f", start_x_, start_y_);
    // ROS_INFO("Direction: %.2f radians, Distance: %.2f meters", direction, total_distance_);
    if (!movement_started_) {
      movement_started_ = true;
      ROS_INFO("Received 2D Nav Goal. Starting movement along y-axis positive direction.");
      ROS_INFO("Current position: x=%.2f, y=%.2f", ugv_state_.x, ugv_state_.y);
      ROS_INFO("Direction: %.2f radians (along y-axis positive)", ugv_state_.yaw);
    } else {
      ROS_INFO("Movement already started. Ignoring new 2D Nav Goal.");
    }
  }

  void update(double dt) {
    // 更新时间戳
    ugv_state_.header.stamp = ros::Time::now();
    
    if (!reached_boundary_ && movement_started_) {
      // 计算这一时间步长内移动的距离
      double step_distance = ugv_state_.v_linear * dt;
      
      // 更新位置
      ugv_state_.x += ugv_state_.v_linear*ugv_state_.dir_x * dt;
      ugv_state_.y += ugv_state_.v_linear*ugv_state_.dir_y * dt;
      // 检查是否到达边界
      if (std::abs(ugv_state_.x) >= map_size_ / 2 || std::abs(ugv_state_.y) >= map_size_ / 2) {
        reached_boundary_ = true;
        ROS_INFO("Reached map boundary! Stopping at (%.2f, %.2f)", ugv_state_.x, ugv_state_.y);
      }
    }
    
    // 发布无人车状态
    publishState();
    
    // 发布TF变换
    publishTransform();
  }

private:
  void publishState() {
    // 创建并填充状态消息
    simple_ugv_sim::UGVState state_msg;
    
    state_msg.header = ugv_state_.header;
    state_msg.x = ugv_state_.x;
    state_msg.y = ugv_state_.y;
    state_msg.z = ugv_state_.z;
    state_msg.yaw = ugv_state_.yaw;
    state_msg.v_linear = ugv_state_.v_linear;
    state_msg.v_angular = ugv_state_.v_angular;
    state_msg.dir_x = ugv_state_.dir_x;
    state_msg.dir_y = ugv_state_.dir_y;
    
    // 发布状态消息
    state_pub_.publish(state_msg);
  }
  
  void publishTransform() {
    ros::Time current_time = ros::Time::now();
    std::vector<geometry_msgs::TransformStamped> transforms;

    // 1. world 到 base_link 的变换
    geometry_msgs::TransformStamped base_trans;
    base_trans.header.stamp = current_time;
    base_trans.header.frame_id = "world";
    base_trans.child_frame_id = "base_link";
    base_trans.transform.translation.x = ugv_state_.x;
    base_trans.transform.translation.y = ugv_state_.y;
    base_trans.transform.translation.z = ugv_state_.z;
    
    tf2::Quaternion q;
    //ugv_state_.yaw = init_direction;
    q.setRPY(0, 0, ugv_state_.yaw);
    base_trans.transform.rotation.x = q.x();
    base_trans.transform.rotation.y = q.y();
    base_trans.transform.rotation.z = q.z();
    base_trans.transform.rotation.w = q.w();
    
    transforms.push_back(base_trans);

    // 2. base_link 到各个轮子的变换
    std::vector<std::string> wheel_names = {"front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"};
    std::vector<std::array<double, 3>> wheel_positions = {
        {0.2, 0.225, -0.05},
        {0.2, -0.225, -0.05},
        {-0.2, 0.225, -0.05},
        {-0.2, -0.225, -0.05}
    };

    for (size_t i = 0; i < wheel_names.size(); ++i) {
        geometry_msgs::TransformStamped wheel_trans;
        wheel_trans.header.stamp = current_time;
        wheel_trans.header.frame_id = "base_link";
        wheel_trans.child_frame_id = wheel_names[i];
        wheel_trans.transform.translation.x = wheel_positions[i][0];
        wheel_trans.transform.translation.y = wheel_positions[i][1];
        wheel_trans.transform.translation.z = wheel_positions[i][2];

        // 设置轮子的旋转（轮子绕Y轴旋转）
        tf2::Quaternion wheel_q;
        wheel_q.setRPY(0, M_PI/2, 0);
        wheel_trans.transform.rotation.x = wheel_q.x();
        wheel_trans.transform.rotation.y = wheel_q.y();
        wheel_trans.transform.rotation.z = wheel_q.z();
        wheel_trans.transform.rotation.w = wheel_q.w();

        transforms.push_back(wheel_trans);
    }

    // 发布所有变换
    broadcaster_.sendTransform(transforms);
}
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ugv_motion_node");
  
  UGVMotion ugv;
  
  // 获取发布频率
  ros::NodeHandle nh;
  double publish_rate;
  nh.param("/ugv/publish_rate", publish_rate, 50.0);  // 默认50Hz
  
  ros::Rate rate(publish_rate);
  ros::Time last_time = ros::Time::now();
  
  double map_size;
  nh.param("/map_size", map_size, 50.0);
  ROS_INFO("Map size: %.2f x %.2f", map_size, map_size);
  ROS_INFO("Map boundaries: x: [%.2f, %.2f], y: [%.2f, %.2f]", 
           -map_size/2, map_size/2, -map_size/2, map_size/2);
  ROS_INFO("UGV will stop when reaching the map boundary.");

  while (ros::ok()) {
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    last_time = current_time;
    
    ugv.update(dt);
    
    ros::spinOnce();
    rate.sleep();
  }
  
  return 0;
}
