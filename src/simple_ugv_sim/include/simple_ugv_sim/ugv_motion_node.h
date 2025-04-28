#ifndef UGV_TYPES_H
#define UGV_TYPES_H

#include <ros/ros.h>
#include <std_msgs/Header.h>

namespace simple_ugv_sim {

// 无人车状态结构体
struct UGVState1 {
  std_msgs::Header header;  // 时间戳和坐标系信息
  
  // 位置信息
  double x;        // X坐标 (m)
  double y;        // Y坐标 (m)
  double z;        // Z坐标 (m)
  double yaw;      // 偏航角 (rad)
  
  // 速度信息
  double v_linear; // 线速度 (m/s)
  double v_angular; // 角速度 (rad/s)
  
  // 方向向量
  double dir_x;    // 方向向量X分量
  double dir_y;    // 方向向量Y分量
  
  // 构造函数
  UGVState1() : 
    x(0.0), y(0.0), z(0.0), yaw(0.0),
    v_linear(0.0), v_angular(0.0),
    dir_x(1.0), dir_y(0.0){
    header.frame_id = "world";
  }
};

// 转换为ROS消息的函数可以在具体实现中添加

} // namespace simple_ugv_sim

#endif // UGV_TYPES_H
