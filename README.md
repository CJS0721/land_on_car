# 说明
编译可能需要编两次，报错的地方和修改的地方无关；
主要修改了fast-planner的控制部分：

1、 src/Fast-Planner/uav_simulator/so3_quadrotor_simulator/src/quadrotor_simulator_so3.cpp 中的 **函数getControl()** 在订阅了so3_cmd话题的主程序中计算并输出 四旋翼**输入力矩、螺旋桨转速**文件  

2、 读取生成的轨迹并初步计算so3_cmd : src/Fast-Planner/uav_simulator/so3_control/src/so3_control_nodelet.cpp 中的 **函数position_cmd_callback()** 嵌套 **函数 publishSO3Command()** 实现读取 规划器生成的轨迹信息 const quadrotor_msgs::PositionCommand::ConstPtr& cmd （此处我增加了MPC所需的预测区间内的状态数组;

3、 /home/ubuntu20/moving_car/src/Fast-Planner/fast_planner/plan_manage/src/traj_server.cpp 中 在代码行 pos_cmd_pub.publish(cmd); 之前增加了MPC所需的预测区间内状态的赋值；

4、 除上述修改外，还在  src/Fast-Planner/uav_simulator/Utils/quadrotor_msgs/msg/**SO3Command.msg** 和 **PositionCommand.msg**，注意是 **quadrotor_msgs** 目录下的，另一个目录下有个同名文件，没做修改，不知道有什么用src/Fast-Planner/uav_simulator/Utils/**multi_map_server**/quadrotor_msgs/msg/SO3Command.msg ;

5、为了适应 控制器所需的一些新的状态变量，还在quadrotor_simulator_so3.cpp 中的 typedef struct _Command 增加了两个变量  omega_d[3] 和 omega_d_dot[3]；

6、代码运行可能主要卡壳的地方在于 so3_control_nodelet.cpp 中的 publishSO3Command(),该函数 读取 新加的预测区间状态：mpc_controller_.calculateControlMPC(Des_pos,Des_vel,Des_acc); 输出新需要的状态 omega_d[3] 和 omega_d_dot[3]；
MPC计算、omega_d[3] 和 omega_d_dot[3] 都在src/Fast-Planner/uav_simulator/so3_control/include/so3_control/**SO3Control.h** 中进行;
