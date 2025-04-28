# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "plan_env;path_searching;bspline;bspline_opt;traj_utils;message_runtime;trajectory_msgs;tf2;tf2_ros;tf2_geometry_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lplan_manage".split(';') if "-lplan_manage" != "" else []
PROJECT_NAME = "plan_manage"
PROJECT_SPACE_DIR = "/home/ubuntu20/moving_car/install"
PROJECT_VERSION = "0.0.0"
