package_dir="/home/mzy/Code/workSpace/crp450_test_ws/src"


source $package_dir/gazebo_crp450/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash $package_dir/gazebo_crp450/PX4-Autopilot $package_dir/gazebo_crp450/PX4-Autopilot/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$package_dir/gazebo_crp450/PX4-Autopilot:$package_dir/gazebo_crp450/PX4-Autopilot/Tools/sitl_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$package_dir/gazebo_crp450/models
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$package_dir/../devel/lib
