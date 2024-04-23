source ~/sp_nav_ws/devel/setup.bash
gnome-terminal -x bash -c "roslaunch sp_decision sp_decision.launch ;exec bash"
sleep 1s
gnome-terminal -x bash -c "rosrun sp_decision cmd_vel_test;exec bash"

