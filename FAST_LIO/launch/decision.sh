source ~/sp_nav_ws/devel/setup.bash
#source ~/Desktop/nav/catkin_ws/devel/setup.bash
gnome-terminal -x bash -c "roslaunch sim_referee_system sim_referee_system.launch ;exec bash"
sleep 2s
gnome-terminal -x bash -c "roslaunch sp_decision sp_decision.launch ;exec bash"
sleep 1s
gnome-terminal -x bash -c "rosrun sp_decision armor_relay;exec bash"
sleep 1s
gnome-terminal -x bash -c "rosrun sp_decision vel_relay;exec bash"
