source ~/sp_nav_ws/devel/setup.bash
#转发cmd_vel话题，调试导航用
gnome-terminal -x bash -c "rosrun sentry_communicator relay ;exec bash"
