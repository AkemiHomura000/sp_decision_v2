source /home/sentry/sp_nav_ws/devel/setup.bash
#roslaunch relocalization_nav.launch
gnome-terminal -x bash -c "roslaunch fast_lio relocalization_nav.launch;exec bash"
#roslaunch relocalization_mid_360.launch

