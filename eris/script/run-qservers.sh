#!/bin/bash
# THIS SCRIPT EXECUTES THE SM AND EM QUERY SERVERS
# START THE QUERY CLIENT AT A SEPARATE TERMINAL

cd $HOME
#xterm -hold -fn 6x10 -geometry 60x24-0+0 -e "roscore" &
xterm -hold -geometry -0+0 -e "rosrun eris query_server" #& uncomment this also to run EM server
#xterm -hold -geometry -0+370 -e "rosrun eris em_query_server" # uncomment this to run EM server

#xterm -hold -e "roscore" &
#xterm -hold -e "rosrun nodelet nodelet standalone velodyne_pointcloud/CloudNodelet" &
#cd $HOME/Desktop/OpenC
#xterm -hold -e "source setup.bash; rosrun pcl_tutorial front_view_bag input:=/velodyne_points" &
#cd /media/BE8C6D3A8C6CEDF9/Users/KARTHICK/Desktop/project/ros_datas
#xterm -hold -e "rosbag play file2.bag"

exit 0