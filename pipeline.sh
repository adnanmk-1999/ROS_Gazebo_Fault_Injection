#!/bin/bash

for (( i=0;$i<1;i=`expr $i+1`))
do
    gnome-terminal -t "start_gazebo" -- bash -c "source devel/setup.bash;roslaunch panda_gazebo start_simulation.launch; exec bash"
    sleep 2s

   
    gnome-terminal -t "start_playing" -- bash -c "source devel/setup.bash;rosrun moveit_tutorials own_pick_place_VJZL"
    sleep 3s
    gnome-terminal -t "plot" -- bash -c "source devel/setup.bash;rqt_plot"
    gnome-terminal -t "record" -- bash -c "source devel/setup.bash;rosbag record /faulty_joint_states "
    sleep 100s
    

    gnome-terminal -t "end_everything" -- bash -c "ps aux | grep ros | awk '{print \$2}' | xargs kill -9;ps aux | grep rviz | awk '{print \$2}' | xargs kill -9;exec bash"
    # gnome-terminal -t "end_everything" -- bash -c "ps aux | grep ros | awk '{print \$2}'; exec bash"
    sleep 10s

done



