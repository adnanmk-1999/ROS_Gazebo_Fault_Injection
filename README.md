# ROS_Gazebo_Fault_Injection
Initial setup to starting with ROS Gazebo MoveIT LibFranka for Fault Injection

//Procedure to start fresh with the distribution shift project and fault injection

Set up your system by following:

1.      Ubuntu 20.04, desk image, LTS
2.      ROS noetic (ROS wiki)
3.      MOVEIT NOETIC: https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html
4.      Frankaros: https://frankaemika.github.io/docs/installation_linux.html (do not clone libfranka Franka Research 3)

Do till WS creation in the step 4 just before the rosbuild command 'catkin_make'.

Copy dependency packages from this 'src' to your 'src' inside worksspace. (Remember for project FrankaEmika_DS_WS - joint state publisher package is not necessary, also 'faulty_joint_states' on line 111 in /home/adnan/FrankaEmika_FI_ws/src/moveit/moveit/moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp must be changed to 'joint_states' as in Distribution Shift project, faults are not injected)

Then run the command 'catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build'.

Wait til building completes - takes some time

Run the following commands to checkif the built is successful

    roslaunch panda_gazebo start_pick_and_place_world.launch

    roslaunch panda_gazebo put_robot_in_world.launch

    rosrun joint_state_publisher fault_injector

The frankaEmika initially in a wierd position will correct its position after running the third command above 'rosrun'.

Finally copy and paste the files outside of the src folder - 'pipeline' files

Hurray! You have successfully completed the initial setup.
Happy Coding!!

