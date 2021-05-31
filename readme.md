# Point Cloud Playgroud

## package to experiment with PCL & ROS

### Install & Quickstart:

1) Install the ROS perception_pcl package:

    sudo apt-get install ros-noetic-perception-pcl

2) Go to your ros workspace, clone and build the package:

    cd ~catkin_ws/src
    git clone 
    cd ~catkin_ws && catkin_make

3) Download the example bag file from here into the directoy called /bagfiles:

https://www.notion.so/circuitlaunch/Point-Cloud-Library-9642f8fb0c9844c68c5ad63896371eef

4) Play back the bagfile and visualize it in Rviz:

Terminal 1: roslaunch point_cloud_playground view.launch

Terminal 2: navigate to /point_cloud_playground/bagfiles, then enter:

    rosbag play example_kinect_data.bag

You should see the point cloud data playing back in Rviz



