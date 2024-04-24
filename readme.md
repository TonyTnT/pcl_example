# Description
This package contains a template ros node that transforms an incoming ROS2 pointcloud2 into a PCL pointcloud2.      
PCL can then be used to process the pointcloud.    
After sucessfull processing the pointcloud, it is turned into a second ROS2 pointcloud2 and is published.       


# Build the package      
Ubuntu 20.04 and ROS2 Foxy is required for this package.      


Clone this repository in your ros2 workspace (most commonly `$/home/$USER/dev_ws`)     
  
    
Install pcl_conversions to convert from ROS2 pointcloud2 to PCL pointcloud2    

```
sudo apt-get install ros-${ROS_DISTRO}-pcl-conversions -y
```


Build the package with colcon.
```
cd /home/$USER/dev_ws && 
colcon build --packages-select pcl_example --event-handlers console_direct+ &&
cd ..
```

# Run the node 

In your ros2 sourced terminal session call:    
    
``` 
ros2 run pcl_example pcl_example_node 
ros2 launch pcl_example launch.py
``` 
    