Vendored from UniversalRobots/Universal_Robots_ROS2_Driver tag 2.16.0, ur_controllers. BSD license retained in each source. Source: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/2.16.0/ur_controllers

Changes: isolated namespace/include/plugin identity; initialize last_commanded_time_ from the controller node clock instead of default SYSTEM_TIME. No other control-law changes. Installed /opt/ros packages are untouched.
