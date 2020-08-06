<?xml version="1.0"?> 
<launch>
    
    <arg name="serial_port" default="/dev/ttyUSB0" />
    <arg name="frame_id" default="imu_link" />
    <arg name="operation_mode" default="IMU"/>
    <arg name="reset_orientation" default="true" />
    <arg name="frequency" default="50" /> 
    <arg name="use_magnetometer" default="false" />
    <arg name="use_temperature" default="false" /> 

    <node pkg="tf2_ros" type="static_transform_publisher" name="link1_broadcaster" args="0 0 0 0 0 0 1 fixed_frame $(arg frame_id)" />    

    <node pkg="ros_imu_bno055" type="imu_ros.py" name="ros_imu_bno055_node" output="screen">
        <param name="serial_port" value="$(arg serial_port)" />
        <param name="frame_id" value="$(arg frame_id)" />
        <param name="operation_mode" value="$(arg operation_mode)" />
        <param name="reset_orientation" value = "$(arg reset_orientation)" />
        <param name="frequency" value="$(arg frequency)" />
        <param name="use_magnetometer" value="$(arg use_magnetometer)" />    
        <param name="use_temperature" value="$(arg use_temperature)" />    
    </node>

    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find ros_imu_bno055)/utils/view_imu_rviz.rviz" />

</launch>