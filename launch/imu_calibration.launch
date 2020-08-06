<?xml version="1.0"?> 
<launch>
    
    <arg name="serial_port" default="/dev/ttyUSB0" />
    <arg name="operation_mode" default="IMU" />

    <node pkg="ros_imu_bno055" type="imu_calibration.py" name="ros_imu_bno055_calibration_node" output="screen">
        <param name="serial_port" value="$(arg serial_port)" />    
        <param name="operation_mode" value="$(arg operation_mode)" />    
    </node>

</launch>