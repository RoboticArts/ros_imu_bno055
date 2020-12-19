#!/usr/bin/env python3

"""
Copyright (c) 2020 Robotic Arts Industries

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

   * Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.


THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

"""

"""

Author:  Robert Vasquez Zavaleta

"""

import rospy
import os
from ros_imu_bno055.imu_bno055_api import * 
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import MagneticField
from std_srvs.srv import Empty, EmptyResponse
from std_srvs.srv import Trigger, TriggerResponse 

# rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 base_link imu_link 1000

class SensorIMU:

    def __init__(self):

        # Init node
        rospy.init_node('ros_imu_bno055_node', anonymous=False)

        # Get node name
        self.node_name = rospy.get_name()

        # Get ros params
        self.get_ros_params()

        # Create an IMU instance
        self.bno055 = BoschIMU(port = self.serial_port)

        # Internal variables
        self.imu_data_seq_counter = 0
        self.imu_magnetometer_seq_counter = 0
        self.imu_temperature_seq_counter = 0
        self.stop_request = False

        # Create topics
        self.pub_imu_data = rospy.Publisher('imu/data', Imu, queue_size=1)

        if self.use_magnetometer == True:
            self.pub_imu_magnetometer = rospy.Publisher('imu/magnetometer', MagneticField, queue_size=1)
        
        if self.use_temperature == True:
            self.pub_imu_temperature = rospy.Publisher('imu/temperature', Temperature, queue_size=1)


        # Create service
        self.reset_imu_device = rospy.Service('imu/reset_device', Empty, self.callback_reset_imu_device)
        self.calibration_imu_staus = rospy.Service('imu/calibration_status', Trigger, self.callback_calibration_imu_status)
        

        # Print node status
        rospy.loginfo(self.node_name + " ready!")


    def get_ros_params(self):

        self.serial_port = rospy.get_param(self.node_name + '/serial_port','/dev/ttyUSB0')
        self.frame_id = rospy.get_param(self.node_name + '/frame_id', 'imu_link')
        self.operation_mode_str = rospy.get_param(self.node_name + '/operation_mode', 'IMU')
        self.oscillator_str = rospy.get_param(self.node_name + '/oscillator', 'INTERNAL')
        self.reset_orientation = rospy.get_param(self.node_name + '/reset_orientation', True)
        self.frequency = rospy.get_param(self.node_name + '/frequency', 20)
        self.use_magnetometer = rospy.get_param(self.node_name + '/use_magnetometer', False)
        self.use_temperature = rospy.get_param(self.node_name + '/use_temperature', False)

        switcher = {

            'IMU': IMU,
            'COMPASS': COMPASS,
            'M4G': M4G,
            'NDOF_FMC_OFF': NDOF_FMC_OFF,
            'NDOF': NDOF,
        }

        self.operation_mode = switcher.get(self.operation_mode_str, 'IMU')

        switcher = {

            'EXTERNAL': EXTERNAL_OSCILLATOR,
            'INTERNAL': INTERNAL_OSCILLATOR 
        }
        
        self.oscillator = switcher.get(self.oscillator_str, 'INTERNAL')


    def set_imu_configuration(self):

        # IMU configuration: required every time the IMU is turned on or reset

        # Enable IMU configuration
        status_1 = self.bno055.enable_imu_configuration()

        if status_1 == RESPONSE_OK:
            rospy.loginfo("Configuration mode activated")
        else:
            rospy.logerr("Unable to activate configuration mode")

        
        # Set IMU units
        status_2 = self.bno055.set_imu_units( acceleration_units = METERS_PER_SECOND,   # Linear acceleration units
                                            angular_velocity_units = RAD_PER_SECOND,  # Anguar velocity units  
                                            euler_orientation_units = RAD,            # Euler orientation units
                                            temperature_units = CELSIUS,              # Temperature units
                                            orientation_mode = WINDOWS_ORIENTATION    # Orientation mode
                                           )
                                    
        if status_2 == RESPONSE_OK:
            rospy.loginfo("Units configured successfully")
        else:
            rospy.logwarn("Unable to configure units")


        # Set imu axis
        status_3 = self.bno055.set_imu_axis(axis_placement = P1)

        if status_3 == RESPONSE_OK:
            rospy.loginfo("Axis configured successfully")
        else:
            rospy.logwarn("Unable to configure axis")

        
        status_calibration = self.load_calibration_from_file()

        if status_calibration == RESPONSE_OK:
            rospy.loginfo("Calibration loaded successfully")
        else:
            rospy.loginfo("Calibration not detected. IMU will use default calibration")

        
        status_oscillator = self.bno055.set_oscillator(oscillator_type = self.oscillator)
        
        if status_oscillator == RESPONSE_OK:
                rospy.loginfo("%s oscillator configured successfully", self.oscillator_str)

        else:
            rospy.loginfo("Unable to configure oscillator")


        # Set operation mode. Exit configuration mode and activate IMU to work
        status_4 = self.bno055.set_imu_operation_mode(operation_mode = self.operation_mode)

        if status_4 == RESPONSE_OK:
            rospy.loginfo("Operation mode configured successfully")
        else:
            rospy.logerr("Unable to configure operation mode ") 


        # Check all status 
        if (status_1 == RESPONSE_OK and status_2 == RESPONSE_OK 
           and status_2 == RESPONSE_OK and status_4 == RESPONSE_OK):

            rospy.loginfo("IMU is working now in %s mode!", self.operation_mode_str)

        else:
            rospy.logwarn("The IMU was not configured correctly. It may not work")


    def reset_imu(self):

        status = self.bno055.reset_imu()

        if status == RESPONSE_OK:
            rospy.loginfo("IMU successfully reset")
        else:
            rospy.logwarn("Reset IMU failed")


    def load_calibration_from_file(self):

        status = -1
        dir_path = os.path.dirname(os.path.realpath(__file__))

        # Read calibration from file
        try:
            binary_file = open(str(dir_path) + "/" + self.operation_mode_str + "_calibration", "rb")
            calibration_data = binary_file.read()
            binary_file.close()
            calibration_exists = True
        except:
            calibration_data = 0
            calibration_exists = False


        # Load calibration into the IMU    
        if calibration_exists == True:
            status = self.bno055.set_calibration(calibration_data)
        else:
            status = RESPONSE_ERROR
        
        return status
    

    def callback_reset_imu_device(self, req):
        
        rospy.loginfo("====================")
        rospy.loginfo("Service: Reseting IMU...")
        self.stop_request = True
        rospy.sleep(1)
        self.reset_imu()
        self.set_imu_configuration()
        self.stop_request = False
        rospy.loginfo("Service: IMU reset completed!")

        result = EmptyResponse()
        return result


    def callback_calibration_imu_status(self, req):

        self.stop_request = True
        # Delay proportional to the frequency of the node
        time.sleep(1/self.frequency)
        calibration_status, status = self.bno055.get_calibration_status()
        self.stop_request = False

        result = TriggerResponse()

        if status == RESPONSE_OK:
            
            sys = " [System: " + str(calibration_status[0]) + "]"
            gyr = " [Gyroscope: " + str(calibration_status[1]) + "]"
            acc = " [Accelerometer: " + str(calibration_status[2]) + "]"
            mag = " [Magnetometer: " + str(calibration_status[3]) + "]" 

            result.message = sys + gyr + acc + mag
            result.success = True

        else:
            rospy.logwarn("Unable to read IMU calibration")
            result.message = "Unable to read IMU calibration"
            result.success = False

        return result

    def publish_imu_data(self):
            
        imu_data = Imu()  
        
        quaternion = self.bno055.get_quaternion_orientation()
        linear_acceleration = self.bno055.get_linear_acceleration()
        gyroscope = self.bno055.get_gyroscope()
        
        imu_data.header.stamp = rospy.Time.now()
        imu_data.header.frame_id = self.frame_id
        imu_data.header.seq = self.imu_data_seq_counter

        imu_data.orientation.w = quaternion[0]
        imu_data.orientation.x = quaternion[1]
        imu_data.orientation.y = quaternion[2]
        imu_data.orientation.z = quaternion[3]

        imu_data.linear_acceleration.x = linear_acceleration[0]
        imu_data.linear_acceleration.y = linear_acceleration[1]
        imu_data.linear_acceleration.z = linear_acceleration[2]

        imu_data.angular_velocity.x = gyroscope[0]
        imu_data.angular_velocity.y = gyroscope[1]
        imu_data.angular_velocity.z = gyroscope[2]

        imu_data.orientation_covariance[0] = -1
        imu_data.linear_acceleration_covariance[0] = -1
        imu_data.angular_velocity_covariance[0] = -1

        self.imu_data_seq_counter=+1

        self.pub_imu_data.publish(imu_data)


    def publish_imu_magnetometer(self):

        imu_magnetometer = MagneticField()

        magnetometer = self.bno055.get_magnetometer()

        imu_magnetometer.header.stamp = rospy.Time.now()
        imu_magnetometer.header.frame_id = self.frame_id
        imu_magnetometer.header.seq = self.imu_magnetometer_seq_counter

        imu_magnetometer.magnetic_field.x = magnetometer[0]
        imu_magnetometer.magnetic_field.y = magnetometer[1]
        imu_magnetometer.magnetic_field.z = magnetometer[2]

        self.imu_magnetometer_seq_counter=+1

        self.pub_imu_magnetometer.publish(imu_magnetometer)

    def publish_imu_temperature(self):

        imu_temperature = Temperature()

        temperature = self.bno055.get_temperature()

        imu_temperature.header.stamp = rospy.Time.now()
        imu_temperature.header.frame_id = self.frame_id
        imu_temperature.header.seq = self.imu_temperature_seq_counter

        imu_temperature.temperature = temperature 

        self.imu_temperature_seq_counter=+1

        self.pub_imu_temperature.publish(imu_temperature)

    def run(self):

        # Reset IMU to reset axis orientation. 
        if self.reset_orientation == True:
            self.reset_imu()

        # Configuration is necessary every time the IMU is turned on or reset
        self.set_imu_configuration()

        # Set frequency
        rate = rospy.Rate(self.frequency)

        while not rospy.is_shutdown():

            if self.stop_request == False:
                
                #start_time = time.time()
                self.bno055.update_imu_data()
                #print("--- %s seconds ---" % (time.time() - start_time)) 

                # Publish imu data
                self.publish_imu_data()
   
                # Publish magnetometer data
                if self.use_magnetometer == True:
                    self.publish_imu_magnetometer()

                # Publish temperature data                
                if self.use_temperature == True:
                    self.publish_imu_temperature()

            rate.sleep()


if __name__ == '__main__':

    imu = SensorIMU()

    try:
        imu.run()

    except rospy.ROSInterruptException:
        pass

