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

from bno055.registers import *

import serial #pyserial
import math
from math import sqrt
import time
import sys, os

class BNO055:

    serial_port = serial.Serial()

    def __init__(self, port = "/dev/ttyUSB0"):
        
        self.is_configuration_enabled = False
        self.operation_mode = -1

        # These units are modified internally set_imu_units()
        self.acceleration_units = int()
        self.angular_velocity_units = int()
        self.euler_orientation_units = int()
        self.temperature_units = int()
        self.orientation_mode = int()

        self.raw_accelerometer = []
        self.raw_magnetometer = []
        self.raw_gyroscope = []
        self.raw_euler = []
        self.raw_quaternion = []
        self.raw_linear_acceleration = []
        self.raw_gravity = []
        self.raw_temperature = 0

        # The BNO055 supports UART interface with the following settings:
        # 115200 bps, 8N1 (8 data bits, no parity bit, one stop bit)

        self.serial_port = serial.Serial(port = port,
                                         baudrate = 115200,
                                         timeout = 0.1,
                                         bytesize = serial.EIGHTBITS,
                                         parity=serial.PARITY_NONE,
                                         stopbits= serial.STOPBITS_ONE
                                        )


    # Returns number of bytes used by a INT variable
    def int_byte_size(self, value):

        if value != 0:
            valueBits = int(math.log(value, 2) + 1)

            if valueBits % 8 != 0:
                number_of_bytes = (valueBits / 8) + 1
            else:
                number_of_bytes = valueBits / 8
        else:
            number_of_bytes = 1
            

        return int(number_of_bytes)


    def _print(self, msg):

        if ENABLE_DEBUG == True:
            print(msg)

    def build_write_command(self, address, data):

        try:
            data_length = self.int_byte_size(data) # Returns the number of bytes used by int variable
        except:
            data_length = len(data)
 
        command = bytearray()
        command.append(START_BYTE)
        command.append(WRITE)
        command.append(address)
        command.append(data_length)            # Size of the data to write 
        
        try:
            command.append(data)
        except:
            command.extend(data)            

        return command


    def build_read_command(self, address, response_data_length):

        command = bytearray()
        command.append(START_BYTE)
        command.append(READ)
        command.append(address)
        command.append(response_data_length)   # Size of the expected response data

        return command


    def check_response(self, response):

        state = RESPONSE_ERROR

        if len(response) > 1:

            response_header = response[0]
            response_status = response[1]

            if response_header == RESPONSE_HEADER:
                
                if response_status == WRITE_SUCCESS:
                    self._print("Write succes")
                    state = RESPONSE_OK

                if response_status == READ_FAIL:
                    self._print("Read fail")
                    state = RESPONSE_ERROR

                if response_status == WRITE_FAIL:  
                    self._print("Write fail")
                    state = RESPONSE_ERROR

                if response_status == REGMAP_INVALID_ADDRESS:
                    self._print("Regmap invalid address")
                    state = RESPONSE_ERROR

                if response_status == REGMAP_WRITE_DISABLED:
                    self._print("Regmap write disabled")
                    state = RESPONSE_ERROR

                if response_status == WRONG_START_BYTE:
                    self._print("Wrong start byte")
                    state = RESPONSE_ERROR

                if response_status == BUS_OVER_RUN_ERROR:
                    self._print("Bus over run error")
                    state = RESPONSE_ERROR

                if response_status == MAX_LENGTH_ERROR:
                    self._print("Max length error")
                    state = RESPONSE_ERROR

                if response_status ==  MIN_LENGTH_ERROR:
                    self._print("Min length error")
                    state = RESPONSE_ERROR

                if response_status == RECEIVE_CHARACTER_TIMEOUT:
                    self._print("Receive character timeout")
                    state = RESPONSE_ERROR

            elif response_header == READ_SUCCESS:
                self._print("Read success")
                state = RESPONSE_OK

            else:
                self._print("Response header not detected")
                state = RESPONSE_ERROR

        else:
            self._print("Timeout expired: data not received")
            state = RESPONSE_ERROR

        return state
       

    def write_imu(self, address, data):
        
        status = -1
        attempts = 0

        while status != RESPONSE_OK:

            # Write register with data value
            command = self.build_write_command(address, data) 
            self.serial_port.write(command)

            # Get serial response 
            command_length = 2   # Response is always made up of two bytes
            response = self.serial_port.read(command_length)
            
            # Check response
            status = self.check_response(response)

            attempts+=1
            if attempts >= 10:
                self._print("Error, after ten attempts the response was not received correctly")
                status = RESPONSE_ERROR
                break

        

        return response, status


    def read_imu(self, address, response_data_length):

        # Read register
        command = self.build_read_command(address, response_data_length) 
        self.serial_port.write(command)

        # Get serial header response
        header_response = self.serial_port.read(2)
        status = self.check_response(header_response)

        # If status it is OK, get the rest of response
        if status == RESPONSE_OK:       
            response_data = self.serial_port.read(response_data_length) 
            response = header_response + response_data
        else:
            response = 0

        # Get serial response
        #command_length = 2 + response_data_length # 1 start byte + 1 read command byte + response data bytes           
        #response = self.serial_port.read(command_length)

        # Check that the response was received correctly 
        #status = self.check_response(response)


        return response, status


    def set_imu_default_configuration(self):

        # Enable configuration is required to write into IMU
        self.enable_imu_configuration()


        # Available units:
        #   - Linear acceleration:
        #           - METERS_PER_SECOND
        #           - MILI_G
        #   - Angular velocity units:
        #           - RAD_PER_SECOND
        #           - DEG_PER_SECOND
        #   - Euler orientation units:
        #           - RAD
        #           - DEG
        #   - Temperature units:
        #           - CELSIUS
        #           - FAHRENHEIT
        #   - Orientation mode:
        #           - WINDOWS_ORIENTATION
        #           - ANDROIND_ORIENTATION

        self.set_imu_units( acceleration_units = METERS_PER_SECOND,   # Linear acceleration units
                            angular_velocity_units = RAD_PER_SECOND,  # Anguar velocity units  
                            euler_orientation_units = RAD,            # Euler orientation units
                            temperature_units = CELSIUS,              # Temperature units
                            orientation_mode = WINDOWS_ORIENTATION    # Orientation mode
                          )


        # Available axis placements (see section 3.4):
        #    - P0                           
        #    - P1  (default, used in bno055 aliexpress board)        
        #    - P2                             
        #    - P3
        #    - P4
        #    - P5
        #    - P6
        #    - P7

        self.set_imu_axis(axis_placement = P1 )


        # Available operation modes:
        #    - IMU                           
        #    - COMPASS             
        #    - M4G                              
        #    - NDOF_FMC_OFF
        #    - NDOF (recommended for robotics)

        self.set_imu_operation_mode(operation_mode = IMU) 


    def reset_imu(self):

        # Reset does not return a standard result, then write_imu
        # and read_imu are not used
        
        # Reset IMU
        reset_command = self.build_write_command(SYS_TRIGGER, RST_SYS) 
        self.serial_port.write(reset_command)

        # Prepare the command to query the ID
        id_command = self.build_read_command(CHIP_ID, 1)
        response = []

        # Wait until the IMU is ready again. If the IMU is still 
        # resetting it only returns 1 byte for any command
        while len(response) < 2:
            self.serial_port.write(id_command)
            response = self.serial_port.read(3)
            time.sleep(0.1)
        
        self._print("IMU successfully reset")

        # If the reset fails, the IMU stays in the while loop
        status = RESPONSE_OK

        return status 


    def get_imu_id(self):

        imu_id = -1

        response, status = self.read_imu(CHIP_ID, 1)
        
        if status == RESPONSE_OK:

            imu_id = response[2]
        
        return imu_id


    def enable_imu_configuration(self):
        
        status = self.set_imu_operation_mode(CONFIGMODE)
        
        # if self.is_configuration_enabled == False:

        #     response, status = self.write_imu(OPR_MODE, CONFIGMODE)

        #     if status == RESPONSE_OK:
        #         self.is_configuration_enabled = True
        #         self.operation_mode = CONFIGMODE

        #     # Switch Any operation mode to CONFIGMODE requires 19ms (see table 3-6)
        #     time.sleep(0.2) # Sleep 200 ms

        # else:
        #     status = RESPONSE_OK

        return status


    def set_imu_units(self, acceleration_units, angular_velocity_units,
                      euler_orientation_units, temperature_units, orientation_mode ):

        status = -1

        if self.is_configuration_enabled == True:


            self.acceleration_units = acceleration_units  
            self.angular_velocity_units = angular_velocity_units 
            self.euler_orientation_units = euler_orientation_units          
            self.temperature_units = temperature_units             
            self.orientation_mode = orientation_mode  

            response, status = self.write_imu(UNIT_SEL, self.acceleration_units |    
                                                        self.angular_velocity_units |     
                                                        self.euler_orientation_units  |                
                                                        self.temperature_units |             
                                                        self.orientation_mode   
                                                        )

        else:
            self._print("Operation mode 'CONFIGMODE' is not set!")  
            status = RESPONSE_ERROR

        return status


    def set_imu_axis(self, axis_placement):

        status = -1

        if self.is_configuration_enabled == True:

            config_switcher = {
                    P0: AXIS_REMAP_CONFIG_P0,
                    P1: AXIS_REMAP_CONFIG_P1,
                    P2: AXIS_REMAP_CONFIG_P2,
                    P3: AXIS_REMAP_CONFIG_P3,
                    P4: AXIS_REMAP_CONFIG_P4,
                    P5: AXIS_REMAP_CONFIG_P5,
                    P6: AXIS_REMAP_CONFIG_P6,
                    P7: AXIS_REMAP_CONFIG_P7
                }

            sign_switcher = {
                    P0: AXIS_REMAP_SIGN_P0,
                    P1: AXIS_REMAP_SIGN_P1,
                    P2: AXIS_REMAP_SIGN_P2,
                    P3: AXIS_REMAP_SIGN_P3,
                    P4: AXIS_REMAP_SIGN_P4,
                    P5: AXIS_REMAP_SIGN_P5,
                    P6: AXIS_REMAP_SIGN_P6,
                    P7: AXIS_REMAP_SIGN_P7
            }

            remap_config = config_switcher.get(axis_placement, P2)
            remap_sign = sign_switcher.get(axis_placement, P2)

            response_remap_config, status_config = self.write_imu(AXIS_MAP_CONFIG, remap_config)
            response_remap_sign, status_sign = self.write_imu(AXIS_MAP_SIGN, remap_sign)

            if status_config == RESPONSE_OK and status_sign == RESPONSE_OK:
                status = RESPONSE_OK
            else:
                status = RESPONSE_ERROR

        else:
            self._print("Operation mode 'CONFIGMODE' is not set!")
            status = RESPONSE_ERROR

        return status


    def set_imu_operation_mode(self, operation_mode):   
        
        if self.operation_mode != operation_mode:

            self.operation_mode = operation_mode

            response, status = self.write_imu(OPR_MODE, operation_mode)
            
            if status == RESPONSE_OK:
               
                if operation_mode == CONFIGMODE:
                    self.is_configuration_enabled = True
                else:
                    self.is_configuration_enabled = False

            # Switch CONFIGMODE to Any operation mode requires 7ms (see table 3-6)
            # Switch Any operation mode to CONFIGMODE requires 19ms (see table 3-6)
            time.sleep(0.2) # Sleep 200 ms
        
        else:
            status = RESPONSE_OK


        return status

    def set_oscillator(self, oscillator_type):

        status = -1
        
        # Ask if clock configuration is available
        response_clk, status_clk = self.read_imu(SYS_CLK_STATUS, 1)

        if status_clk == RESPONSE_OK:

            main_clock_status = response_clk[2] 

            # If clock is available, set the oscillator
            if main_clock_status == 0:                

                    #Set oscillator type into the IMU
                    response, status = self.write_imu(SYS_TRIGGER, oscillator_type) 
            
            else:
                status = RESPONSE_ERROR
                self._print("Main clock not available")
        else:
            status = RESPONSE_ERROR
        
        

        return  status
        

    def get_calibration(self):

        self.enable_imu_configuration()

        response, status = self.read_imu(CALIBRATION_ALL_DATA, CALIBRATION_ALL_DATA_LENGTH)

        if status == RESPONSE_OK:

            calibration = response[2:24]

            accelerometer_offset_x = response[2:4]
            accelerometer_offset_y = response[4:6]
            accelerometer_offset_z = response[6:8]

            magnetometer_offset_x = response[8:10]            
            magnetometer_offset_y = response[10:12]
            magnetometer_offset_z = response[12:14]

            gyroscope_offset_x = response[14:16]
            gyroscope_offset_y = response[16:18]
            gyroscope_offset_z = response[18:20]

            accelerometer_radius = response[20:22]
            magnetometer_radius = response[22:24]

        else:
            calibration = 0

        return calibration, status


    def set_calibration(self, calibration):

        self.enable_imu_configuration()

        response, status = self.write_imu(CALIBRATION_ALL_DATA, calibration)

        if status == RESPONSE_OK:
            self._print("IMU calibration successful")
        else:
            self._print("IMU calibration failed")

        return status


    def calibrate_imu(self, operation_mode):

        calibration_status = []

        self.set_imu_operation_mode(operation_mode)
        
        response, status = self.get_calibration_status()

        if status == RESPONSE_OK:
            calibration_status = response


        return calibration_status, status


    def get_calibration_status(self):

        calibration_status = []
        
        response, status = self.read_imu(CALIB_STAT, CALIB_STAT_LENGHT)

        if status == RESPONSE_OK:

            system_calibration_status = (response[2] >> 6)  & 0b11
            gyroscope_calibration_status = (response[2] >> 4) & 0b11
            accelerometer_calibration_status  = (response[2] >> 2) & 0b11
            magnetometer_calibration_status = (response[2] >> 0) & 0b11

            calibration_status = [system_calibration_status, gyroscope_calibration_status,
                                  accelerometer_calibration_status, magnetometer_calibration_status]


            #print(bin(system_calibration_status))
            #print(bin(gyroscope_calibration_status))
            #print(bin(accelerometer_calibration_status))
            #print(bin(magnetometer_calibration_status))

        return calibration_status, status


    def update_imu_data(self):

        response, status = self.read_imu(VECTOR_ALL_DATA, VECTOR_ALL_DATA_LENGTH)

        if status == RESPONSE_OK:
            self.raw_accelerometer = response[2:8]
            self.raw_magnetometer = response[8:14]
            self.raw_gyroscope = response[14:20]
            self.raw_euler = response[20:26]
            self.raw_quaternion = response[26:34]
            self.raw_linear_acceleration = response[34:40]
            self.raw_gravity = response[40:46]
            self.raw_temperature = response[46:47]


    def get_quaternion_orientation(self):

        # Get quaternion orientation vector data
        response = self.raw_quaternion
        
        raw_quaternion_w = int.from_bytes(response[0:2], 'little',  signed= True) 
        raw_quaternion_x = int.from_bytes(response[2:4], 'little',  signed= True) 
        raw_quaternion_y = int.from_bytes(response[4:6], 'little',  signed= True) 
        raw_quaternion_z = int.from_bytes(response[6:8], 'little',  signed= True) 
        
        # Normalize quaternion
        norm = sqrt(
          raw_quaternion_x * raw_quaternion_x +
          raw_quaternion_y * raw_quaternion_y +
          raw_quaternion_z * raw_quaternion_z +
          raw_quaternion_w * raw_quaternion_w)

        quaternion_w = raw_quaternion_w / norm
        quaternion_x = raw_quaternion_x / norm
        quaternion_y = raw_quaternion_y / norm
        quaternion_z = raw_quaternion_z / norm
     
        # print (quaternion_w)
        # print (quaternion_x)
        # print (quaternion_y)
        # print (quaternion_z)

        return quaternion_w, quaternion_x, quaternion_y, quaternion_z


    def get_euler_orientation(self):

        # Get euler orientation vector data
        response = self.raw_euler

        # Get  euler orientation axis data
        raw_euler_x = int.from_bytes(response[0:2], 'little',  signed= True) 
        raw_euler_y = int.from_bytes(response[2:4], 'little',  signed= True) 
        raw_euler_z = int.from_bytes(response[4:6], 'little',  signed= True)

        if self.euler_orientation_units == RAD:
            # Convert values to an appropriate range (section 3.6.4)
            euler_x = raw_euler_x / ANGULAR_RAD_SCALE
            euler_y = raw_euler_y / ANGULAR_RAD_SCALE
            euler_z = raw_euler_z / ANGULAR_RAD_SCALE

        elif self.euler_orientation_units == DEG:
            # Convert values to an appropriate range (section 3.6.4)
            euler_x = raw_euler_x / ANGULAR_DEG_SCALE
            euler_y = raw_euler_y / ANGULAR_DEG_SCALE
            euler_z = raw_euler_z / ANGULAR_DEG_SCALE
            

        else:
            self._print("Error: wrong angle unit, you can use: RAD or DEG ")
            return 
        
        # print (euler_x)
        # print (euler_y)
        # print (euler_z)

        return euler_x, euler_y, euler_z


    def get_gyroscope(self):

        # Get gyroscope vector data
        response = self.raw_gyroscope

        # Get gyroscope axis data
        raw_gyroscope_x = int.from_bytes(response[0:2], 'little',  signed= True) 
        raw_gyroscope_y = int.from_bytes(response[2:4], 'little',  signed= True) 
        raw_gyroscope_z = int.from_bytes(response[4:6], 'little',  signed= True)

        if self.angular_velocity_units == RAD_PER_SECOND:
            # Convert values to an appropriate range (section 3.6.4)
            gyroscope_x = raw_gyroscope_x / ANGULAR_RAD_SCALE
            gyroscope_y = raw_gyroscope_y / ANGULAR_RAD_SCALE
            gyroscope_z = raw_gyroscope_z / ANGULAR_RAD_SCALE

        elif self.angular_velocity_units == DEG_PER_SECOND:
            # Convert values to an appropriate range (section 3.6.4)
            gyroscope_x = raw_gyroscope_x / ANGULAR_DEG_SCALE
            gyroscope_y = raw_gyroscope_y / ANGULAR_DEG_SCALE
            gyroscope_z = raw_gyroscope_z / ANGULAR_DEG_SCALE

        else:
            self._print("Error: wrong angle unit, you can use: RAD or DEG ")
            return 
        
        #self._print (gyroscope_x)
        #self._print (gyroscope_y)
        #self._print (gyroscope_z)

        return gyroscope_x, gyroscope_y, gyroscope_z


    def get_linear_acceleration(self):

        # Get linear acceleration vector data
        response = self.raw_linear_acceleration

        # Get linear acceleration axis data
        raw_linear_acceleration_x = int.from_bytes(response[0:2], 'little',  signed= True) 
        raw_linear_acceleration_y = int.from_bytes(response[2:4], 'little',  signed= True) 
        raw_linear_acceleration_z = int.from_bytes(response[4:6], 'little',  signed= True)

        if self.acceleration_units == METERS_PER_SECOND:
            # Convert values to an appropriate range (section 3.6.4)
            linear_acceleration_x = raw_linear_acceleration_x / LINEAR_SCALE
            linear_acceleration_y = raw_linear_acceleration_y / LINEAR_SCALE
            linear_acceleration_z = raw_linear_acceleration_z / LINEAR_SCALE
        
        elif self.acceleration_units == MILI_G:
            # No conversion needed
            linear_acceleration_x = raw_linear_acceleration_x
            linear_acceleration_y = raw_linear_acceleration_y
            linear_acceleration_z = raw_linear_acceleration_z
        else:
            self._print("Error: wrong angle unit, you can use: METERS_PER_SECOND or MILI_G ")
            return 

        #self._print (linear_acceleration_x)
        #self._print (linear_acceleration_y)
        #self._print (linear_acceleration_z)

        return linear_acceleration_x, linear_acceleration_y, linear_acceleration_z


    def get_magnetometer(self):
        
        # Get magnetometer vector data
        response = self.raw_magnetometer

        # Get magnetometer axis data
        raw_magnetometer_x = int.from_bytes(response[0:2], 'little',  signed= True) 
        raw_magnetometer_y = int.from_bytes(response[2:4], 'little',  signed= True) 
        raw_magnetometer_z = int.from_bytes(response[4:6], 'little',  signed= True)

        # Convert values to an appropriate range (section 3.6.4)
        magnetometer_x = raw_magnetometer_x / MAGNETOMETER_SCALE
        magnetometer_y = raw_magnetometer_y / MAGNETOMETER_SCALE
        magnetometer_z = raw_magnetometer_z / MAGNETOMETER_SCALE

        #self._print (magnetometer_x)
        #self._print (magnetometer_y)
        #self._print (magnetometer_z)

        return magnetometer_x, magnetometer_y, magnetometer_z

    
    def get_gravity(self):

        # Get gravity vector data
        self.raw_gravity

        # Get gravity axis data
        raw_gravity_x = int.from_bytes(response[0:2], 'little',  signed= True) 
        raw_gravity_y = int.from_bytes(response[2:4], 'little',  signed= True) 
        raw_gravity_z = int.from_bytes(response[4:6], 'little',  signed= True)

        if self.acceleration_units == METERS_PER_SECOND:
            # Convert values to an appropriate range (section 3.6.4)
            gravity_x = raw_gravity_x / GRAVITY_SCALE
            gravity_y = raw_gravity_y / GRAVITY_SCALE
            gravity_z = raw_gravity_z / GRAVITY_SCALE
        
        elif self.acceleration_units == MILI_G:
            # No conversion needed
            gravity_x = raw_gravity_x
            gravity_y = raw_gravity_y
            gravity_z = raw_gravity_z
        
        else:
            self._print("Error: wrong angle unit, you can use: METERS_PER_SECOND or MILI_G ")
            return 

        #self._print(gravity_x)
        #self._print(gravity_y)
        #self._print(gravity_z)

        return gravity_x, gravity_y, gravity_z


    def get_temperature(self):

        # Get temperature response
        response = self.raw_temperature
        
        # Get temperature data
        raw_temperature = int.from_bytes(response[0:1], 'little',  signed= True) 
       
        if self.temperature_units == CELSIUS:
            # No conversion needed
            temperature = raw_temperature

        elif self.temperature_units == FAHRENHEIT:
            # Convert values to an appropriate range (section 3.6.4)
            temperature = raw_temperature / TEMPERATURE_F_SCALE
        
        else:
            self._print("Error: temperature unit, you can use: CELSIUS or FAHRENHEIT ")
            return 

        #self._print(temperature)

        return temperature



