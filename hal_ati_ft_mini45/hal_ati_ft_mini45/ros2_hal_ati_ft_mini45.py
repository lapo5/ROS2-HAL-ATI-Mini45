# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header

from std_srvs.srv import Empty

import sys
import os

# Include parent directory, to make it possible to include Stanalone HAL Module
from . HAL_ATI_FT_Mini45 import HAL_ATI_FT_Mini45
from ati_ft_mini45_interfaces.msg import FT  
from ati_ft_mini45_interfaces.srv import ATIinformation   
from ati_ft_mini45_interfaces.srv import BiasesComputed   
from ati_ft_mini45_interfaces.srv import CalibrationMatrix   
from ati_ft_mini45_interfaces.srv import SetBiases   


class HalAtiFTMini45(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        self.hal_ft_sensor = HAL_ATI_FT_Mini45()

        self.publisher_ = self.create_publisher(FT, '/ati_ft_mini45/ft_measures', 10)    
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.srv_get_info = self.create_service(ATIinformation, 
                                        '/ati_ft_mini45/get_information', self.get_information)

        self.ser_biases_computed = self.create_service(BiasesComputed, 
                                        '/ati_ft_mini45/get_biases', self.get_biases)

        self.ser_get_calib_matrix = self.create_service(CalibrationMatrix, 
                                        '/ati_ft_mini45/get_calibration_matrix', self.get_calibration_matrix)

        self.ser_set_biases = self.create_service(SetBiases, 
                                        '/ati_ft_mini45/set_biases', self.set_biases)

        self.ser_compute_calibration = self.create_service(Empty, 
                                        '/ati_ft_mini45/compute_calibration', self.compute_calibration)

        self.ser_compute_bias = self.create_service(Empty, 
                                        '/ati_ft_mini45/compute_bias', self.compute_bias)

        do_calibration = False
        self.hal_ft_sensor.init(do_calibration=do_calibration)

        self.calib_filename = '/home/marco/ros2_devel/src/hal_ati_ft_mini45/hal_ati_ft_mini45/FT10484_Net.xml'
        self.hal_ft_sensor.parse_calibration_XML(self.calib_filename)


    def get_information(self, request, response):
        response.serial_number = self.hal_ft_sensor.serial_number
        response.firmware_version = self.hal_ft_sensor.firmware_version
        response.counts_per_force = self.hal_ft_sensor.counts_per_force
        response.counts_per_torque = self.hal_ft_sensor.counts_per_torque
        response.force_unit = self.hal_ft_sensor.force_unit
        response.torque_unit = self.hal_ft_sensor.torque_unit

        return response

    def get_biases(self, request, response):

        if not self.hal_ft_sensor.bias_computed:
            self.get_logger().info('No Biases Computed')
        response.bias_fx = self.hal_ft_sensor.bias[0]
        response.bias_fy = self.hal_ft_sensor.bias[1]
        response.bias_fz = self.hal_ft_sensor.bias[2]
        response.bias_tx = self.hal_ft_sensor.bias[3]
        response.bias_ty = self.hal_ft_sensor.bias[4]
        response.bias_tz = self.hal_ft_sensor.bias[5]

        return response

    def get_calibration_matrix(self, request, response):
        
        if not self.hal_ft_sensor.is_calibrated:
            self.get_logger().info('No Calibration Computed')

        response.calib_matrix = []
        for i in range(0, 6):
            for j in range(0, 6):
                response.calib_matrix.append(self.hal_ft_sensor.calib_matrix[i][j])

        return response

    def set_biases(self, request, response):

        self.hal_ft_sensor.bias[0] = request.bias_fx
        self.hal_ft_sensor.bias[1] = request.bias_fy
        self.hal_ft_sensor.bias[2] = request.bias_fz
        self.hal_ft_sensor.bias[3] = request.bias_tx
        self.hal_ft_sensor.bias[4] = request.bias_ty
        self.hal_ft_sensor.bias[5] = request.bias_tz

        self.hal_ft_sensor.bias_computed = True

        response.ret = True

        return response

    def compute_calibration(self, request, response):

        self.hal_ft_sensor.calibrate_sensor()
        return response

    def compute_bias(self, request, response):

        self.hal_ft_sensor.compute_bias()
        return response

    def timer_callback(self):

        if not self.hal_ft_sensor.is_calibrated or not self.hal_ft_sensor.bias_computed:
            pass
        else:
            msg = FT()   
            forces_torques = self.hal_ft_sensor.get_force_torque()   
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "ATI_FT_Mini45"                              
            msg.fx = float(forces_torques[0])                                      
            msg.fy = float(forces_torques[1])                           
            msg.fz = float(forces_torques[2])                           
            msg.tx = float(forces_torques[3])                           
            msg.ty = float(forces_torques[4])                           
            msg.tz = float(forces_torques[5])                        
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = HalAtiFTMini45()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
