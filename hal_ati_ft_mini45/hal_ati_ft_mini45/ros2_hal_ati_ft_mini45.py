import sys
import os

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from std_srvs.srv import Empty
from geometry_msgs.msg import WrenchStamped

from ament_index_python.packages import get_package_share_directory

# Include parent directory, to make it possible to include Stanalone HAL Module
from . HAL_ATI_FT_Mini45 import HAL_ATI_FT_Mini45

from ati_ft_mini45_interfaces.srv import ATIinformation, BiasesComputed, CalibrationMatrix, SetBiases

class HalAtiFTMini45(Node):

    def __init__(self):
        super().__init__('hal_ati_ft_mini45')

        self.declare_parameter("hz", "10.0")
        self.rate = float(self.get_parameter("hz").value)

        self.declare_parameter("services.get_information", "/ati_ft_mini45/get_information")
        self.get_information_service_name = self.get_parameter("services.get_information").value

        self.declare_parameter("services.get_biases", "/ati_ft_mini45/get_biases")
        self.get_biases_service_name = self.get_parameter("services.get_biases").value

        self.declare_parameter("services.get_calibration_matrix", "/ati_ft_mini45/get_calibration_matrix")
        self.get_calibration_matrix_service_name = self.get_parameter("services.get_calibration_matrix").value

        self.declare_parameter("services.set_biases", "/ati_ft_mini45/set_biases")
        self.set_biases_service_name = self.get_parameter("services.set_biases").value

        self.declare_parameter("services.compute_calibration", "/ati_ft_mini45/compute_calibration")
        self.compute_calibration_service_name = self.get_parameter("services.compute_calibration").value

        self.declare_parameter("services.compute_bias", "/ati_ft_mini45/compute_bias")
        self.compute_bias_service_name = self.get_parameter("services.compute_bias").value

        self.declare_parameter("frames.ft_sensor", "ati_ft_link")
        self.ft_link = self.get_parameter("frames.ft_sensor").value

        self.declare_parameter("publishers.ft_measures", "/ati_ft_mini45/ft_measures")
        self.pub_ft_measures_name = self.get_parameter("publishers.ft_measures").value

        self.publisher_ = self.create_publisher(WrenchStamped, self.pub_ft_measures_name, 1)    

        self.srv_get_info = self.create_service(ATIinformation, 
                                        self.get_information_service_name, self.get_information)

        self.ser_biases_computed = self.create_service(BiasesComputed, 
                                        self.get_biases_service_name, self.get_biases)

        self.ser_get_calib_matrix = self.create_service(CalibrationMatrix, 
                                        self.get_calibration_matrix_service_name, self.get_calibration_matrix)

        self.ser_set_biases = self.create_service(SetBiases, 
                                        self.set_biases_service_name, self.set_biases)

        self.ser_compute_calibration = self.create_service(Empty, 
                                        self.compute_calibration_service_name, self.compute_calibration)

        self.ser_compute_bias = self.create_service(Empty, 
                                        self.compute_bias_service_name, self.compute_bias)


        self.bus_type = "socketcan"

        self.declare_parameter("channel", "can0")
        self.channel = self.get_parameter("channel").value

        self.declare_parameter("calibrate_sensors_on_start", "False")
        self.do_calibration = self.get_parameter("calibrate_sensors_on_start").value

        self.hal_ft_sensor = HAL_ATI_FT_Mini45(bustype=self.bus_type, channel=self.channel)
        self.hal_ft_sensor.init(do_calibration=self.do_calibration)

        package_share_directory = get_package_share_directory('hal_ati_ft_mini45')
        self.calib_filename = package_share_directory + '/resources/FT10484_Net.xml'
        
        self.hal_ft_sensor.parse_calibration_XML(self.calib_filename)


        self.timer = self.create_timer(1.0/self.rate, self.timer_callback)


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

        if not self.hal_ft_sensor.is_calibrated:
            print('FT Not Calibrated')
        else:
            msg = WrenchStamped()   
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.ft_link

            forces_torques = self.hal_ft_sensor.get_force_torque()   
                                 
            msg.wrench.force.x = float(forces_torques[0])                                      
            msg.wrench.force.y = float(forces_torques[1])                           
            msg.wrench.force.z = float(forces_torques[2])                           
            msg.wrench.torque.x = float(forces_torques[3])                           
            msg.wrench.torque.y = float(forces_torques[4])                           
            msg.wrench.torque.z = float(forces_torques[5])                        
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HalAtiFTMini45()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('ATI FT Node stopped cleanly')
    except BaseException:
        print('Exception in ATI FT Node:', file=sys.stderr)
        raise
    finally:
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        rclpy.shutdown() 



if __name__ == "__main__":
    main()
