import time

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from HAL_ATI_FT_Mini45 import HAL_ATI_FT_Mini45
from defines import *

import os

hal_ft_sensor = HAL_ATI_FT_Mini45()

hal_ft_sensor.init(do_calibration=False)

sn = hal_ft_sensor.get_serial_number()
print("Serial Number: {0}".format(sn))

force_unit, torque_unit = hal_ft_sensor.get_units()
print("Force Unit: {0}".format(force_unit))
print("Torque Unit: {0}".format(torque_unit))

counts_per_force, counts_per_torque = hal_ft_sensor.get_counts_per_unit()
print('Counts per Force: {0}'.format(counts_per_force))
print('Counts per Torque: {0}'.format(counts_per_torque))

firmware_version = hal_ft_sensor.get_firmware_version()
print('Firmware Version: {0}'.format(firmware_version))

root_directory = os.path.dirname(os.path.realpath(__file__)) + '/'
filename = 'FT10484_Net.xml'
hal_ft_sensor.parse_calibration_XML(root_directory + filename)

print("Loaded Calibration done in {0}".format(hal_ft_sensor.calibration_date))
print('Calibration Matrix:\n {0}'.format(hal_ft_sensor.calib_matrix))

# calib_matrix = hal_ft_sensor.calibrate_sensor()
# print('Calibration Matrix:\n {0}'.format(calib_matrix))

print('\n')
hal_ft_sensor.compute_bias()
print('Computed Biases:\n {0}'.format(hal_ft_sensor.bias))

print('\n\n\n\n')
while True:
    forces_torques = hal_ft_sensor.get_force_torque()
    print('Forces: {0} - Torques: {0}'.format(forces_torques[0:3], forces_torques[3:6]))
    time.sleep(1.0)
