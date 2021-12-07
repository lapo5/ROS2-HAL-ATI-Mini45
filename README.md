# HAL ATI Mini45

Hardware Abstraction Layer for Force/Torque Sensor ATI Mini45.

## Description

ROS2 Package to interface via CAN to the FT ATI Mini45 sensor.
The package is splitted in core and interfaces.
The packet read the raw data from the sensor with the protocol specified in the manual 'FT Sensor - CAN Interface Manual.pdf'.
The package include also standalone HAL_ATI_FT_Mini45.py and a standalone python3 test.py

## Output: 

- ft_measures: 	force/torque measurements (sensor msgs/FTStamped)
	published in ft_sensor link

## Services

- Compute calibration: trigger computation of the calibration matrix (refer to the ATI Manual).
- Compute biases: trigger the computation of the biases. Not recommanded if the sensor moves in space
- Set biases: set statically the biases.
- Get biases: get the biases computed.
- Get calibration: get calibration matrix.
- Get information: serial_number, firmware_version, counts_per_force, counts_per_torque, force_unit, torque_unit

## Params

- calibrate_sensors_on_start: whether to calibrate the sensor on startup or not.
- channel: which can interface
- hz: rate to publish ft data

## Start Can Interface 

# Case of CAN interface with USB adapted

Run can_init.sh with admin privileges to enable can interface.

