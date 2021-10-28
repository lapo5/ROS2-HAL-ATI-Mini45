import time
import can
import struct
import numpy as np

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
	from .defines import *
except:
	from defines import *

import ast
from xml.etree import ElementTree as ET

class HAL_ATI_FT_Mini45:
    def __init__(self, bustype='socketcan', channel='can0'):
        self.bustype = bustype
        self.channel = channel

        self.bias_computed = False

    def init(self, do_calibration=False):
        self.bus = can.interface.Bus(channel=self.channel, bustype=self.bustype)
        self.buffer = can.BufferedReader()
        self.notifier = can.Notifier(self.bus, [self._get_message, self.buffer])

        self.serial_number = self.get_serial_number()
        self.firmware_version = self.get_firmware_version()
        self.counts_per_force, self.counts_per_torque = self.get_counts_per_unit()

        self.force_unit, self.torque_unit = self.get_units()

        if do_calibration:
            self.calibrate_sensor()
        else:
            self.calib_matrix = np.zeros([6, 6], dtype=np.float32)
            self.is_calibrated = False

        self.bias = np.zeros([6], dtype=np.float32)

    def calibrate_sensor(self):
        self.calib_matrix = np.zeros([6, 6], dtype=np.float32)
        self.calib_matrix = self.get_transreducer_calibration_matrix()
        self.is_calibrated = True

    def _get_message(self, msg):
        return msg

    def get_serial_number(self):
        msg = can.Message(arbitration_id=FT_SERIAL_NUMBER, data=[0], is_extended_id=False)
        self.bus.send(msg)
        message = self.buffer.get_message()
        message_data = message.data

        stopping = 8
        for i in range(0, 8):
            if message_data[i] == 0:
                stopping = i

        return message_data[0:stopping].decode()

    def parse_calibration_XML(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()

        calib_net_info = root[tblNetFTCalibrationInfo]

        self.calibration_date = calib_net_info[xml_data_CalibrationDate].text
        self.gauge_gains = np.asarray(calib_net_info[xml_data_GaugeGains].text.strip().split(" "), dtype=np.int)
        self.gauge_offsets = np.asarray(calib_net_info[xml_data_GaugeOffsets].text.strip().split(" "), dtype=np.int)

        fx_row = np.asarray(calib_net_info[xml_data_MatrixFX].text.strip().split(" "), dtype=np.float32)
        fy_row = np.asarray(calib_net_info[xml_data_MatrixFY].text.strip().split(" "), dtype=np.float32)
        fz_row = np.asarray(calib_net_info[xml_data_MatrixFZ].text.strip().split(" "), dtype=np.float32)
        tx_row = np.asarray(calib_net_info[xml_data_MatrixTX].text.strip().split(" "), dtype=np.float32)
        ty_row = np.asarray(calib_net_info[xml_data_MatrixTY].text.strip().split(" "), dtype=np.float32)
        tz_row = np.asarray(calib_net_info[xml_data_MatrixTZ].text.strip().split(" "), dtype=np.float32)
        self.calib_matrix[0, :] = fx_row
        self.calib_matrix[1, :] = fy_row
        self.calib_matrix[2, :] = fz_row
        self.calib_matrix[3, :] = tx_row
        self.calib_matrix[4, :] = ty_row
        self.calib_matrix[5, :] = tz_row

        calib_info = root[tblCalibrationInformation]
        self.max_ratings = np.asarray(calib_info[xml_data2_MaxRatings].text.strip().split(" "), dtype=np.int)
        self.resolutions = np.asarray(calib_info[xml_data2_Resolutions].text.strip().split(" "), dtype=np.int)
        self.ranges = np.asarray(calib_info[xml_data2_Ranges].text.strip().split(" "), dtype=np.int)
        self.scale_factors = np.asarray(calib_info[xml_data2_6BitScaleFactors].text.strip().split(" "), dtype=np.int)

        self.is_calibrated = True


    def get_units(self):
        msg = can.Message(arbitration_id=UNIT_CODES, data=[0], is_extended_id=False)
        self.bus.send(msg)
        message = self.buffer.get_message()
        message_data = message.data
        force_unit_code = message_data[0]

        if force_unit_code == UNIT_CODES_LBF:
            force_unit = 'lbf'
        elif force_unit_code == UNIT_CODES_N:
            force_unit = 'N'
        elif force_unit_code == UNIT_CODES_KLBF:
            force_unit = 'Klbf'
        elif force_unit_code == UNIT_CODES_KN:
            force_unit = 'kN'
        elif force_unit_code == UNIT_CODES_KGF:
            force_unit = 'kgf'
        elif force_unit_code == UNIT_CODES_GF:
            force_unit = 'gf'
        else:
            raise ValueError('Unknown Force Unit.')

        torque_unit_code = message_data[1]

        if torque_unit_code == UNIT_CODES_LBF_IN:
            torque_unit = 'lbf-in'
        elif torque_unit_code == UNIT_CODES_LBF_FT:
            torque_unit = 'lbf-ft'
        elif torque_unit_code == UNIT_CODES_N_M:
            torque_unit = 'N-m'
        elif torque_unit_code == UNIT_CODES_N_MM:
            torque_unit = 'N-mm'
        elif torque_unit_code == UNIT_CODES_KGF_CM:
            torque_unit = 'kgf-cm'
        elif torque_unit_code == UNIT_CODES_KN_M:
            torque_unit = 'kN-m'
        else:
            raise ValueError('Unknown Torque Unit.')

        return force_unit, torque_unit

    def get_counts_per_unit(self):
        msg = can.Message(arbitration_id=COUNTS_PER_UNIT, data=[0], is_extended_id=False)
        self.bus.send(msg)
        message = self.buffer.get_message()
        message_data = message.data

        counts_per_force = int(message_data[0:4].hex(), 16)
        counts_per_torque = int(message_data[4:8].hex(), 16)
        return counts_per_force, counts_per_torque

    def get_transreducer_calibration_matrix(self):
        calib_matrix = np.zeros([6, 6], dtype=np.float32)

        for i in range(0, 6):
            msg = can.Message(arbitration_id=MATRIX_SG0_SG1, data=[i], is_extended_id=False)
            self.bus.send(msg)
            message1 = self.buffer.get_message()
            message2 = self.buffer.get_message()
            message3 = self.buffer.get_message()

            first_data = second_data = third_data = None

            if message1.arbitration_id == int(MATRIX_SG0_SG1):
                first_data = message1.data
            elif message1.arbitration_id == int(MATRIX_SG2_SG3):
                second_data = message1.data
            elif message1.arbitration_id == int(MATRIX_SG4_SG5):
                third_data = message1.data

            if message2.arbitration_id == int(MATRIX_SG0_SG1):
                first_data = message2.data
            elif message2.arbitration_id == int(MATRIX_SG2_SG3):
                second_data = message2.data
            elif message2.arbitration_id == int(MATRIX_SG4_SG5):
                third_data = message2.data

            if message3.arbitration_id == int(MATRIX_SG0_SG1):
                first_data = message3.data
            elif message3.arbitration_id == int(MATRIX_SG2_SG3):
                second_data = message3.data
            elif message3.arbitration_id == int(MATRIX_SG4_SG5):
                third_data = message3.data

            if first_data is None or second_data is None or third_data is None:
                raise ValueError('Not received Data for Calibration Matrix.')

            calib_matrix[i, 0] = struct.unpack('>f', first_data[0:4])[0]
            calib_matrix[i, 1] = struct.unpack('>f', first_data[4:8])[0]
            calib_matrix[i, 2] = struct.unpack('>f', second_data[0:4])[0]
            calib_matrix[i, 3] = struct.unpack('>f', second_data[4:8])[0]
            calib_matrix[i, 4] = struct.unpack('>f', third_data[0:4])[0]
            calib_matrix[i, 5] = struct.unpack('>f', third_data[4:8])[0]

        return calib_matrix

    def read_forces_torques(self):
        msg = can.Message(arbitration_id=STATUS_SG0_SG2_SG4, data=[0], is_extended_id=False)
        self.bus.send(msg)
        message1 = self.buffer.get_message()
        message2 = self.buffer.get_message()

        first_data = second_data = None
        if message1.arbitration_id == int(STATUS_SG0_SG2_SG4):
            first_data = message1.data
        elif message1.arbitration_id == int(SG1_SG3_SG5):
            second_data = message1.data

        if message2.arbitration_id == int(STATUS_SG0_SG2_SG4):
            first_data = message2.data
        elif message2.arbitration_id == int(SG1_SG3_SG5):
            second_data = message2.data

        if first_data is None or second_data is None:
            raise ValueError('Not received Data for Calibration Matrix.')

        # Convert to bitmask
        scale = 16
        num_of_bits = 16
        bitmask = bin(int(first_data[0:2].hex(), scale))[2:].zfill(num_of_bits)

        error_found = False
        for i in range(0, 16):
            if bitmask[i] == '1':
                print('Error Detected in ' + detect_error_code(i))
                error_found = True

        if error_found:
            raise Exception('Detected error within the Status code')

        sg0 = convert_hex_to_signed_int(first_data[2:4].hex())

        sg1 = convert_hex_to_signed_int(second_data[0:2].hex())

        sg2 = convert_hex_to_signed_int(first_data[4:6].hex())

        sg3 = convert_hex_to_signed_int(second_data[2:4].hex())

        sg4 = convert_hex_to_signed_int(first_data[6:8].hex())

        sg5 = convert_hex_to_signed_int(second_data[4:6].hex())

        return [sg0, sg1, sg2, sg3, sg4, sg5]

    def reset(self):
        msg = can.Message(arbitration_id=RESET, data=[0], is_extended_id=False)
        self.bus.send(msg)

    def get_firmware_version(self):
        msg = can.Message(arbitration_id=FIRMWARE_VERSION, data=[0], is_extended_id=False)
        self.bus.send(msg)
        message = self.buffer.get_message()
        message_data = message.data

        major_version = message_data[0]
        minor_version = message_data[1]

        return str(major_version) + '.' + str(minor_version)

    def compute_bias(self):
        self.bias = self.read_forces_torques()

        for i in range(0, 99):
            new_bias = self.read_forces_torques()
            for j in range(0, 6):
                self.bias[j] = self.bias[j] + new_bias[j]

        for i in range(0, 6):
            self.bias[i] = self.bias[i] / 100.0

        self.bias_computed = True

    def force_torque_computation(self, raw_data):

        if not self.is_calibrated:
            raise Exception("Compute or Upload Calibration Matrix First")

        ft = np.zeros([6], dtype=np.float32)

        for row in range(0, 6):
            for col in range(0, 6):
                if row < 3:
                    ft[row] += (self.calib_matrix[row, col] * (raw_data[col] - self.bias[col])) / float(self.counts_per_force)
                else:
                    ft[row] += (self.calib_matrix[row, col] * (raw_data[col] - self.bias[col])) / float(self.counts_per_torque)

        return ft

    def get_force_torque(self):
        ft_raw = self.read_forces_torques()
        return self.force_torque_computation(ft_raw)

