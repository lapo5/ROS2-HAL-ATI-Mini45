######## FORCE/TORQUE INDEX ########
ATI_FT_FX = 0
ATI_FT_FY = 1
ATI_FT_FZ = 2
ATI_FT_TX = 3
ATI_FT_TY = 4
ATI_FT_TZ = 5

######## MESSAGES ######## 
STATUS_SG0_SG2_SG4 = 0x200
SG1_SG3_SG5 = 0x201
MATRIX_SG0_SG1 = 0x202
MATRIX_SG2_SG3 = 0x203
MATRIX_SG4_SG5 = 0x204
FT_SERIAL_NUMBER = 0x205
SET_ACTIVE_CALIBRATION = 0x206
COUNTS_PER_UNIT = 0x207
UNIT_CODES = 0x208
RESET = 0x20C
SET_BASE_ID = 0x20D
SET_BAUD_RATE = 0x20E
FIRMWARE_VERSION = 0x20F

######## STATUS ######## 
WATCHDOG_RESET = 0
DAC_ADC_TOO_HIGH = 1
DAC_ADC_TOO_LOW = 2
ANALOG_GROUND_OOF = 3
POWER_SUPPLY_TOO_HIGH = 4
POWER_SUPPLY_TOO_LOW = 5
BAD_ACTIVE_CALIBRATION = 6
EEPROM_FAILURE = 7
CONFIGURATION_INVALID = 8
RESERVED_1 = 9
RESERVED_2 = 10
TEMPERATURE_TOO_HIGH = 11
TEMPERATURE_TOO_LOW = 12
RESERVED_3 = 13
CAN_BUS_ERROR = 14
ANY_ERROR = 15

######## UNIT CODES ########
UNIT_CODES_LBF = 1
UNIT_CODES_N = 2
UNIT_CODES_KLBF = 3
UNIT_CODES_KN = 4
UNIT_CODES_KGF = 5
UNIT_CODES_GF = 6
UNIT_CODES_LBF_IN = 1
UNIT_CODES_LBF_FT = 2
UNIT_CODES_N_M = 3
UNIT_CODES_N_MM = 4
UNIT_CODES_KGF_CM = 5
UNIT_CODES_KN_M = 6

########### CALIBRATION XML FILE
tblNetFTCalibrationInfo = 0
xml_data_SerialNumber = 0
xml_data_BodyStyle = 1
xml_data_CalibrationPartNumber = 2
xml_data_Family = 3
xml_data_CalibrationDate = 4
xml_data_MatrixFX = 5
xml_data_MatrixFY = 6
xml_data_MatrixFZ = 7
xml_data_MatrixTX = 8
xml_data_MatrixTY = 9
xml_data_MatrixTZ = 10
xml_data_GaugeGains = 11
xml_data_GaugeOffsets = 12
xml_data_GCalibrationIndex = 13

tblCalibrationInformation = 1
xml_data2_CalibrationPartNumber = 0
xml_data2_ForceUnits = 1
xml_data2_TorqueUnits = 2
xml_data2_CountsPerForce = 3
xml_data2_CountsPerTorque = 4
xml_data2_MaxRatings = 5
xml_data2_Resolutions = 6
xml_data2_Ranges = 7
xml_data2_6BitScaleFactors = 8

def detect_error_code(i):
    if i == WATCHDOG_RESET:
        return 'WATCHDOG_RESET'
    elif i == DAC_ADC_TOO_HIGH:
        return 'DAC_ADC_TOO_HIGH'
    elif i == DAC_ADC_TOO_HIGH:
        return 'DAC_ADC_TOO_HIGH'
    elif i == ANALOG_GROUND_OOF:
        return 'ANALOG_GROUND_OOF'
    elif i == POWER_SUPPLY_TOO_HIGH:
        return 'POWER_SUPPLY_TOO_HIGH'
    elif i == POWER_SUPPLY_TOO_LOW:
        return 'POWER_SUPPLY_TOO_LOW'
    elif i == BAD_ACTIVE_CALIBRATION:
        return 'BAD_ACTIVE_CALIBRATION'
    elif i == EEPROM_FAILURE:
        return 'EEPROM_FAILURE'
    elif i == CONFIGURATION_INVALID:
        return 'CONFIGURATION_INVALID'
    elif i == RESERVED_1:
        return 'RESERVED_1'
    elif i == RESERVED_2:
        return 'RESERVED_2'
    elif i == RESERVED_3:
        return 'RESERVED_3'
    elif i == CAN_BUS_ERROR:
        return 'CAN_BUS_ERROR'
    else:
        return 'ANY_ERROR'

scale = 16
num_of_bits = 16
def convert_hex_to_signed_int(hex_value):
    bitmask = bin(int(hex_value, scale))[2:].zfill(num_of_bits)
    result = 0
    if bitmask[0] == '1':
        result = -32768

    for i in range(1, 16):
        if bitmask[i] != '0':
            result = result + pow(2, 16 - i - 1)

    return result