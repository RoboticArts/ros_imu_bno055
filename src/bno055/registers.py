# Enable print to debug
ENABLE_DEBUG = False

# Calibration
NOT_CALIBRATED = 0x00
FULL_CALIBRATION = 0x01

# IMU identification
CHIP_ID = 0x00                          # Returns IMU ID
BNO055_ID = 0xA0                        # All BNO055 has same ID

# Command utils (see 4.7)
START_BYTE = 0xAA
WRITE = 0x00
READ = 0x01
RESPONSE_HEADER = 0xEE
WRITE_SUCCESS = 0x01
READ_SUCCESS = 0xBB
READ_FAIL = 0x02
WRITE_FAIL = 0x03
REGMAP_INVALID_ADDRESS = 0x04 
REGMAP_WRITE_DISABLED = 0x05
WRONG_START_BYTE = 0x06
BUS_OVER_RUN_ERROR = 0x7
MAX_LENGTH_ERROR = 0x08
MIN_LENGTH_ERROR = 0x09
RECEIVE_CHARACTER_TIMEOUT = 0x0A

# Custom command utils
RESPONSE_OK  = 0x0B
RESPONSE_ERROR = 0x0C

# IMU configurations

# System configuration
SYS_TRIGGER = 0x3F
RST_SYS = 0x20
CLK_SEL = 0x80
SYS_CLK_STATUS = 0x38
INTERNAL_OSCILLATOR = 0x00
EXTERNAL_OSCILLATOR = 0x01


# Power configuration (section 3.2)
PWR_MODE = 0x3E                         # Set power mode
NORMAL_MODE = 0x00                      # Normal power
LOW_POWER_MODE = 0x1                    # Low power

# Unit configuration (section 3.6.1)
UNIT_SEL = 0x3B                         # Set units
METERS_PER_SECOND   = 0b00000000        # Acceleration
MILI_G              = 0b00000001        # Acceleration units
DEG_PER_SECOND      = 0b00000000        # Anguar velocity units
RAD_PER_SECOND      = 0b00000010        # Anguar velocity units
DEG                 = 0b00000000        # Euler orientation units
RAD                 = 0b00000100        # Euler orientation units
CELSIUS             = 0b00000000        # Temperature units 
FAHRENHEIT          = 0b00001000        # Temperature units
WINDOWS_ORIENTATION = 0b00000000        # Orientation mode      
ANDROID_ORIENTATION = 0b10000000        # Orientation mode

# Operational mode configuration (section 3.3 table 3.5)
OPR_MODE =  0x3D                        # Set operation mode
CONFIGMODE = 0x00                       # Enable configuration
IMU = 0x08                              # Relative orientation, magnetometer will suspend
COMPASS = 0x09                          # Absolute orientation, gyroscope will suspend
M4G = 0x0A                              # Relative orientation, gyroscope will suspend
NDOF_FMC_OFF = 0x0B                     # Absolute orientation, all sensors will work
NDOF = 0x0C                             # Absolute orientation, all sensors will work

# Axis configuration
AXIS_MAP_CONFIG = 0x41
AXIS_MAP_SIGN = 0x42

AXIS_REMAP_CONFIG_P0 = 0x21
AXIS_REMAP_CONFIG_P1 = 0x24
AXIS_REMAP_CONFIG_P2 = 0x24
AXIS_REMAP_CONFIG_P3 = 0x21
AXIS_REMAP_CONFIG_P4 = 0x24
AXIS_REMAP_CONFIG_P5 = 0x21
AXIS_REMAP_CONFIG_P6 = 0x21
AXIS_REMAP_CONFIG_P7 = 0x24

AXIS_REMAP_SIGN_P0 = 0x04
AXIS_REMAP_SIGN_P1 = 0x00
AXIS_REMAP_SIGN_P2 = 0x06
AXIS_REMAP_SIGN_P3 = 0x02
AXIS_REMAP_SIGN_P4 = 0x03
AXIS_REMAP_SIGN_P5 = 0x01
AXIS_REMAP_SIGN_P6 = 0x07
AXIS_REMAP_SIGN_P7 = 0x05

P0 = 0x00
P1 = 0x01
P2 = 0x02
P3 = 0x03
P4 = 0x04
P5 = 0x05
P6 = 0x06
P7 = 0x07



# Each vector is accessible by indicating the first register of its axis
VECTOR_ALL_DATA = 0x08                   # First axis register is ACC_DATA_X_LSB
VECTOR_ACCELERATION = 0x08               # First axis register is ACC_DATA_X_LSB 
VECTOR_GYROSCOPE = 0x14                  # First axis register is GYR_DATA_X_LSB 
VECTOR_QUATERNION_ORIENTATION = 0x20     # First axis register is QUA_DATA_W_LSB 
VECTOR_LINEAR_ACCELERATION = 0x28        # First axis register is LIA_DATA_X_LSB 
VECTOR_EULER_ORIENTATION = 0x1A          # First axis register is EUL_DATA_X_LSB 
VECTOR_MAGNETOMETER = 0x0E               # First axis register is MAG_DATA_X_LSB 
VECTOR_GRAVITY = 0x2E                    # First axis register is GRV_DATA_X_LSB
TEMPERATURE = 0X34                       # Register is TEMP 

# Each axis component has 2 bytes
VECTOR_ALL_DATA_LENGTH = 46              # acc, gyr, qua, linacc, eu, mag, grav and temp
VECTOR_ACCELERATION_LENGTH = 6           # x,y,z  (2 bytes x 3 axis)
VECTOR_GYROSCOPE_LENGTH = 6              # roll, yaw, pitch (2 bytes x 3 axis)
VECTOR_QUATERNION_ORIENTATION_LENGTH = 8 # w,x,y,z (2 bytes x 4 axis)
VECTOR_LINEAR_ACCELERATION_LENGTH = 6    # x,y,z  (2 bytes x 3 axis)
VECTOR_EULER_ORIENTATION_LENGTH = 6      # roll, yaw, pitch (2 bytes x 3 axis)
VECTOR_MAGNETOMETER_LENGTH = 6           # x,y,z  (2 bytes x 3 axis)
VECTOR_GRAVITY_LENGTH = 6                # x,y,z  (2 bytes x 3 axis)
TEMPERATURE_LENGTH = 1                   # temperature (1 byte)


# Convert the value received from the imu into real units (see section 3.6.4)
# The rest of values do not need conversion
ANGULAR_RAD_SCALE = 900.0                # 1 rad/s = 900 LSB
ANGULAR_DEG_SCALE = 16.0                 # 1 rad/s = 16 LSB
LINEAR_SCALE = 100.0                     # 1 m/s2 = 100 LSB 
MAGNETOMETER_SCALE = 16.0                # 1 uT = 16 LSB
GRAVITY_SCALE = 100.0                    # 1 m/s2 = 100 LSB
TEMPERATURE_F_SCALE = 2.0                # 2 F = 1 LSB


# IMU Calibration
CALIBRATION_ALL_DATA = 0x55              # First axis register is ACC_OFFSET_X_LSB
CALIBRATION_ALL_DATA_LENGTH = 22         # acc, mag, gyrc, acc radius, mag radius
CALIB_STAT = 0x35                        # Calibration register status
CALIB_STAT_LENGHT = 1
