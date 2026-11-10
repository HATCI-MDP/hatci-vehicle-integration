std_msgs/Header header

uint8 cdcu_ctrl_mode
float32 rwa
uint8 gear
uint8 cc_button
uint8 driverbraking
float32 max_rwa_deg

# Wheel speeds
float32 spd_mps
float32 spd_fl_mps
float32 spd_fr_mps
float32 spd_rl_mps
float32 spd_rr_mps

# GPS
sensor_msgs/NavSatFix fix

# IMU
CANImu imu

# Errors
bool error
bool spd_error
bool cc_button_error

# Wheel directions
int8 wheel_dir_fl
int8 wheel_dir_fr
int8 wheel_dir_rl
int8 wheel_dir_rr

# Wheel pulses
float32 wheel_pulse_fl
float32 wheel_pulse_fr
float32 wheel_pulse_rl
float32 wheel_pulse_rr
