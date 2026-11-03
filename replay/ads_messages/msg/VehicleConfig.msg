# VehicleConfig.msg
std_msgs/Header header

# Vehicle config
string vehicle_name
string camera_name

# Camera + lidar intrinsics and extrinsics
ExtraCameraInfo cam_intrinsics_extrinsics

# Vehicle params
float32 max_rwa_deg
float32 accel
float32 velocity
uint32 wheelbase
uint32 x_cam2axle
uint32 hitch_height
uint32 coupler_height

# Hitch ball params
uint32 x_position
uint32 max_y
uint32 max_x_error

# CAN signal names
CANSignals can_signals

# CAN message names
uint32[] msg_filter
string swa_message
string scc_message
string gear_message
string driverbraking_message
string spd_message_whl
string gps_message
string imu_message
