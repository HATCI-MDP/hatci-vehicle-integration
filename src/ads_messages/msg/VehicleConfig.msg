# VehicleConfig.msg
std_msgs/Header header

# Vehicle config
string vehicle_name
string camera_name

# Camera intrinsics and extrinsics
ExtraCameraInfo cam_intrinsics_extrinsics

# Vehicle params
float32 max_rwa_deg
float32 accel
float32 velocity
uint32 wheelbase
uint32 x_cam2axle

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
