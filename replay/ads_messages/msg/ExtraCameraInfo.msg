std_msgs/Header header

# Camera intrinsics
string camera_name
uint32 image_width
uint32 image_height
float64[9] camera_matrix
string distortion_model
float64[] distortion_coefficients
float64[9] rectification_matrix
float64[12] projection_matrix
float64 scale_factor
float64[5] ss_coeffs

# Camera to Checkerboard Extrinsics
float64[9] r_b_c
float64[3] t_b_c

# LiDAR to Camera
float64[9] lidar_to_camera_rotation_matrix
float64[3] lidar_to_camera_translation_vector

# Camera to Vehicle
float64[9] r_vc
float64[3] t_vc
float64[3] y_veh_in_cam
float64 cam_veh_yaw_offset_scale

# LiDAR to Vehicle
float64[9] r_vl
float64[3] t_vl
