float32 setpoint_phi_dot    # commanded average  wheel vel ~ trans vel
float32 setpoint_gamma_dot  # commanded steering angle vel ~ angualr vel
float32 setpoint_phi        # commanded average wheel pos
float32 phi                 # average wheel pos
float32 setpoint_gamma      # commanded steering angle
float32 gamma               # steering angle
float32 setpoint_theta      # commanded body tilt
float32 theta               # body tilt
float32 d1_u                # control command for balnce loop
float32 d3_u                # control command for steering loop
float32 dutyL               # left  motor duty cycle
float32 dutyR               # right motor duty cycle

# 2017-02-22 LLW Added odometry data 
float32 wheel_angle_L       # total rotation of left  wheel (radians) (+ is forward)
float32 wheel_angle_R       # total rotation of right wheel (radians) (+ is forward)
float32 body_frame_easting  # displacemnt of body frame (m) (+ is East )
float32 body_frame_northing # displacemnt of body frame (m) (+ is North)
float32 body_frame_heading  # compass heading (radians) 

float32 vBatt               # battery voltage in volts
bool    armed               # controllers are active
bool    running             # balance program is running


