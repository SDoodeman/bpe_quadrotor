std_msgs/Header header
# Reference signals as seen from drone i
uint8 drone_id
float64 total_time
# Desired reference for the leader drone
float64[3] pdes0
float64[3] vdes0
float64[3] udes0
float64[3] jdes0
# Desired reference for follower 1
float64[3] pdes1
float64[3] vdes1
float64[3] udes1
float64[3] jdes1
# Desired reference for follower 2
float64[3] pdes2
float64[3] vdes2
float64[3] udes2
float64[3] jdes2
# Current state of the system seen from drone i
# State of the leader
float64[3] p0
float64[3] v0
# State of the first follower
float64[3] p1
float64[3] v1
# State of the second follower
float64[3] p2
float64[3] v2
# Position error
float64[3] pos_error
float64[3] vel_error
# Desired system inputs
float64[3] desired_acceleration
float64 thrust_reference
# Desired attitude of the drone (in deg)
float64 desired_roll
float64 desired_pitch
float64 desired_yaw
# Current attitude of the drone (in deg)
#float64 state_roll
#float64 state_pitch
#float64 state_yaw