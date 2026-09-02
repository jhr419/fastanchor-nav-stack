uint8 EXECUTE=0
uint8 CANCEL=1
uint8 HOLD=2

uint8 command
int32 order
int64 traj_id
builtin_interfaces/Time start_time

float64[] knots
geometry_msgs/Point[] pos_pts

float64[] yaw_pts
float64 yaw_dt
