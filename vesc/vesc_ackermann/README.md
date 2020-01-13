# ackermann_to_vesc_node
parameters: 
* /vesc/speed_to_erpm_gain
* /vesc/speed_to_erpm_offset
* /vesc/steering_angle_to_servo_gain
* /vesc/steering_angle_to_servo_offset

subscribed topics: 
* /vesc/ackermann_cmd_throttled

published topics: 
* /vesc/commands/motor/speed
* /vesc/commands/servo/position

# ackermann_to_odom_node
parameters: 
* /vesc/speed_to_erpm_gain
* /vesc/speed_to_erpm_offset
* /vesc/steering_angle_to_servo_gain
* /vesc/steering_angle_to_servo_offset

subscribed topics: 
* /vesc/sensors/core
* /vesc/sensors/servo_position_command

published topics: 
* /vesc/odom
