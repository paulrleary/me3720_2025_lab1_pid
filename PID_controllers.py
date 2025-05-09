import numpy as np 

# For each PID controller, explore the effects of each of the three parameters (Kp, Ki, Kd) on the system's response.
# Also explore the effects of "windup" and "anti-windup" on the system's response, which is to say the effects of time range of the integral and derivative terms.
# For CTE, also explore the effects of the "lookahead distance" on the system's response.
# Finally speed control may make more sense to function off a set point rather than an error, but you can try your own approach.

def depth_pid_controller(error_value):
    # Here you can implement your PID control logic for depth
    return 1*error_value
 
def heading_pid_controller(error_value):
    # Here you can implement your PID control logic for heading
    return 1*error_value

def speed_pid_controller(error_value):
    # Here you can implement your PID control logic for speed
    return 1*error_value

lookahead_distance = 0.0
