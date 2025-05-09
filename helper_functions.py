import numpy as np

def vert_force_to_thrusters(force):
    ind_force = force/4
    
    command = np.zeros(8)
    command[0:4] = ind_force

    return command

# def yaw_force_to_thrusters(force):
#     ind_force = force/4
    
#     command = np.zeros(8)
#     command[4] = -1*ind_force
#     command[5] = ind_force
#     command[6] = ind_force
#     command[7] = -1*ind_force

def yaw_force_to_thrusters(force):
    ind_force = force/4
    
    command = np.zeros(8)
    command[4] = ind_force
    command[5] = -1*ind_force
    command[6] = -1*ind_force
    command[7] = -ind_force

    return command

def speed_force_to_thrusters(force):
    ind_force = force/4
    
    command = np.zeros(8)
    command[4:8] = ind_force
  
    return command

def control_angle_delta_degrees(current_angle, set_angle):
    """
    Calculate the difference between two angles in degrees.
    
    Parameters:
    current_angle (float): First angle in degrees.
    set_angle (float): Second angle in degrees.
    
    Returns:
    delta_angle (float): The difference between the two angles in degrees.
    """
    delta_angle = (current_angle - set_angle + 180) % 360 - 180
    return delta_angle