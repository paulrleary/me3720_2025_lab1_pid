import numpy as np
import holoocean

from scipy.spatial.transform import Rotation
np.set_printoptions(suppress=True) # Suppress scientific notation in printing
import matplotlib.pyplot as plt
import time

from simple_pid import PID

from HoveringAUV_Physical_Model import global_hydro_forces, thrusters_to_body_forces, forces_to_accelerations
from Frenet_Serret_Error import frenet_seret_cross_track_error as cte
from PID_controllers import depth_pid, yaw_pid, speed_pid, lookahead_distance
from helper_functions import vert_force_to_thrusters, yaw_force_to_thrusters, speed_force_to_thrusters, control_angle_delta_degrees
scenario = {
    "name": "hovering_dynamics",
    "package_name": "Ocean",
    "world": "OpenWater",
    "main_agent": "auv0",
    "agents": [
        {
            "agent_name": "auv0",
            "agent_type": "HoveringAUV",
            "sensors": [
                {
                    "sensor_type": "DynamicsSensor",
                    # "configuration":{
                    #     "UseRPY": False # Use quaternion
                    # }
                },
            ],
            "control_scheme": 2, # this is the custom dynamics control scheme
            "location": [380,-775,-10],
            # "rotation": [20,20,90]
            "rotation": [0,0,90]
        }
    ]
}


start_time = time.time()
def get_states_6dof(dynamics_sensor_output_rpy):
    x = dynamics_sensor_output_rpy
    
    a = x[:3]
    v = x[3:6]
    p = x[6:9]
    alpha = x[9:12]
    omega = x[12:15]
    theta = x[15:18]

    pos = np.concatenate((p,theta))
    vel = np.concatenate((v,omega))
    acc = np.concatenate((a,alpha))
    t = time.time() - start_time

    state_6dof = dict(pose=pos, velocity=vel,acceleration=acc, time=t)

    return state_6dof

plt.ion()
pos_x_data = []
pos_y_data = []

track_x_data = []
track_y_data = []

fig, ax = plt.subplots()
position_line, = ax.plot(pos_x_data, pos_y_data)

track_line, = ax.plot([], [], 'r--', label='Track')

ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_title("Real-time Plot")


hydro_forces_global = np.zeros(6)

acceleration_global = np.zeros(6)

thruster_command = np.zeros(8)

# Comment these out to leave thruster command initialized to zero.
# thruster_command[0:4] = 3
# thruster_command[4:8] += 10


def rotate_6dof_forces(forces, eulers):
    R=Rotation.from_euler('xyz', eulers, degrees=True).as_matrix()
    rotated_forces = np.zeros(6)
    rotated_forces[0:3] = R@forces[0:3]
    rotated_forces[3:6] = R@forces[3:6]

    return rotated_forces

from Tracks import hourglass_small as track_list

def distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

# Make environment
with holoocean.make(scenario_cfg=scenario) as env:
    goal_idx = 1
    

    for wp in track_list:
        track_x_data.append(wp[0])
        track_y_data.append(wp[1])
    
    track_line.set_xdata(track_x_data)
    track_line.set_ydata(track_y_data)

   
    while True:

        
        current_path = [track_list[goal_idx-1],
                         track_list[goal_idx]]
        
        
        # Step simulation
        simul_state = env.step(acceleration_global)
        
        # Get hydro forces from HoloOcean
        sensor_state = simul_state["DynamicsSensor"]
        hydro_forces_global = global_hydro_forces(sensor_state)
        # print(hydro_forces_global)
        auv_state = get_states_6dof(simul_state["DynamicsSensor"])
        
        pose = auv_state["pose"]
        vel = auv_state["velocity"]
                
        normal_error, binormal_error, frenet_serret_frame = cte(pose[:3], current_path[0], current_path[1])
        D = distance(pose[:2], current_path[1][:2]) # distance to the next waypoint, horizontal only

        # INSERT HERE: using your PID controllers, deriving errors from the Frenet-Serret frame above, control the vehcle to follow the track in 3 dimensions.
        # You will need to involve logic to decide when to switch to the next waypoint.  We will say for our purposes that the maximum "watch circle" for each waypoint is 5m.
        # You may need to control speed based on distance to the next waypoint.
        depth_error = 0
        depth_control = 0
        heading_error = 0
        heading_control = 0
        speed_command = 0

        print(f"Normal Error: {normal_error:.3f}, Heading Error: {heading_error:.3f}, Heading Control: {heading_control:.3f}, Binormal Error: {binormal_error:.3f}, Depth Error: {depth_error:.3f},  Depth Control: {depth_control:.3f}")

        vert_thruster_command = vert_force_to_thrusters(depth_control)   
        heading_thruster_command = yaw_force_to_thrusters(heading_control)
        speed_thruster_command = speed_force_to_thrusters(speed_command)

        thruster_command = vert_thruster_command+heading_thruster_command+speed_thruster_command



        theta = pose[3:6]
        pos = pose[0:3]

        thrust_forces_body = thrusters_to_body_forces(thruster_command)
        # print(thrust_forces_body)

        thrust_forces_global = rotate_6dof_forces(thrust_forces_body, theta)
        
        net_force_global = hydro_forces_global + thrust_forces_global

        acceleration_global = forces_to_accelerations(net_force_global)

        if D < 5:
            goal_idx += 1
            if goal_idx >= len(track_list):
                print('Finished track!')
                break
            else:
                print('Goal reached! Next Waypoint!')
    

        pos_x_data.append(pose[0])
        pos_y_data.append(pose[1])

        position_line.set_xdata(pos_x_data)
        position_line.set_ydata(pos_y_data)

        ax.relim()
        ax.autoscale_view()

        fig.canvas.draw()
        fig.canvas.flush_events()
