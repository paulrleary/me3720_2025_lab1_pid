import numpy as np
import holoocean

from scipy.spatial.transform import Rotation
np.set_printoptions(suppress=True) # Suppress scientific notation in printing
import matplotlib.pyplot as plt
import time

from HoveringAUV_Physical_Model import global_hydro_forces, thrusters_to_body_forces, forces_to_accelerations
from PID_controllers import depth_pid_controller, heading_pid_controller, speed_pid_controller, lookahead_distance
from helper_functions import vert_force_to_thrusters, yaw_force_to_thrusters, control_angle_delta_degrees

scenario = {
    "name": "hovering_dynamics",
    "package_name": "Ocean",
    "world": "PierHarbor",
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
            # "location": [-50,0,-10],
            "location" : [800,-560,-60],
            "rotation": [0,0,5]
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
x_data = []
y1_ctl_data = []
y1_set_data = []
y2_ctl_data = []
y2_set_data = []


''
fig, (ax1, ax2) = plt.subplots(2,1,figsize=(10, 5))
line1_ctl, = ax1.plot(x_data, y1_ctl_data,color='blue')
line1_set, = ax1.plot(x_data, y1_set_data,color='black')
line2_ctl, = ax2.plot(x_data, y2_ctl_data,color='red')
line2_set, = ax2.plot(x_data, y2_set_data,color='black')

ax1.set_ylabel("Depth (m)")
ax1.set_title("Depth/Heading Control Plots")

ax2.set_xlabel("time (s)")
ax2.set_ylabel("Heading")


hydro_forces_global = np.zeros(6)

acceleration_global = np.zeros(6)

thruster_command = np.zeros(8)


# Make environment
def rotate_6dof_forces(forces, eulers):
    R=Rotation.from_euler('xyz', eulers, degrees=True).as_matrix()
    rotated_forces = np.zeros(6)
    rotated_forces[0:3] = R@forces[0:3]
    rotated_forces[3:6] = R@forces[3:6]

    return rotated_forces

depth_command = -50
heading_command = -15


with holoocean.make(scenario_cfg=scenario) as env:
    while True:
        
        # Step simulation
        simul_state = env.step(acceleration_global)
        
        # Get hydro forces from HoloOcean
        sensor_state = simul_state["DynamicsSensor"]
        hydro_forces_global = global_hydro_forces(sensor_state)

        auv_state = get_states_6dof(simul_state["DynamicsSensor"])
        
        st = auv_state["pose"]
        t = auv_state["time"]
        depth = st[2] #note that python uses "0 indexing"
        yaw = st[5]

        # INSERT HERE: Implementing PID, control for depth, heading and speed (in that order), where the goal is to drive the vehicle in a constant circle at depth.
        # normal_error, binormal_error, frenet_serret_frame = cte(st[:3], current_path[0], current_path[1])

        depth_error = depth - depth_command
        depth_control = depth_pid_controller(depth_error)

        # cross_track_error = normal_error
        # heading_setpoint = np.arctan2(lookahead_distance, cross_track_error) * 180 / np.pi
        
        heading_error = control_angle_delta_degrees(yaw, heading_command)
        heading_control = heading_pid_controller(heading_error)

        vert_thruster_command = np.zeros(8)
        heading_thruster_command = np.zeros(8)

        # RIGHT NOW, THE THRUSTERS ARE NOT ACTUATED, because PID has not yet been implemented.
        # vert_thruster_command = vert_force_to_thrusters(depth_control)   
        # heading_thruster_command = yaw_force_to_thrusters(heading_control)

        # speed_thruster_command = np.array([0, 0, 0, 0, 1, 1, 1, 1])*1
        speed_thruster_command = np.zeros(8)
        # print(f"Depth Error: {depth_error:.3f}, Depth Control: {depth_control:.3f}", Depth Command: {depth_command:.3f}")
        
        print(f"Heading Error: {heading_error:.3f}, Heading Control: {heading_control:.3f}, Yaw: {yaw:.3f}, Heading Command: {heading_command:.3f}")
        
        thruster_command = vert_thruster_command+heading_thruster_command+speed_thruster_command

        theta = st[3:6]

        thrust_forces_body = thrusters_to_body_forces(thruster_command)
    

        thrust_forces_global = rotate_6dof_forces(thrust_forces_body, theta)
        
        net_force_global = hydro_forces_global + thrust_forces_global

        acceleration_global = forces_to_accelerations(net_force_global)

        

        x_data.append(t)
        y1_ctl_data.append(depth)
        y1_set_data.append(depth_command)
        y2_ctl_data.append(yaw)
        y2_set_data.append(heading_command)

        line1_ctl.set_xdata(x_data)
        line1_ctl.set_ydata(y1_ctl_data)
        line1_set.set_xdata(x_data)
        line1_set.set_ydata(y1_set_data)

        line2_ctl.set_xdata(x_data)
        line2_ctl.set_ydata(y2_ctl_data)
        line2_set.set_xdata(x_data)
        line2_set.set_ydata(y2_set_data)


        ax1.relim()
        ax1.autoscale_view()
        ax2.relim()
        ax2.autoscale_view()

        fig.canvas.draw()
        fig.canvas.flush_events()

