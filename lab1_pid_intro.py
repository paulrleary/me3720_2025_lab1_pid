import numpy as np
import holoocean

from scipy.spatial.transform import Rotation
np.set_printoptions(suppress=True) # Suppress scientific notation in printing
import matplotlib.pyplot as plt
import time

from HoveringAUV_Physical_Model import global_hydro_forces, thrusters_to_body_forces, forces_to_accelerations

scenario = {
    "name": "hovering_dynamics",
    "package_name": "Ocean",
    "world": "SimpleUnderwater",
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
            "location": [0,0,-10],
            "rotation": [20,20,90]
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
y_data = []

fig, ax = plt.subplots()
line, = ax.plot(x_data, y_data)

ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_title("Real-time Plot")


hydro_forces_global = np.zeros(6)

acceleration_global = np.zeros(6)

thruster_command = np.zeros(8)

# Comment these out to leave thruster command initialized to zero.
# thruster_command[0:4] = 0.1
# thruster_command[[4,7]] += 0.4

# Make environment
def rotate_6dof_forces(forces, eulers):
    R=Rotation.from_euler('xyz', eulers, degrees=True).as_matrix()
    rotated_forces = np.zeros(6)
    rotated_forces[0:3] = R@forces[0:3]
    rotated_forces[3:6] = R@forces[3:6]

    return rotated_forces


with holoocean.make(scenario_cfg=scenario) as env:
    while True:
        
        # Step simulation
        simul_state = env.step(acceleration_global)
        
        # Get hydro forces from HoloOcean
        sensor_state = simul_state["DynamicsSensor"]
        hydro_forces_global = global_hydro_forces(sensor_state)

        auv_state = get_states_6dof(simul_state["DynamicsSensor"])
        
        st = auv_state["pose"]

        # INSERT HERE: Implementing PID, control for depth, heading and speed (in that order), where the goal is to drive the vehicle in a constant circle at depth.

        theta = st[3:6]

        thrust_forces_body = thrusters_to_body_forces(thruster_command)
        print(thrust_forces_body)

        thrust_forces_global = rotate_6dof_forces(thrust_forces_body, theta)
        
        net_force_global = hydro_forces_global + thrust_forces_global

        acceleration_global = forces_to_accelerations(net_force_global)

        x_data.append(st[0])
        y_data.append(st[1])

        line.set_xdata(x_data)
        line.set_ydata(y_data)

        ax.relim()
        ax.autoscale_view()

        fig.canvas.draw()
        fig.canvas.flush_events()
