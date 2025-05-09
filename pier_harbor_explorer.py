import holoocean
import numpy as np
from pynput import keyboard
import matplotlib.pyplot as plt
import time
np.set_printoptions(suppress=True) # Suppress scientific notation in printing

from helper_functions import vert_force_to_thrusters, yaw_force_to_thrusters

scenario = {
    "name": "explorer",
    "world": "PierHarbor",
    "package_name": "Ocean",
    "main_agent": "auv0",
    "ticks_per_sec": 10,
    "agents": [
        {
            "agent_name": "auv0",
            "agent_type": "HoveringAUV",
            "sensors": [
                {
                    "sensor_type": "RGBCamera",
                    "socket": "CameraSocket",
                    "configuration": {
                        "CaptureWidth": 512,
                        "CaptureHeight": 512
                    }
                },
                {
                    "sensor_type": "DynamicsSensor",
                    # "configuration":{
                    #     "UseRPY": False # Use quaternion
                    # }
                },
            ],
            "control_scheme": 0,
            "location": [800,-560,-5],
            # "location": [800,-10,-5],
            "rotation": [0, 0, -130]
        }
    ]
}


depth_command = -100
heading_command = 146

pressed_keys = list()
force = 25

def on_press(key):
    global pressed_keys
    if hasattr(key, 'char'):
        pressed_keys.append(key.char)
        pressed_keys = list(set(pressed_keys))

def on_release(key):
    global pressed_keys
    if hasattr(key, 'char'):
        pressed_keys.remove(key.char)

listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()

def parse_keys(keys, val):
    command = np.zeros(8)
    if 'i' in keys:
        command[0:4] += val
    if 'k' in keys:
        command[0:4] -= val
    if 'j' in keys:
        command[[4,7]] += 0.25*val
        command[[5,6]] -= 0.25*val
    if 'l' in keys:
        command[[4,7]] -= 0.25*val
        command[[5,6]] += 0.25*val

    if 't' in keys:
        command[4:8] += 3*val
    if 'g' in keys:
        command[4:8] -= 2*val
    if 'f' in keys:
        command[[4,6]] += val
        command[[5,7]] -= val
    if 'h' in keys:
        command[[4,6]] -= val
        command[[5,7]] += val

    return command

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

# track_line, = ax.plot([], [], 'r--', label='Track')

ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_title("Real-time Plot")



z_force = 0
psi_force = 0
with holoocean.make(scenario_cfg=scenario) as env:
    while True:
        if 'q' in pressed_keys:
            break
        
        # # KEY PRESS DRIVE CAN BE UNCOMMENTED FOR TROUBLESHOOTING, BUT MAY CAUSE CONFUSION WITH OTHER COMMANDS
        command = parse_keys(pressed_keys, force) # command is a numpy array of length 8, corresponding to the vehicle's 8 thrusters.  thruster values are in Newtons
        # print(command)

        

        # INSERT HERE: command vehicle to drive to depth = depth_command and yaw = heading_command, by updating z_force and psi_force.
        
        # Goal is to get to desired depth and heading as rapidly as possible, but also without colliding with bottom
        # x_force_to_thrusters commands serve to distribute commanded force amongst four relevant thrusters. Familiarize yourself with the import structure which allows definition of these functions in a separate file.
        # Note: check on sign of psi force.  if yaw_thruster_cmd appears to drive yaw in opposite of intended direction, multiply psi_force by -1, but also discuss/share
        
        # Warning: uncommenting these values may make you seasick.
        # z_force = -160
        # psi_force = 40

        # depth_thruster_cmd = vert_force_to_thrusters(z_force)
        # yaw_thruster_cmd = yaw_force_to_thrusters(psi_force)
        # command = depth_thruster_cmd+yaw_thruster_cmd
        # print(command)

        #send to holoocean
        env.act("auv0", command)
        simul_state = env.tick()
        
        auv_state = get_states_6dof(simul_state["DynamicsSensor"])
        
        st = auv_state["pose"] # auv_state is a python dictionary with pose velocity and acceleration in 6DOF, as well as a timestamp in seconds since the beginning of execution.  right now, all states are in global coordinates
        # print(st) #uncomment to view the states of the vehicle

        print(st)
        t = auv_state["time"]

        depth = st[2] #note that python uses "0 indexing"
        yaw = st[5]

        
        pos_x_data.append(st[0])
        pos_y_data.append(st[1])
        position_line.set_xdata(pos_x_data)
        position_line.set_ydata(pos_y_data)

        ax.relim()
        ax.autoscale_view()

        fig.canvas.draw()
        fig.canvas.flush_events()






