import numpy as np
from holoocean.agents import HoveringAUV
from scipy.spatial.transform import Rotation

g = 9.81 # gravity
b = 3 # linear damping
c = 2 # angular damping
HoveringAUV.mass += 1 # alternatively make it sink


def thrusters_to_body_forces(thruster_array):
    Fs = thruster_array[5]
    Fp = thruster_array[4]
    Ap = thruster_array[7]
    As = thruster_array[6]

    V_fs = thruster_array[0]*-1
    V_fp = thruster_array[1]*-1
    V_ap = thruster_array[2]*-1
    V_as = thruster_array[3]*-1

    # The assignments below should be the correct ones, but using the above to deal with the mismatch between +z down here, and +z up in holoocean
    # Fs = thruster_array[4]
    # Fp = thruster_array[5]
    # Ap = thruster_array[6]
    # As = thruster_array[7]

    # V_fs = thruster_array[0]
    # V_fp = thruster_array[1]
    # V_ap = thruster_array[2]
    # V_as = thruster_array[3]

    # === Thruster Command Inputs (Offset Mode) ===
    # These are signed offsets from neutral (81), valid range: [-21, +21]
    # Fp   = 15   # Forward Port
    # Fs   = 15   # Forward Starboard
    # Ap   = 0    # Aft Port
    # As   = 0    # Aft Starboard

    # V_fp = 0    # Vertical Forward Port
    # V_fs = 0    # Vertical Forward Starboard
    # V_ap = 5    # Vertical Aft Port
    # V_as = 5    # Vertical Aft Starboard

    # pwm_command = np.array([Fp, Fs, Ap, As, V_fp, V_fs, V_a])  # if V_a is used
    thruster_command = np.array([Fp, Fs, Ap, As, V_fp, V_fs, V_ap, V_as])

    # === Angles and positions ===
    alpha_f1 = 210; alpha_f2 = 150
    alpha_r1 = 150; alpha_r2 = 210

    y_V_fp = -0.2; x_V_fp = 0.5
    y_V_fs = 0.2;  x_V_fs = 0.5
    y_V_ap = -0.2; x_V_ap = -0.5
    y_V_as = 0.2;  x_V_as = -0.5

    y_Fp = -0.5; x_Fp = 0.3
    y_Fs = 0.5;  x_Fs = 0.3
    y_Ap = -0.5; x_Ap = -0.4
    y_As = 0.5;  x_As = -0.4

    # === Allocation matrix T (6x8) ===
    T = np.zeros((6,8))

    # SURGE
    T[0,0] = -np.cos(np.deg2rad(alpha_f1))
    T[0,1] = -np.cos(np.deg2rad(alpha_f2))
    T[0,2] = -np.cos(np.deg2rad(alpha_r1))
    T[0,3] = -np.cos(np.deg2rad(alpha_r2))

    # SWAY
    T[1,0] = -np.sin(np.deg2rad(alpha_f1))
    T[1,1] = -np.sin(np.deg2rad(alpha_f2))
    T[1,2] = -np.sin(np.deg2rad(alpha_r1))
    T[1,3] = -np.sin(np.deg2rad(alpha_r2))

    # HEAVE
    T[2,4] = -1
    T[2,5] = -1
    T[2,6] = -1

    # ROLL
    T[3,4] = -y_V_fp
    T[3,5] = -y_V_fs
    T[3,6] = -y_V_ap
    T[3,7] = -y_V_as

    # PITCH
    T[4,4] = x_V_fp
    T[4,5] = x_V_fs
    T[4,6] = x_V_ap
    T[4,7] = x_V_as

    # YAW
    T[5,0] = y_Fp * np.cos(np.deg2rad(alpha_f1)) - x_Fp * np.sin(np.deg2rad(alpha_f1))
    T[5,1] = y_Fs * np.cos(np.deg2rad(alpha_f2)) - x_Fs * np.sin(np.deg2rad(alpha_f2))
    T[5,2] = y_Ap * np.cos(np.deg2rad(alpha_r1)) - x_Ap * np.sin(np.deg2rad(alpha_r1))
    T[5,3] = y_As * np.cos(np.deg2rad(alpha_r2)) - x_As * np.sin(np.deg2rad(alpha_r2))

    # === Call your thrust conversion function ===
    # We may need to implement PWM_Thrust in Python
    # For now, let's mock it as identity, or *1
    def PWM_Thrust(pwm_command):

        return pwm_command.astype(float)

    thrusts = PWM_Thrust(thruster_command)  # 8x1 vector

    # === Calculate total forces and moments ===
    tau = T @ thrusts  # 6x1 vector: [X; Y; Z; K; M; N]

    return tau
    # labels = ['Surge (X)', 'Sway (Y)', 'Heave (Z)', 'Roll (K)', 'Pitch (M)', 'Yaw (N)']
    # for i in range(6):
    #     print(f'{labels[i]}: {tau[i]:.3f} N or N·m')


def global_hydro_forces(x):
    # Extract all info from state
    a = x[:3]
    v = x[3:6]
    p = x[6:9]
    alpha = x[9:12]
    omega = x[12:15]
    # quat = x[15:19]
    # R = Rotation.from_quat(quat).as_matrix()
    theta = x[15:18]
    R=Rotation.from_euler('xyz', theta, degrees=True).as_matrix()

    # Sum all forces
    force = np.zeros(3)
    force[2] += -HoveringAUV.mass * g # gravity
    force[2] += HoveringAUV.water_density * g * HoveringAUV.volume # buoyancy
    force -= v*b # Damping

    # Sum all torques
    torque = np.zeros(3)
    buoy_force = HoveringAUV.water_density*g*HoveringAUV.volume*np.array([0,0,1]) # in global frame
    cob = R@HoveringAUV.cob # move center of buoyancy to global frame
    torque += np.cross(cob, buoy_force) # torque from buoyancy
    torque -= omega*c # damping
    return np.append(force, torque)

    
def forces_to_accelerations(forces):
    force = forces[0:3]
    torque = forces[3:6]
    # Convert force & torque to accelerations
    lin_accel = force / HoveringAUV.mass
    ang_accel = np.linalg.inv(HoveringAUV.I)@torque
    return np.append(lin_accel, ang_accel)


