import numpy as np

def frenet_seret_cross_track_error(position, path_start, path_end):
    """
    Compute the Frenet-Serret frame and cross-track errors for a vehicle following a path in 3D space.
    
    Parameters:
    position (np.array): The position of the vehicle in 3D space.
    path_start (np.array): The start point of the path.
    path_end (np.array): The end point of the path.
    
    Returns:
    T (np.array): Tangent vector of the Frenet-Serret frame.
    N (np.array): Normal vector of the Frenet-Serret frame.
    B (np.array): Binormal vector of the Frenet-Serret frame.
    normal_error (float): Cross-track error in the normal direction.
    binormal_error (float): Cross-track error in the binormal direction.
    """
    # Compute tangent (T)
    T = path_end - path_start
    T = T / np.linalg.norm(T)

    # Compute normal (N) parallel to x-y plane: N = [-Ty, Tx, 0]
    N = np.array([-T[1], T[0], 0])
    if np.linalg.norm(N) != 0:
        N = N / np.linalg.norm(N)
    else:
        # If T is vertical, set N along x-axis
        N = np.array([1, 0, 0])

    # Compute binormal (B)
    B = np.cross(T, N)
    B = B / np.linalg.norm(B)

    # Project vehicle position onto path to find closest point
    AP = position - path_start
    proj_length = np.dot(AP, T)
    closest_pt = path_start + proj_length * T

    # Compute error vector
    error_vec = position - closest_pt

    # Cross-track errors
    normal_error = np.dot(error_vec, N)
    binormal_error = np.dot(error_vec, B)

    frenet_serret_frame = np.array([T, N, B])
    # Check if the frame is orthonormal
    # if not np.allclose(np.dot(T, N), 0) or not np.allclose(np.dot(T, B), 0) or not np.allclose(np.dot(N, B), 0):
    #     raise ValueError("Frenet-Serret frame is not orthonormal")
    # if not np.allclose(np.linalg.norm(T), 1) or not np.allclose(np.linalg.norm(N), 1) or not np.allclose(np.linalg.norm(B), 1):
    #     raise ValueError("Frenet-Serret frame vectors are not unit vectors")

    return normal_error, binormal_error, frenet_serret_frame

