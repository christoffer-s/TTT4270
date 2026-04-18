import numpy as np

def tzyx(yaw, pitch):
    """
    Creates a Z-Y rotation matrix (equivalent to tzyx in many IMU toolboxes).
    """
    # Rotation about Y-axis (pitch)
    Ry = np.array([
        [np.cos(pitch), 0, -np.sin(pitch)],
        [0, 1, 0],
        [np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    # Rotation about Z-axis (yaw)
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # Combined rotation: Rz * Ry
    return Rz @ Ry