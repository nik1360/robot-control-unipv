import numpy as np
from typing import Tuple



def get_handles(sim, names:list) -> list:
    handles = []
    for name in names:
        handle = sim.getObject(name)
        handles.append(handle)
    return handles

def get_joint_positions(sim, joint_handles:list) -> list:
    joint_positions = []
    for handle in joint_handles:
        pos = sim.getJointPosition(handle)
        joint_positions.append(pos)
    return joint_positions

def get_joint_velocities(sim, joint_handles:list) -> list:
    joint_velocities = []
    for handle in joint_handles:
        vel = sim.getJointVelocity(handle)
        joint_velocities.append(vel)
    return joint_velocities






"""
Function that returns the rotation matrix and the position vector of an object w.r.t. the world 
reference frame.
"""
def get_pose(sim, object_id:int, respect_to:int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    position = sim.getObjectPosition(object_id, respect_to)
    orientation = sim.getObjectOrientation(object_id, respect_to)

    matrix_list = sim.buildMatrix(position, orientation) + [0, 0, 0, 1]
    matrix = np.reshape(matrix_list, (4,4))

    return matrix

def matrix_to_euler(sim, T:np.ndarray) -> Tuple[int, int, int]:
    return sim.getEulerAnglesFromMatrix(T[0:3, 0:4].flatten().tolist())

def set_object_pose(sim, object_id:int, T:np.ndarray):
    sim.setObjectMatrix(object_id, T[0:3, 0:4].flatten().tolist())