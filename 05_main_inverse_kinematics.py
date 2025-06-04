import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from coppelia_utils import get_handles, get_joint_positions, get_pose, set_joint_target_positions
from robotics_utils import forward_kinematics, inverse_kinematics, ur5_dh_params 


COPPELIA_SCENE_PATH =  "/home/nikolas/UNIPV/robot-control-unipv/coppelia_scenes/ur5_scene.ttt"       # NOTE: you have to put the absolute path!
if __name__ == "__main__":
    client = RemoteAPIClient()
    sim = client.require('sim') # This is the object that represents the simulation
    sim.setStepping(True)       # In this way the simulation is controlled via the step() method

    # Load the simulation scenario using the .ttt file 
    # NOTE: you have to put the absolute path!
    #sim.loadScene(COPPELIA_SCENE_PATH)

    joint_names = ["/UR5/joint{0}", "/UR5/joint{1}", "/UR5/joint{2}", "/UR5/joint{3}", 
                   "/UR5/joint{4}", "/UR5/joint{5}"] 
    joint_handles = get_handles(sim=sim, names=joint_names)
    base_handle = get_handles(sim=sim, names=["/UR5"])[0]
    ee_handle = get_handles(sim=sim, names=["/UR5/link{5}"])[0]

    target_handle = get_handles(sim=sim, names=["/ik_target"])[0]

    # Retrieve the pose of the robot base with respect to the world
    world_T_0 = get_pose(sim=sim, object_id=base_handle, respect_to=-1)
    world_T_0[2,3] = world_T_0[2,3] - 0.01465   # This removes a small offset that is already taken care of in the DH params (must be done only for the UR5)


    # Get the current joint position, useful for the initialization of the IK algorithm
    q = get_joint_positions(sim=sim, joint_handles=joint_handles)


    # Define the desired pose 
    world_T_ee_des = get_pose(sim, target_handle, -1)

    # Compute the configuration associated with the desired pose with IK
    success, q_ik = inverse_kinematics(dh_params=ur5_dh_params, T_des=world_T_ee_des, q_first_guess=q,
                                       base_world_transform=world_T_0, conv_thresh=1e-3, max_iterations=1e4, 
                                       damping_factor=0.001, step_size=0.1, verbose=False)

    


    
    #jacobian = calc_full_jacobian_modified_dh(kinematic_chain)
    print(f"[INFO] q_ik : {q_ik}")

    sim.startSimulation()
    
    for it in range(0, 1000):
        set_joint_target_positions(sim=sim, joint_handles=joint_handles, q_des = q_ik)
        sim.step()

    sim.stopSimulation()
    print("[INFO] Simulation finished!")