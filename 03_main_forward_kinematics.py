
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from coppelia_utils import get_handles, get_joint_positions, get_joint_velocities, get_pose
from robotics_utils import ur5_dh_params, forward_kinematics
import numpy as np
import matplotlib.pyplot as plt

if __name__=="__main__":

    client = RemoteAPIClient()
    sim = client.require('sim') # This is the object that represents the simulation
    sim.setStepping(True)       # In this way the simulation is controlled via the step() method

    # Load the simulation scenario using the .ttt file 
    # NOTE: you have to put the absolute path!
    sim.loadScene("/home/nikolas/UNIPV/robot-control-unipv/coppelia_scenes/ur5_scene.ttt")

    # In order to access to any object of the scene, one must know the ID (or handle)
    # The handle can be accessed knowing the obejct name

    joint_names = ["/joint1", "/joint2", "/joint3", "/joint4", "/joint5", "/joint6"]    
    joint_handles = get_handles(sim=sim, names=joint_names)
    base_handle = get_handles(sim=sim, names=["/UR5"])[0]
    ee_handle = get_handles(sim=sim, names=["/end_effector"])[0]

    # Retrieve the pose of the robot base with respect to the world
    world_T_0 = get_pose(sim=sim, object_id=base_handle, respect_to=-1)

    # Setup some simulation settings 
    sim_duration = 5 # Simulation duration [s]
    sim_timestep = sim.getSimulationTimeStep()

    sim_iterations = int(sim_duration/sim_timestep)

    sim_history = {'time':np.zeros(sim_iterations),
                   'q':np.zeros((sim_iterations, len(joint_handles))),
                   'dq':np.zeros((sim_iterations, len(joint_handles))),
                   'pos_err_norm':np.zeros(sim_iterations),
                   'or_err_norm':np.zeros(sim_iterations)}


    
    # Start the simulation
    sim.startSimulation()
    
    for it in range(0, sim_iterations):
        # Retrieve joint positions and velocities
        q = get_joint_positions(sim=sim, joint_handles=joint_handles)
        dq = get_joint_velocities(sim=sim, joint_handles=joint_handles)

        # Retrieve the End-Effector pose from CoppeliaSim (for comparison only)
        world_T_ee_coppelia = get_pose(sim=sim, object_id=ee_handle, respect_to=-1) # Pose matrix retrieved by CoppeliaSim
        p_coppelia = world_T_ee_coppelia[0:3, 3]    

        # Compute the forward kinematics of the robot
        kinematic_chain = forward_kinematics(q=q, base_world_transform=np.eye(4), dh_params=ur5_dh_params)
        world_T_ee = kinematic_chain[-1]    # Pose matrix computed via FK        

        p_err_norm = np.linalg.norm(world_T_ee_coppelia[0:3, 3] - world_T_ee[0:3, 3])
        or_err_norm = np.linalg.norm(world_T_ee_coppelia[0:3,0:3] - world_T_ee[0:3,0:3])

        sim_history['time'][it] = it*sim_timestep
        sim_history['q'][it,:] = q
        sim_history['dq'][it,:] = dq
        sim_history['pos_err_norm'][it] = p_err_norm
        sim_history['or_err_norm'][it] = or_err_norm

        # Perform the simulation step
        sim.step()
    sim.stopSimulation()

    # Plot some results
    plt.figure()
    plt.plot(sim_history['time'], sim_history['pos_err_norm'], label="Position")
    plt.plot(sim_history['time'], sim_history['or_err_norm'], label="Orientation")
    plt.grid(True)
    plt.xlabel("Time [s]")
    plt.ylabel("Error norm")
    plt.legend()
    plt.show()