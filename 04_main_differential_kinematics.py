
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from coppelia_utils import get_handles, get_joint_positions, get_joint_velocities, get_pose, get_object_velocity
from robotics_utils import ur5_dh_params, forward_kinematics, geometric_jacobian
import numpy as np
import matplotlib.pyplot as plt



COPPELIA_SCENE_PATH = "/home/nikolas/UNIPV/robot-control-unipv/coppelia_scenes/ur5_scene.ttt"       # NOTE: you have to put the absolute path!

if __name__=="__main__":

    client = RemoteAPIClient()
    sim = client.require('sim') # This is the object that represents the simulation
    sim.setStepping(True)       # In this way the simulation is controlled via the step() method

    # Load the simulation scenario using the .ttt file 
    
    sim.loadScene(COPPELIA_SCENE_PATH)

    # In order to access to any object of the scene, one must know the ID (or handle)
    # The handle can be accessed knowing the obejct name

    joint_names = ["/UR5/joint{0}", "/UR5/joint{1}", "/UR5/joint{2}", "/UR5/joint{3}", 
                   "/UR5/joint{4}", "/UR5/joint{5}"] 
    joint_handles = get_handles(sim=sim, names=joint_names)
    base_handle = get_handles(sim=sim, names=["/UR5"])[0]
    ee_handle = get_handles(sim=sim, names=["/UR5/link{5}"])[0]

    # Retrieve the pose of the robot base with respect to the world
    world_T_0 = get_pose(sim=sim, object_id=base_handle, respect_to=-1)
    world_T_0[2,3] = world_T_0[2,3] - 0.01465   # This removes a small offset that is already taken care of in the DH params (must be done only for the UR5)

    # Setup some simulation settings 
    sim_duration = 5 # Simulation duration [s]
    sim_timestep = sim.getSimulationTimeStep()

    sim_iterations = int(sim_duration/sim_timestep)

    sim_history = {'time':np.zeros(sim_iterations),
                   'q':np.zeros((sim_iterations, len(joint_handles))),
                   'dq':np.zeros((sim_iterations, len(joint_handles))),
                   'v_ee_dk':np.zeros((sim_iterations, 6)),
                   'v_ee_coppelia':np.zeros((sim_iterations, 6))}
    
    # Start the simulation
    sim.startSimulation()
    
    for it in range(0, sim_iterations):
        # Retrieve joint positions and velocities
        q = get_joint_positions(sim=sim, joint_handles=joint_handles)
        dq = get_joint_velocities(sim=sim, joint_handles=joint_handles)

        # Compute the forward kinematics of the robot
        kinematic_chain = forward_kinematics(q=q, base_world_transform=world_T_0, dh_params=ur5_dh_params)
        
        # Compute the Jacobian matrix
        J = geometric_jacobian(kinematic_chain=kinematic_chain, q=q)

        # Compute the EE velocity vector (6x1) via differential kinematics
        v_ee_dk = J@np.array(dq)

        v_ee_coppelia = get_object_velocity(sim=sim, handle=ee_handle)

        sim_history['time'][it] = it*sim_timestep
        sim_history['q'][it,:] = q
        sim_history['dq'][it,:] = dq
        sim_history['v_ee_dk'][it,:] = v_ee_dk
        sim_history['v_ee_coppelia'][it,:] = v_ee_coppelia

        # Perform the simulation step
        sim.step()
    sim.stopSimulation()



    # Plot some results
    plt.rcParams['text.usetex'] = True

    labels = ["$v_{x}$", "$v_{y}$", "$v_{z}$", "$\omega_{\\alpha}$", "$\omega_{\\beta}$", "$\omega_{\gamma}$"]
    fig, axes = plt.subplots(nrows=6, ncols=1, figsize = (6, 2*6))

    for i in range(0, 6):
        axes[i].plot(sim_history['time'], sim_history['v_ee_dk'][:, i], label="DK")
        axes[i].plot(sim_history['time'], sim_history['v_ee_coppelia'][:,i], label="Coppelias")
        axes[i].set(xlabel='Time [s]', ylabel=labels[i])
        axes[i].grid(True)
        axes[i].legend(loc="upper right")

    plt.suptitle('EE Velocity', fontsize=18, fontweight="bold")
    plt.tight_layout()
    plt.show()