"""
This file allows to connect to the CoppeliaSim simulator and 
perform some simulation steps.
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from coppelia_utils import get_handles, get_joint_positions, get_joint_velocities

COPPELIA_SCENE_PATH = "/home/nikolas/UNIPV/robot-control-unipv/coppelia_scenes/ur5_scene.ttt"       # NOTE: you have to put the absolute path!

if __name__=="__main__":

    client = RemoteAPIClient()
    sim = client.require('sim') # This is the object that represents the simulation
    sim.setStepping(True)       # In this way the simulation is controlled via the step() method

    # Load the simulation scenario using the .ttt file 
    # NOTE: you have to put the absolute path!
    sim.loadScene(COPPELIA_SCENE_PATH)

    # In order to access to any object of the scene, one must know the ID (or handle)
    # The handle can be accessed knowing the obejct name

    joint_names = ["/UR5/joint{0}", "/UR5/joint{1}", "/UR5/joint{2}", "/UR5/joint{3}", 
                   "/UR5/joint{4}", "/UR5/joint{5}"] 
    joint_handles = get_handles(sim=sim, names=joint_names)

    # Setup some simulation settings 
    sim_duration = 5 # Simulation duration [s]
    sim_timestep = sim.getSimulationTimeStep()
    
    # Start the simulation
    sim.startSimulation()
    while (t := sim.getSimulationTime()) < sim_duration:
        # Retrieve joint positions and velocities
        q = get_joint_positions(sim=sim, joint_handles=joint_handles)
        dq = get_joint_velocities(sim=sim, joint_handles=joint_handles)

        # Print the retrieved information
        q_str = f"[{q[0]:.2f} {q[1]:.2f} {q[2]:.2f} {q[3]:.2f} {q[4]:.2f} {q[5]:.2f}]"
        dq_str = f"[{dq[0]:.2f} {dq[1]:.2f} {dq[2]:.2f} {dq[3]:.2f} {dq[4]:.2f} {dq[5]:.2f}]"
        print(f'[INFO] Time: {t:.2f} [s] -> q ={q_str} dq ={dq_str}')

        # Perform the simulation step
        sim.step()
    sim.stopSimulation()