"""
This file allows to connect to the CoppeliaSim simulator,
perform some simulation steps and read the some information 
from the robot joint
"""

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

if __name__=="__main__":

    client = RemoteAPIClient()
    sim = client.require('sim') # This is the object that represents the simulation
    sim.setStepping(True)       # In this way the simulation is controlled via the step() method

    # Load the simulation scenario using the .ttt file 
    # NOTE: you have to put the absolute path!
    sim.loadScene("/home/nikolas/UNIPV/robot-control-unipv/coppelia_scenes/ur5_scene.ttt")

    # Setup some simulation settings 
    sim_duration = 5 # Simulation duration [s]
    sim_timestep = sim.getSimulationTimeStep()
    
    # Start the simulation
    sim.startSimulation()
    while (t := sim.getSimulationTime()) < sim_duration:
        print(f'[INFO] Simulation time: {t:.2f} [s]')
        sim.step()
    sim.stopSimulation()