# Robot Programming UNIPV
## What is the repository about?
This repository contains the source code for the implementation of some of the concepts teach in the Robot Control course at the University of Pavia. The simulations are carried out by means of the [CoppeliaSim](https://www.coppeliarobotics.com/) simulator. 

**NOTE:** This repository is still in development, more material will be added in the next moths. For any question or request, fell free to contact me at <nikolas.sacchi01@universitadipavia.it>  

## Prerequisites 
In order to run the simulations contained in this repository, you are need 
1. **CoppeliaSim**: the educational version can be installed following the instruction available [here](https://www.coppeliarobotics.com/);
2. **Python**:  a version > 3.8 of the Python interpreter can be downloaded and installed from [here](https://www.python.org/).

The interface between the simulator and the code is done via the **ZeroMQ remote API**, whose documentation is available [at this page](https://manual.coppeliarobotics.com/en/zmqRemoteApiOverview.htm). To use the API, you are required to install the Python library `coppeliasim_zmqremoteapi_client`. This can be done via the `pip` manager by means of the command
```
pip install coppeliasim_zmqremoteapi_client
```
The ZeroMQ remote API consists in a collection of functions that allow to access properties fo the simulation scene. Since at first could be difficult to chose which function to use, I have prepared some documented python functions, contained in the file `coppelia_utils.py`, which, exploiting the ZeroMQ remote API, allow to perform some basic operations with the simulator.  

In the scripts, some other python libraries, such as `numpy` and `matplotlib` will be used. I have collected all the dependencies in the file `requirements.txt`. To install them, just run 
```
pip install requirements.txt
```

## Usage
The repository contains different scripts, named with the format `xx_description.py`, where `xx` is an incremental number which indicates the order with which the scripts should be inspected. The higher the number of the script, the more advanced is the concept in this last one.

The functions employed for the computation of the kinematics, dynamics, etc., they are included in the `robotics_utils.py` module, while the utils for interfacng with the simulator are in the module `coppelia_utils.py`. 

The scripts makes reference to different scenarios, contained in the `coppelia_scenes` folder, and characterized by `.ttt` extension. All the scenarios contains the **UR5** robot, whose documentation containing the kinematics and dynamics parameters is available [here](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/). 


