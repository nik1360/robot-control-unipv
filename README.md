# Robot Programming UNIPV
## What is the repository about?
This repository contains the source code for the implementation of some of the concepts teach in the Robot Control course at the University of Pavia. The simulations are carried out by means of the [CoppeliaSim](https://www.coppeliarobotics.com/) simulator. 

## Prerequisites 
In order to run the simulations contained in this repository, you are need 
1. **CoppeliaSim**: the educational version can be installed following the instruction available [here](https://www.coppeliarobotics.com/);
2. **Python**:  a version > 3.8 of the Python interpreter can be downloaded and installed from [here](https://www.python.org/).

The interface between the simulator and the code is done via the **ZeroMQ remote API**, whose documentation is available [at this page](https://manual.coppeliarobotics.com/en/zmqRemoteApiOverview.htm). To use the API, you are required to install the Python library `coppeliasim_zmqremoteapi_client`. This can be done via the `pip` manager by means of the command
```
pip install coppeliasim_zmqremoteapi_client
```
The ZeroMQ remote API consists in a collection of functions that allow to access properties fo the simulation scene. Since at first could be difficult to chose which function to use, I have prepared some documented python functions, contained in the file `coppelia_utils.py`, which, exploiting the ZeroMQ remote API, allow to perform some basic operations with the simulator.  



