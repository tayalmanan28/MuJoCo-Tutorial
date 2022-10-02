# Mujoco Tutorial
Tutorial on MuJoCo

## Installation and Getting started:

### Conda Environment Setup

To install Anaconda follow the instructions in this [webpage](https://www.digitalocean.com/community/tutorials/how-to-install-the-anaconda-python-distribution-on-ubuntu-20-04-quickstart) (Ubuntu 20.04)

Create a conda environment for MuJoCo setup:  
```
conda create --name mujoco-tut  
```
Switch to the newly create environment (you will notice the name of the environment on the command line in the extreme left):  
```
conda activate mujoco-tut  
```

Then, clone the repository on your system:
```
git clone https://github.com/tayalmanan28/Mujoco-Tutorial.git
```
Install the following required packages:
```
pip install -r requirements.txt
```

### Checking MuJoCo Installation

For Installing MuJoCo on your system follow the [blog](https://tayalmanan28.github.io/my_blogs/mujoco/simulations/robotics/2022/01/21/MuJoCo.html)

## Contents

The main purpose of this repo is providing the starter code required to run a MuJoCo simulation with keyboard and mouse callbacks using its Python bindings. The base class is in `mujoco_base.py`. To create your own MuJoCo simulation, you can create a new class that inherits `mujoco_base.MuJoCoBase`. An example of this usage is provided in `example_projectile.py`, the new class should implement the functions

```[Python]
- reset()       # Initializes the enviroment and control callback
- controller()  # Adds control actions
- simulate()    # Copy the simulate() function from 
                # mujoco_base.MuJoCoBase and add your own twist
```

## MuJoCo Bootcamp Examples


```[Markdown]
- Projectile with drag
- Control a simple pendulum
- Control a double pendulum
- Leg swing
- Manipulator drawing
- Control an underactuated pendulum
- Gymnast swing/release on a bar
- 2D Hopper
- Initial Value Problem
- Inverse Kinematics
- 2D Biped
```

