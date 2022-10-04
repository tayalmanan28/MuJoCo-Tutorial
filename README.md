# Mujoco Tutorial
Tutorial on how to get started with MuJoCo simulations.

Currently we are also participating in Hacktober Fest 2022 so if you want to contribute to this repository please follow the contributing instructions given in the contributing Section [below](https://github.com/tayalmanan28/MuJoCo-Tutorial/blob/main/README.md#contributing)
![image](https://user-images.githubusercontent.com/42448031/193699422-a75d4807-e7ab-456a-9f57-e82195647c3b.png)


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

### MuJoCo Installation

For Installing MuJoCo on your system follow the [blog](https://tayalmanan28.github.io/my_blogs/mujoco/simulations/robotics/2022/01/21/MuJoCo.html)

### Running example

``` python3 run.py ```

## Contents

The main purpose of this repo is providing the starter code required to run a MuJoCo simulation with keyboard and mouse callbacks using its Python bindings. The base class is in `mujoco_base.py`. To create your own MuJoCo simulation, you can create a new class that inherits `mujoco_base.MuJoCoBase`. An example of this usage is provided in `example_projectile.py`, the new class should implement the functions

```[Python]
- reset()       # Initializes the enviroment and control callback
- controller()  # Adds control actions
- simulate()    # Copy the simulate() function from 
                # mujoco_base.MuJoCoBase and add your own twist
```

## MuJoCo Examples


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

## READMEs in different Languages

español: [Readme](https://github.com/tayalmanan28/MuJoCo-Tutorial/blob/main/Readme_sp.md)

## Contributing

So you can contribute to this repository in 2 ways: 
1. By adding new examples of MuJoCo environments
2. By help in resolving the existing issues

### For Contributing a new example to this repo:

- Fork this repository. You can fork this repository by clicking on fork button on top right corner. Once you fork this will create a copy of repo on your account
- Follow the above steps for installation 
- Go to the `examples` folder and go through different mujoco environment examples.
- Create a valid xml file. The instructions for making an XML File are mentioned [here](https://mujoco.readthedocs.io/en/latest/overview.html?highlight=hello.xml#examples)
- Then you can use one of the environments as a base to create a mujoco environment for your example and discuss if there are any issues.
- Once completed, create a pull request, read about submitting a pull request in the DigitalOcean tutorial "[How To Create a Pull Request on GitHub](https://www.digitalocean.com/community/tutorials/how-to-create-a-pull-request-on-github)".


### For Contributions directly from Issues:

- Fork this repository. You can fork this repository by clicking on fork button on top right corner. Once you fork this will create a copy of repo on your account
- Follow the above steps for installation 
- Based on your experience select an issue from the issues button above and ask for assigning the issue to you. Work on the issue and discuss it if you face any problems.
- Create a pull request, read about submitting a pull request in the DigitalOcean tutorial "[How To Create a Pull Request on GitHub](https://www.digitalocean.com/community/tutorials/how-to-create-a-pull-request-on-github)".

Soon, we will review your request and merge your pull requests to the main branch of project if your pull request is valid.  You will also get notification once your pull request is merged with existing code base. After that you will be able to see your details in contributor section on the page below.


## LICENSE

The code is licenced under the MIT license and free to use by anyone without any restrictions.
***

<p align='center'>Created with :heart: by <a href="https://github.com/tayalmanan28">Manan Tayal</a> and contributed by: <br>
<a href="https://github.com/bchainbuidler">Blockchain Buidler</a>,
<a href="https://github.com/Guillermo">Guiller</a>

