# Citation Project
**pddlstream** branch ([main](https://github.com/caelan/pddlstream/tree/stable)) 

## Installation

<!--$ git clone --recursive --branch main https://github.com/caelan/pddlstream.git-->
```
$ git clone --recursive --branch main git@github.com:caelan/pddlstream.git
$ cd pddlstream
pddlstream$ git submodule update --init --recursive
pddlstream$ ./downward/build.py
```
<!--```
$ git clone --recursive https://github.com/caelan/pddlstream.git

If building fails, install FastDownward's dependencies using your package manager:
* APT (Linux): `$ sudo apt-get install cmake g++ g++-multilib make python`
<!--* Homebrew (OS X): TBD
* MacPorts (OS X): TBD
* N/A (Windows): install each dependency manually-->

If necessary, see FastDownward's [documentation](http://www.fast-downward.org/ObtainingAndRunningFastDownward) for more detailed installation instructions.

<!--My FastDownward "fork" is several years old. If you have trouble compiling FastDownward on a newer machine, try installing the experimental [downward](https://github.com/caelan/pddlstream/tree/downward) PDDLStream branch.-->

PDDLStream actively supports python2.7 as well as the most recent version of python3.
<!--(many robotics applications still require python2.7)-->

Make sure to recursively update **pddlstream**'s submodules when pulling new commits.
```
pddlstream$ git pull --recurse-submodules
```

## Examples

This repository contains several robotic and non-robotic PDDLStream example domains.

### PyBullet

Install PyBullet on OS X or Linux using: 
```
$ pip install pybullet numpy scipy
```

Examples:
* LOOPY: `pddlstream$ python -m examples.pybullet.loopy.run`
