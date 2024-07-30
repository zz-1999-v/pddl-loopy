### Citation Project
[pddlstream](https://github.com/caelan/pddlstream/tree/stable)

### Installation
```
$ git clone --recursive --branch main git@github.com:zz-1999-v/pddl-loopy.git
$ cd pddl-loopy
pddl-loopy$ git submodule update --init --recursive
pddl-loopy$ ./downward/build.py
```

### Examples

Install PyBullet on OS X or Linux using: 
```
$ pip install pybullet numpy scipy
```

Examples:
* LOOPY: `pddl-loopy$ python -m examples.pybullet.loopy.run`
