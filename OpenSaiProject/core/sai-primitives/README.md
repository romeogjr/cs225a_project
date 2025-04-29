# sai-primitives

This is a control library that provides an implementation of basic tasks and controllers for a torque controlled robot.
It uses the Operatinal Space framework.
It offers the possibility to be used with the [Ruckig](https://ruckig.com/) for trajectory generation (community version).

Robot controllers can be defined using different tasks:
* The MotionForce task is a cartesian control task, defined on a robot and a compliant frame:
    - It can be full 6dof (translation and rotation) or a subset of the cartesian motions (for example a planar transation and 1 rotation in that plane)
    - It can be set to use operational space control or impedance control
    - Any direction (defined in robot base frame or compliant frame) can be parametrized to perform force control instead of motion control
    - Force control can be open loop or closed loop with passivity based stabilization (https://ieeexplore.ieee.org/abstract/document/7989050, https://ieeexplore.ieee.org/abstract/document/8206036)
* The joint task implements PD control to control the joint positions of a torque controlled robot:
    - It can be full or defined on a subset of the joints
    - It can be parametrixed to use dynamic decoupling or not

The robot controller then takes a list of tasks and implements a hierarchical controller using all the tasks to compute the robot control torques

## Dependencies

sai-primitives depends on sai-model, Eigen3 and Ruckig.

The examples depend on additional libraries : sai-simulation, sai-graphics, sai-common.

## Build instructions

First build ruckig:

```
cd ruckig
mkdir build && cd build
cmake .. && make -j4
cd ../..
```

Next, build sai-primitives

```
mkdir build
cd build
cmake .. && make -j4
```

## Run the examples

Remember that you need sai-simulation, sai-graphics and sai-common in order to compile and run the examples.
Go to build/examples/desired_example and run the example. For example 1 :

```
cd build/examples/01-joint_control
./01-joint_control
```

## Documentation

The documentation can also be accessed online at the following [link](https://manips-sai-org.github.io/sai-primitives/)

It can also be generated locally with doxygen:

```
cd docs
doxygen
```

## License

This software is distributed under the terms of the Stanford Academic Software License Agreement For SAI
