Sai-Interface config files details
===================================

The config files used to parametrize the simulation and/or controller are custom xml files.

Such a config file can contain one `<redisConfiguration>` , one `<simvizConfiguration>` element, and several `<robotControlConfiguration>` elements (one per robot to control).
We will go over those in details using the two example files [ `aa_detailled_panda_simviz_only.xml` ](https://github.com/manips-sai-org/sai-interfaces/blob/master/examples/config_files/aa_detailled_panda_simviz_only.xml) and [ `aa_detailled_panda_control_only.xml` ](https://github.com/manips-sai-org/sai-interfaces/blob/master/examples/config_files/aa_detailled_panda_control_only.xml).

If we want to control a robot task using a haptic device, the config file can also contain one or several `<hapticDeviceControlConfiguration>` elements. A third example file details this: [`aa_detailled_panda_haptic_control.xml`](https://github.com/manips-sai-org/sai-interfaces/blob/master/examples/config_files/aa_detailled_panda_haptic_control.xml)

### The `<redisConfiguration>` element

Let's start with the redis configuration. This one is very simple. It is optionnal, but if present, only one can be present. It has 3 potential attributes to define the namespace prefix for redis keys, the ip address and port of the redis server.

```
<!-- 
The <redisConfiguration> element contains the configuration for the redis clients used by the controllers (and the simviz if present).
Only one is allowed per configuration file.
Its attributes are:
	- namespacePrefix: Optional. The prefix that will be added to all keys. Defaults to "sai::interfaces"
	- ip: Optional. The ip address of the redis server. Defaults to 127.0.0.1 (localhost)
	- port: Optional. The port of the redis server. Defaults to 6379
None of the corresponding config parameters can be changed at runtime.
-->
<redisConfiguration namespacePrefix="sai::interfaces" 
	ip="127.0.0.1" 
	port="6379" />
```

### The `<simvizConfiguration>` element

Let's continue with the simviz configuration. If we want an application with a simulation or rendering, we need to set a simviz configuration in the config file by using the `<simvizConfiguration>` element. We will go step by step over the [commented example file](https://github.com/manips-sai-org/sai-interfaces/blob/master/examples/config_files/aa_detailled_panda_simviz_only.xml) to explain the possible attributes and children elements. The only required attribute is the `worldFilePath` that provides the path to the world file defining the virtual world. For more details on the world file, see the [example world file](https://github.com/manips-sai-org/sai-interfaces/blob/master/examples/world/panda_world.urdf)

```
<!-- 
The <simvizConfiguration> element contains the configuration for the simulation and visualization of
the virtual world. Only one is allowed per configuration file.
Its attributes are:
	- worldFilePath: Required. The path to the world file to be used
	- mode: Optional. The mode of the simulation. Possible values are simviz, simOnly, vizOnly. Defaults to simviz
	- publishMassMatrixToRedis: Optional. Whether to publish the mass matrix to redis for all simulated robots. Defaults to true.
None of the corresponding config parameters can be changed at runtime.
-->
<simvizConfiguration worldFilePath="${WORLD_FILES_FOLDER}/world_panda.urdf"
	mode="simviz"
	publishMassMatrixToRedis="true">

...

</simvizConfiguration>
```

Let's go over its possible children elements.

#### The `<simParameters>` sub element

This child element of `<simvizConfiguration>` contains all the parameters such as the friction coefficient, the timestep of the simulation and so on. It is optional and all its attributes are optional. The missing attributes will use default values. The comments provide all the details:

```
<!-- 
The optional <simParameters> element contains the parameters for the simulation. Only one is allowed per simvizConfiguration element.
If it is not present, the default values are used.
Its attributes are:
	- timestep: Optional - Fixed. The timestep of the simulation. Defaults to 0.001
	- speedupFactor: Optional - Fixed. The speedup factor of the simulation. a value of 2 means that 1 second of real time 
		would correspond to 2 seconds in the simulation. Defaults to 1.0
	- enableJointLimits: Optional - Fixed. Whether to enable joint limits. Defaults to true
	- enableGravityCompensation: Optional. Whether to enable gravity compensation on robots. Defaults to true
	- frictionCoefficient: Optional. The global coefficient of friction (applied to all objects and models). Defaults to 0.5
	- collisionRestitutionCoefficient: Optional. The global coefficient of restitution (applied to all objects and models). Defaults to 0.0
Unless marked Fixed, all the corresponding config parameters can be changed at runtime.
-->
<simParameters timestep="0.001"
	speedupFactor="1.0"
	enableJointLimits="true"
	enableGravityCompensation="true"
	frictionCoefficient="0.5"
	collisionRestitutionCoefficient="0.0"/>
```

#### The `<robotOrObjectSpecificParameters>` sub element

This child element of `<simvizConfiguration>` enables the setting of specific simulation and rendering parameters for a given robot or object in the world. The robot or object must exist in the world file. The parameters that are redundant with the `<simParameters>` (friction coefficient and restitution coefficient) will use the specialized value if defined here for the specific robot or object.

```
<!--
The optional <robotOrObjectSpecificParameters> element contains siulation and rendering parameters for a specific robot or object.
If not present for a given robot or object, the default values are used.
Its attributes are:
	- name: Required. The name of the robot or object
	- dynamicsEnabled: Optional. Whether to enable dynamics for the robot or object. If false, no forces/moments can be applied to
		the robot/object, and its collision is disabled Defaults to true
	- renderingEnabled: Optional. Whether to enable rendering for the robot or object If false, the object is invisible, 
		but can still interact with the world, depending if its dynamics are enabled or not. Defaults to true
	- jointLimitsEnabled: Optional. Whether to enable joint limits for the robot in simulation. No effect on objects. Defaults to true
	- frictionCoefficient: Optional. The coefficient of friction for the robot or object. 
		Overrides the default value if present. Defaults to 0.5
	- collisionRestitutionCoefficient: Optional. The coefficient of restitution for the robot or object. 
		Overrides the default value if present. Defaults to 0.0
	- wireMeshRenderingMode: Optional. Whether to render the robot or object in wire mesh mode. Defaults to false
	- framesRenderingEnabled: Optional. Whether to render the kinematic frames of the robot or object. Defaults to false
	- frameSizeWhenRendering: Optional. The size of the kinematic frames when rendering. Defaults to 0.2
All the corresponding config parameters can be changed at runtime.
-->
<robotOrObjectSpecificParameters name="Panda"
	dynamicsEnabled="true"
	renderingEnabled="true"
	jointLimitsEnabled="true"
	frictionCoefficient="0.5"
	collisionRestitutionCoefficient="0.0"
	wireMeshRenderingMode="false"
	framesRenderingEnabled="false"
	frameSizeWhenRendering="0.2" />
```

#### The `<logger>` sub element

This child element of `<simvizConfiguration>` defines the configuration for the logger attached to the simulation. The logger is a component of the SimvizRedisInterface that can start a new thread to log predefined data (such as the robot state and commands) to a csv file.

```
<!--
The optional <logger> element contains the parameters for the simviz logger.
Only one is allowed per simvizConfiguration element.
Its attributes are:
	- logFolderName: Optional. The relative path of the folder where the log files will be saved. Defaults to "log_files/simviz"
	- logFrequency: Optional. The frequency in Hz at which the log files are saved. Defaults to 100 Hz
	- enabledAtStartup: Optional. Whether to start logging data as soon as the simulation starts. Defaults to false
	- addTimestampToFilename: Optional. Whether to add the timestamp of the logging start to the log file name. Defaults to true
None of the corresponding config parameters can be changed at runtime.
-->
<logger logFolderName="log_files/simviz"
	logFrequency="100"
	enabledAtStartup="false"
	addTimestampToFilename="true" />
```

#### The `<forceSensor>` sub element

This child element of `<simvizConfiguration>` enables the creation of a simulated force sesnor on a robot or object in the simulation. There can be one force sensor per object, and one per link on robots.

```
<!--
The optional <forceSensor> element allows to define a simulated force sensor.
Its attributes are:
	- robotOrObjectName: Required. The name of the robot to which the sensor is attached
	- linkName: Required if robot, Optional if object. The name of the link to which the sensor is attached
	- filterCutoff: Optional. The cutoff frequency in Hz of the low-pass filter applied to the force/torque measurements. Defaults to 5.0 Hz
None of the corresponding config parameters can be changed at runtime.
-->
<forceSensor robotOrObjectName="Panda"
	linkName="end-effector"
	filterCutoff="5.0">
	<!--
	The optional <origin> element represents the transformation between the link frame and the sensor frame.
	for the sensor. Its attributes are:
		- xyz: Optional. The x, y, z translation of the sensor frame in the link frame in meters. Defaults to 0 0 0
		- rpy: Optional. The rotation in XYZ euler angles in Radians of the sensor frame in the link frame. Defaults to 0 0 0
	This frame cannot be changed at runtime.
	-->
	<origin xyz="0 0 0"
		rpy="0 0 0" />
</forceSensor>
```

Its only optional child element is the `<origin>` tag to define the force sensor frame in link frame.

### The `<robotControlConfiguration>` element

Now let's move to the robot control configuration element. Note that in the examples, we separated the simviz configuration and the control configuration in different files for pedagogic purposes, but if you want an application with bot a simulation and a controller, they must be in the same file (and it is the case for many other example config files). If we want an application with a controller for a robot, we need to set a robot control configuration for that robot in the config file by using the `<robotControlConfiguration>` element. We will go step by step over the [commented example file](https://github.com/manips-sai-org/sai-interfaces/blob/master/examples/config_files/aa_detailled_panda_control_only.xml) to explain the possible attributes and children elements. The required attributes here are is the `robotModelFile` that provides the path to the robot urdf model file defining the kinematic and dynamic model of the robot (see the [ROS URDF documentation for more details](http://wiki.ros.org/urdf/XML/model)), and the `robotName` that is used to generate the correct redis keys for the controller (so when using the simulation, the robot name here must match the one in the simulated world, and when controlling an actual robot, the redis keys must be selected accordingly).

```
<!--
The <robotControlConfiguration> element contains the control configuration for a robot.
There can only be one of those per robot (identified by the robot name) in a config file.
a robot can have multiple controllers, all defined within this element, but only one can be active at a time.
Its attributes are:
	- robotName: Required. The name of the robot
	- robotModelFile: Required. The path to the URDF file of the robot
	- controlFrequency: Optional. The control frequency of the robot. Defaults to 1000.0 Hz
	- getMassMatrixFromRedis: Optional. Whether to get the mass matrix from redis (must be published by the simulation or robot driver) or to compute it from the urdf mass parameters. Defaults to true.
None of the corresponding config parameters can be changed at runtime.
-->
<robotControlConfiguration robotName="Panda"
	robotModelFile="${SAI_MODEL_URDF_FOLDER}/panda/panda_arm_sphere.urdf"
	controlFrequency="1000.0"
	getMassMatrixFromRedis="true">

...

</robotControlConfiguration>
```

We will now go over the possible children of the `<robotControlConfiguration>` .

#### The `<baseFrame>` sub element

This child element of `<robotControlConfiguration>` defines the robot base frame in the world. Remember that the control inputs (such as goal positions) are provided in world frame.

```
<!--
The optional <baseFrame> element contains the base frame of the robot with respect to world frame.
Its attributes are:
	- xyz: Optional. The position of the base frame origin in meters. Defaults to 0 0 0
	- rpy: Optional. The orientation of the base frame in XYZ euler angles in Rad. Defaults to 0 0 0
The robot base frame cannot be changed at runtime.
-->
<baseFrame xyz="0 0 0"
	rpy="0 0 0" />
```

#### The `<worldGravity>` sub element

This child element of `<robotControlConfiguration>` defines the world 3d gravity vector. It is useful if the robot controller has to perform gravity compensation.

```
<!--
The optional <worldGravity> element describes the 3D gravity vector in world frame.
Its attributes are:
	- xyz: Optional. The gravity vector in world frame in m/s^2. Defaults to 0 0 -9.81
The gravity vector cannot be changed at runtime.
-->
<worldGravity xyz="0 0 -9.81" />
```

#### The `<logger>` sub element

This child element of `<robotControlConfiguration>` defines the configuration for the logger attached to the robot controllers. The logger is a component of the RobotControlRedisInterface that can start a new thread to log predefined data (such as control task inputs/outputs and controller parameters) to a csv file.

```
<!--
The optional <logger> element contains the parameters for the robot controller logger.
Only one is allowed per robotControlConfiguration element.
Its attributes are:
	- logFolderName: Optional. The relative path of the folder where the log files will be saved. Defaults to "log_files/robot_controllers"
	- logFrequency: Optional. The frequency in Hz at which the log files are saved. Defaults to 100 Hz
	- enabledAtStartup: Optional. Whether to start logging data as soon as the controller starts. Defaults to false
	- addTimestampToFilename: Optional. Whether to add the timestamp of the logging start to the log file name. Defaults to true
None of the corresponding config parameters can be changed at runtime.
-->
<logger logFolderName="log_files/robot_controllers"
	logFrequency="100"
	enabledAtStartup="false"
	addTimestampToFilename="true" />
```

#### The `<controller>` sub element

This is the main child element of the `<robotControlConfiguration>` . Each one of the `<controller>` tags allows the definition of a hierarchical controller that can have motion force cartesian tasks and joint tasks. The controller can perform gravity compensation or not, and the experimental joint limit avoidance task can be enabled for joint position and velocity limits avoidance. The controller can also be configured to saturate the computed torques to the mas torque value defined in the urdf file.

```
<!--
The <controller> element contains the configuration for one controller.
There can be multiple controllers per robotControlConfiguration element.
Each controller must have a unique name inside the robotControlConfiguration element.
A controller is parametrized by a sequence of tasks that are defined within the controller element.
The order in which the tasks are defined determined the hierarchical structure of the controller.
Each subsequent task will be performed in the nullspace of the previous tasks.
The supported tasks are:
	- jointTask: a task to control the robot joints
	- motionForceTask: a task to control the robot end-effector in cartesian space
The attributes of the controller element are:
	- name: Required. The name of the controller. Two controllers on the same robot cannot have the same name
	- gravityCompensation: Optional. Whether to compensate for gravity in the controller. Defaults to false as 
		most robots API perform the gravity compensation internally
	- jointLimitAvoidance: Optional. Whether to enable the joint limit avoidance task for position and velocity limits 
		in the controller. Defaults to false as it is an experimental feature and should be used with caution
	- torqueSaturation: Optional. Whether to saturate the torques computed by the controller. Defaults to false
Only one controller can be active at a time.
The active controller initially is the one defined first in the robotControlConfiguration element.
The active controller can be changed at runtime. All the other parameters are fixed at runtime
-->
<controller name="cartesian_controller"
	gravityCompensation="false"
	jointLimitAvoidance="false"
	torqueSaturation="false">

...

</controller>

```

The children of the `<controller>` element are task definitions:

##### The `<motionForceTask>` task definition inside a `<controller>`

This child element of `<controller>` defined a motion force task in the hierarchy of the corresponding controller

```
<!--
The <motionForceTask> element contains the configuration for a motion-force task.
A motion-force task is a task to control the robot end-effector in cartesian space.
The motion-force task can be controlled in translation and rotation, and in force and moment.
Its attributes are:
	- name: Required - Fixed. The name of the task. Two tasks within a controller cannot have the same name
	- linkName: Required - Fixed. The name of the link to control
	- parametrizatonInCompliantFrame: Optional - Fixed. Whether to parametrize the task in the compliant frame.
		If set to true, then the gains and force space parametrizations are defined in the compliant frame.
		Otherwise, they are defined in the world frame.
	- useDynamicDecoupling: Optional. Whether to use dynamic decoupling within the task.
	- bieThreshold: Optional - Fixed. The threshold for the bounded inertia estimates method.
		The bounded inertia estimate method is a tradeoff between perfect dynamic decoupling
		and orientation controlability in systems with very non isotropic inertia properties.
		When controlling an actual robot, it is advised to leave it between 0.01 and 0.1.
		In simulation, you can set it to 0 in order to have perfect dynamic decoupling.
All default values for the optional parameters here and in the sub-elements are the default
values defined in the MotionForceTask class in the sai-primitives library
see (https://github.com/manips-sai-org/sai-primitives).
Dynamic decoupling can be switched on and off at runtime.
The rest of the parameters corresponding to those attributes are fixed and cannot be changed at runtime.
-->
<motionForceTask name="cartesian_task"
	linkName="end-effector"
	parametrizatonInCompliantFrame="false"
	useDynamicDecoupling="true"
	bieThreshold="0.1">

...

</motionForceTask>
```

The children elements of a `<motionForceTask>` define the initial parametrization of the task. A lot of those parameters can be changed at runtime.

* __The compliant frame definition__:

```
<!--
The optional <compliantFrame> element contains the position and orientation of the
compliant frame with respect to the kinematic frame of the controlled link.
The compliant frame is the frame controlled by the task.
Its attributes are:
	- xyz: Optional. The position of the compliant frame origin in meters.
	- rpy: Optional. The orientation of the compliant frame in XYZ euler angles in Rad.
The compliant frame cannot be changed at runtime.
-->
<compliantFrame xyz="0 0 0"
	rpy="0 0 0" />
```

* __The controlled directions in translation and rotation__:

```
<!--
The optional <controlledDirectionsTranslation> element contains the directions in translation that are controlled by the task.
This enabled to control only a subset of the translation directions (for example for a planar robot).
Its children are <direction> elements, each with an xyz attribute.
Each direction is a vector in the world frame, and there can be as many as needed.
The controlled translation space is the range of the vectors provided here.
If not specified, all directions are controlled.
The controlled directions cannot be changed at runtime.
-->
<controlledDirectionsTranslation>
	<direction xyz="1 0 0" />
	<direction xyz="0 1 0" />
	<direction xyz="0 0 1" />
</controlledDirectionsTranslation>
```

```
<!--
The optional <controlledDirectionsRotation> element contains the directions in rotation that are controlled by the task.
It is the same as the controlledDirectionsTranslation element, but for rotation.
-->
<controlledDirectionsRotation>
	<direction xyz="1 0 0" />
	<direction xyz="0 1 0" />
	<direction xyz="0 0 1" />
</controlledDirectionsRotation>
```

* __The parametrization for the force part of the task__:

```
<!--
The optional <forceControl> element contains the parameters for the force control.
Its attributes are:
	- closedLoopForceControl: Optional. Whether to use closed loop force control.
		If false, open loop force control will be used.
Closed loop force control can be switched on and off at runtime.
Default value can be found in the MotionForceTask class in the sai-primitives library.
-->
<forceControl closedLoopForceControl="false">
	<!--
	The optional <forceSensorFrame> element contains the position and orientation of the
	force sensor frame with respect to the compliant frame.
	Its attributes are:
		- xyz: Optional. The position of the force sensor frame origin in meters.
		- rpy: Optional. The orientation of the force sensor frame in XYZ euler angles in Rad.
	The force sensor frame location cannot be changed at runtime.
	-->
	<forceSensorFrame xyz="0 0 0"
		rpy="0 0 0" />
	<!-- 
	The optional <forceSpaceParametrization> element contains the parameters for the force space parametrization.
	It allows to control certain directions in motion and others in force.
	Its attributes are:
		- dim: Optional. The dimension of the force space.
			- 0: Full motion control
			- 1: A line of force control and a plane of motion control
			- 2: A plane of force control and a line of motion control
			- 3: Full force control
		- direction: Optional. The direction of line when the dimension is 1 or 2 (unused when the dimension is 0 or 3).
	Those values can be changed at runtime.
	Default value for the dim can be found in the MotionForceTask class in the sai-primitives library.
	-->
	<forceSpaceParametrization dim="0"
		direction="0 0 1" />
	<!-- 
	the Optional <momentSpaceParametrization> element contains the parameters for the moment space parametrization.
	It is the same as the forceSpaceParametrization element, but for orientation/moment control.
	-->
	<momentSpaceParametrization dim="0"
		direction="0 0 1" />
	<!-- 
	The optional <forceGains> element contains the gains for the force control.
	Its attributes are:
		- kp: Optional. The proportional gain for the force control.
		- kv: Optional. The derivative gain for the force control (velocity-based damping).
		- ki: Optional. The integral gain for the force control.
	Those values can be changed at runtime.
	Default values can be found in the MotionForceTask class in the sai-primitives library.
	-->
	<forceGains kp="1.0"
		kv="20.0"
		ki="1.5" />
	<!-- 
	The optional <momentGains> element contains the gains for the moment control.
	Its attributes are:
		- kp: Optional. The proportional gain for the moment control.
		- kv: Optional. The derivative gain for the moment control (velocity-based damping).
		- ki: Optional. The integral gain for the moment control.
	Those values can be changed at runtime.
	Default values can be found in the MotionForceTask class in the sai-primitives library.
	-->
	<momentGains kp="1.0"
		kv="20.0"
		ki="1.5" />
</forceControl>
```

* __The velocity saturation configuration__:

```
<!-- 
The optional <velocitySaturation> element contains the velocity saturation parameters for the task.
Velocity saturation limits the velocity of the controlled frame by limiting the control force applied in a particular way.
Its attributes are:
	- enabled: Required if element present. Whether to use velocity saturation.
	- linearVelocityLimit: Optional. The limit for the linear velocity in m/s.
	- angularVelocityLimit: Optional. The limit for the angular velocity in Rad/s.
Those values can be changed at runtime.
Default values can be found in the MotionForceTask class in the sai-primitives library.
-->
<velocitySaturation enabled="true"
	linearVelocityLimit="0.35"
	angularVelocityLimit="0.78" />
```

* __The parametrization for the online trajectory generation__

```
<!-- 
the optional <otg> element contains the parameters for the online trajectory generation.
Its attributes are:
	- type: Required if element present. The value can be
		- "disabled": no otg
		- "acceleration": acceleration limited trajectories
		- "jerk": jerk limited trajectories
	- maxLinearVelocity: Optional. The maximum linear velocity in m/s.
	- maxLinearAcceleration: Optional. The maximum linear acceleration in m/s^2.
	- maxLinearJerk: Optional. The maximum linear jerk in m/s^3.
	- maxAngularVelocity: Optional. The maximum angular velocity in Rad/s.
	- maxAngularAcceleration: Optional. The maximum angular acceleration in Rad/s^2.
	- maxAngularJerk: Optional. The maximum angular jerk in Rad/s^3.
Those values can be changed at runtime.
Default values can be found in the MotionForceTask class in the sai-primitives library.
-->
<otg type="disabled"
	maxLinearVelocity="0.35"
	maxLinearAcceleration="1.5"
	maxLinearJerk="5.0"
	maxAngularVelocity="1.0"
	maxAngularAcceleration="3.0"
	maxAngularJerk="100" />
```

* __The position and orientation gains__:

```
<!-- 
The optional <positionGains> element contains the gains for the position control.
Unlike the force gains, the position gains can be either isotropic or anisotropic.
If defined as a single number, the gains are isotropic.
If defined as a list of 3 numbers, the gains are anisotropic.
Its attributes are:
	- kp: Optional. The proportional gain for the position control.
	- kv: Optional. The derivative gain for the position control.
	- ki: Optional. The integral gain for the position control.
Those values can be changed at runtime.
Default values can be found in the MotionForceTask class in the sai-primitives library.
-->
<positionGains kp="100.0 110.0 100.0"
	kv="20.0" />

<!--
The optional <orientationGains> element contains the gains for the orientation control.
Unlike the moment gains, the orientation gains can be either isotropic or anisotropic.
If defined as a single number, the gains are isotropic.
If defined as a list of 3 numbers, the gains are anisotropic.
Its attributes are:
	- kp: Optional. The proportional gain for the orientation control.
	- kv: Optional. The derivative gain for the orientation control.
	- ki: Optional. The integral gain for the orientation control.
Those values can be changed at runtime.
Default values can be found in the MotionForceTask class in the sai-primitives library.
-->
<orientationGains kp="100.0"
	kv="20.0" />
```

* __The ui interface sliders bounds__:

```
<!-- 
The optional <interface> element contains the limits for the interface sliders.
They can either be a single number (the sliders will all take that value)
or a list of 3 numbers (one for each slider).
Its attributes are:
	- minGoalPosition: Optional. The minimum value for the goal position slider.
	- maxGoalPosition: Optional. The maximum value for the goal position slider.
	- minDesiredForce: Optional. The minimum value for the desired force slider.
	- maxDesiredForce: Optional. The maximum value for the desired force slider.
	- minDesiredMoment: Optional. The minimum value for the desired moment slider.
	- maxDesiredMoment: Optional. The maximum value for the desired moment slider.
Those values cannot be changed at runtime.
The default values can be found in the RobotControllerConfig.h file under the MotionForceTaskConfig::InterfaceConfig struct.
-->
<interface minGoalPosition='-0.5 -0.5 0.0'
	maxGoalPosition='0.5 0.5 0.8'
	minDesiredForce='-50'
	maxDesiredForce='50'
	minDesiredMoment='-5'
	maxDesiredMoment='5' />
```

##### The `<jointTask>` task definition inside a `<controller>`

This child element of `<controller>` defined a joint task (that can be full or partial) in the hierarchy of the corresponding controller

```
<!-- 
The <jointTask> element contains the configuration for a joint task.
A joint task is a task to control the robot joints.
Its attributes are:
	- name: Required - Fixed. The name of the task. Two tasks within a controller cannot have the same name
	- useDynamicDecoupling: Optional. Whether to use dynamic decoupling within the task.
	- bieThreshold: Optional - Fixed. The threshold for the bounded inertia estimates method.
		The bounded inertia estimate method is a tradeoff between perfect dynamic decoupling
		and orientation controlability in systems with very non isotropic inertia properties.
		When controlling an actual robot, it is advised to leave it between 0.01 and 0.1.
		In simulation, you can set it to 0 in order to have perfect dynamic decoupling.
All default values for the optional parameters here and in the sub-elements are the default
values defined in the JointTask class in the sai-primitives library
see (https://github.com/manips-sai-org/sai-primitives).
Dynamic decoupling can be switched on and off at runtime.
The rest of the parameters corresponding to those attributes are fixed and cannot be changed at runtime.		
-->
<jointTask name="joint_task"
	useDynamicDecoupling="true"
	bieThreshold="0.1">
```

The children elements of a `<jointTask>` define the initial parametrization of the task. A lot of those parameters can be changed at runtime.

* __The controlled joints__:

```
<!--
The optional <controlledJointNames> element contains the names of the joints to control as a list.
It enables the definition of a partial joint task.
If not specified or left empty, all joints are controlled and we get a full joint task
-->
<controlledJointNames>
	joint1
	joint2
	joint3
	joint4
	joint5
	joint6
	joint7
</controlledJointNames>
```

* __The initial velocity saturation parametrization__:

```
<!--
The optional <velocitySaturation> element contains the velocity saturation parameters for the task.
Velocity saturation limits the velocity of the controlled joints by limiting the control force applied in a particular way.
Its attributes are:
	- enabled: Required if element present. Whether to use velocity saturation.
	- velocityLimit: Optional. The limit for the velocity in Rad/s. Can be defined as a single number 
		or a list of the correct size for a different limit per joint.
Those values can be changed at runtime.
Default values can be found in the JointTask class in the sai-primitives library.
-->
<velocitySaturation enabled="false"
	velocityLimit="1.1 1.2 1.3 1.4 1.5 1.6 1.7" />
```

* __The initial online trajectory generation parametrization__:

```
<!-- 
The optional <otg> element contains the parameters for the online trajectory generation.
Its attributes are:
	- type: Required if element present. The value can be
		- "disabled": no otg
		- "acceleration": acceleration limited trajectories
		- "jerk": jerk limited trajectories
	- maxVelocity: Optional. The maximum velocity in Rad/s. Can be defined as a single number 
		or a list of the correct size for a different limit per joint.
	- maxAcceleration: Optional. The maximum acceleration in Rad/s^2. Can be defined as a single number 
		or a list of the correct size for a different limit per joint.
	- maxJerk: Optional. The maximum jerk in Rad/s^3. Can be defined as a single number 
		or a list of the correct size for a different limit per joint.
Those values can be changed at runtime.
Default values can be found in the JointTask class in the sai-primitives library.
-->
<otg type="acceleration"
	maxVelocity="1.05"
	maxAcceleration="6.28"
	maxJerk="31.4" />
```

* __The initial gains__: 

```
<!-- 
The optional <gains> element contains the gains for the joint control.
Its attributes are:
	- kp: Optional. The proportional gain for the joint control. Can be defined as a single number 
		or a list of the correct size for a different gain per joint.
	- kv: Optional. The derivative gain for the joint control. Can be defined as a single number 
		or a list of the correct size for a different gain per joint.
	- ki: Optional. The integral gain for the joint control. Can be defined as a single number 
		or a list of the correct size for a different gain per joint.
Those values can be changed at runtime.
Default values can be found in the JointTask class in the sai-primitives library.
-->
<gains kp="50.0"
	kv="14.0"
	ki="0.0" />
```

### The `<hapticDeviceControlConfiguration>` element

Finally, let's look at the haptic control. We support any haptic devices that can use the [chaiHapticDeviceDriver](https://github.com/manips-sai-org/chaiHapticdeviceRedisDriver). In this third example, we have a config file containing all the elements, that defined a simulation with a robot controlled using a haptic device. If we want to control haptically one of the tasks defined in the robot controller, we need to set a haptic device control configuration for that robot task in the config file by using the `<hapticDeviceControlConfiguration>` element. Note that only a motion force task can be controlled haptically. The haptic controller can be setup as an impedance controller where the motion of the haptic device is transferred to the robot, and the forces felt to the robot are sent back and applied to the haptic device; or admittance mode where the haptic device is maintained at the center of its workspace by a force field, and pushing the device in a direction will command a robot velocity in that direction.

Initially, the haptic device will be in homing mode, going to the center of its workspace, and then the control is switched to the desired mode (impedance or admittance).
For haptic devices with switches (or gripper used as a switch), we can require the switch to be pressed in order to exit homing. We can also use the switch to either disconnect the haptic control from the robot task temporarily (clutch mode), or to switch between translation only vs translation plus orientation control. The switch can also be configured to use a click to trigger the switching back and forth, or to have to hold the switch pressed in order to change the mode, and release it to change back.

We will go step by step over the [commented example file](https://github.com/manips-sai-org/sai-interfaces/blob/master/examples/config_files/aa_detailled_panda_haptic_control.xml) to explain the possible attributes and children elements.

```
<!--
The <hapticDeviceControlConfiguration> element contains the configuration for the haptic device control.
There can be multiple <hapticDeviceControlConfiguration> elements in the configuration file, each one
should correspond to a unique haptic device and a unique controlled robot task.
The attributes of the <hapticDeviceControlConfiguration> element are:
	- controlMode: the control mode of the haptic device. Possible values are "impedance" and "admittance". Default is "impedance".
	- switchFunction: the function of the switch. Possible values are "clutch" and "orientationControl". Default is "clutch".
		Clutch means that the switch is used to enable/disable the haptic control (move the haptic device without moving the robot).
		OrientationControl means that the switch is used to switch between translation only, and translation plus orientation control.
	- switchUsageType: the usage type of the switch. Possible values are "click" and "hold". Default is "click".
	- orientationTeleopEnabled: whether the orientation teleoperation is enabled or not initially. Default is false.
	- useSwitchToExitHoming: whether the switch should be used to exit the homing mode or if the homing mode is exited automatically. Default is true.
	- controlFrequency: the control frequency of the haptic device control. Default is 1000.0 Hz.
-->
<hapticDeviceControlConfiguration controlMode="impedance"
	switchFunction="clutch"
	switchUsageType="click"
	orientationTeleopEnabled="false"
	useSwitchToExitHoming="true"
	controlFrequency="1000.0">
```

Let us now look at its children elements

#### The `<controlledRobotTask>` sub element
This is the only mandatory sub element of the `<hapticDeviceControlConfiguration>`. It defined which task in which controller for which robot is to be controlled by the haptic device.

```
<!--
The <controlledRobotTask> element contains the name of the robot, the name of the controller, and the name of the task that the haptic device should control.
One and only one is required per <hapticDeviceControlConfiguration> element.
Its attributes are:
	- robotName: the name of the robot that the haptic device should control.
	- controllerName: the name of the controller for that robot that contains the task the haptic device should control.
	- taskName: the name of the task that the haptic device should control.
-->	
<controlledRobotTask robotName="Panda"
	controllerName="cartesian_controller"
	taskName="eef_task" />
```

#### The `<baseFrame>` sub element
Defined the base frame of the haptic device with respect to the world frame.

```
<!--
The optional <baseFrame> element contains the position and orientation of the base frame of the haptic device with respect to the world frame.
The attributes of the <baseFrame> element are:
	- xyz: the position of the base frame in the world frame. Default is "0 0 0".
	- rpy: the orientation of the base frame in the world frame in radians. Default is "0 0 0".
-->
<baseFrame xyz="0 0 0"
	rpy="0 0 0.5" />
```

#### The `<logger>` sub element
Similar to the loggers for the robot controller and simviz, logs data from the haptic device controller.

```
<!--
The optional <logger> element contains the configuration for the logger.
Its attributes are:
	- logFolderName: the name of the folder where the log files should be saved. Default is "log_files/haptic_controllers".
	- logFrequency: the frequency at which the logger should log data. Default is 100 Hz.
	- enabledAtStartup: whether the logger should be enabled at startup or not. Default is false.
	- addTimestampToFilename: whether the timestamp should be added to the filename of the log files or not. Default is true.
-->
<logger logFolderName="log_files/haptic_controllers"
	logFrequency="100"
	enabledAtStartup="false"
	addTimestampToFilename="true" />
```

#### The `<homingMode>` sub element
Defined the velocities at which the device homes.
```
<!--
The optional <homingMode> element contains the configuration for the homing mode.
Its attributes are:
	- maxLinearVelocity: the homing maximum linear velocity of the haptic device in m/s. Default is 0.15.
	- maxAngularVelocity: the homing maximum angular velocity of the haptic device in rad/s. Default is 3.1415927.
-->
<homingMode maxLinearVelocity="0.15"
	maxAngularVelocity="3.1415927" />
```

#### The `<impedanceMode>` sub element
Defined the configuration of the impedance mode. Only useful if impedance mode is used for that haptic device. It contains several sub elements for the different parametrizeable aspects of the ipedance mode.
```
<!--
The optional <impedanceMode> element contains the configuration for the impedance mode.
It is only used if the controlMode is set to "impedance".
-->
<impedanceMode>
```

##### The `<virtualWorkspaceLimits>` definition in impedance mode
```
<!--
The optional <virtualWorkspaceLimits> element contains the configuration for the virtual workspace limits.
The virtual workspace limits implements a virtual sphere in translation and a virtual cone in orientation.
The device is haptically contrained to stay inside the sphere and cone.
Its attributes are:
	- enabled: whether the virtual workspace limits are enabled or not initially. Default is false.
	- radius: the radius of the virtual workspace in meters. Default is 0.1.
	- angle: the angle of the virtual workspace in radians. Default is 1.0471975512.
-->
<virtualWorkspaceLimits enabled="false"
	radius="0.1"
	angle="1.0471975512" />
```

##### The `<scalingFactors>` definition in impedance mode
```
<!--
The optional <scalingFactors> element contains the scaling factors for the impedance control.
It represents the ratio between the robot motion and the haptic device motion.
For example, a scaling factor of 2 means that 1cm od haptic device motion corresponds to 2cm of robot motion.
Its attributes are:
	- linear: the linear scaling factor. Default is 1.0.
	- angular: the angular scaling factor. Default is 1.0.
-->
<scalingFactors linear="1.0"
	angular="1.0" />
```

##### The `<variableDamping>` definition in impedance mode
Impedance mode can implement variable damping. This is damping that will be applied only in the directions that use direct force feedback. It is damping with a damping coefficient that is a piecewise affine function of the haptic device velocity such that the damping can be set to a low value, or even zero, at low velocities, and to a high value at higher velocities.
```
<!--
The optional <variableDamping> element contains the variable damping parameters for the impedance control.
The variable damping is a piecewise damping function that depends on the linear and angular velocities of the haptic device.
The velocity thresholds and damping vectors needs to be the same size, to each value of the velocity corresponds a value of the damping, 
and the damping is interpolated linearly between the values, in function of the velocity.
If omitted, there is no damping in impedance mode.
For a constant value of damping, set the velocity threshold to a vector with only one value equal to 0.0, 
and the damping to a vector with only one value equal to the desired damping.
Its attributes are:
	- linearVelocityThresholds: the linear velocity thresholds in m/s.
	- linearDamping: the linear damping values in Ns/m.
	- angularVelocityThresholds: the angular velocity thresholds in rad/s.
	- angularDamping: the angular damping values in Nms.
-->
<variableDamping linearVelocityThresholds="0.1 0.3"
	linearDamping="0.0 0.3"
	angularVelocityThresholds="0.25 0.75"
	angularDamping="0.0 0.1" />
```

##### The `<forceFeedback>` definition in impedance mode
The forceFeedback element allows to define both reduction factors and directions of space for proxy based force feedback.
Reduction factors define which proportion of the robot sensed force is applied to the haptic device in direct force feedback directions.
Proxy based force feedback means the force feedback comes from the position error between the robot position and its commanded position from the haptic device, instead of the direct robot sensed force. It can be set in certain directions or the whole space by setting the dimension of the proxy based force feedback space (between 0 and 3) and the single direction of proxy feedback (if the dimension is 1) or direct force feedback (if the dimension is 2).
```
<!--
The optional <forceFeedback> element contains the force feedback parameters for the impedance control.
Those parameters contain the reduction factor (factor between 0 and 1 that reduces the direct feedback force),
as well as the proxy parametrization (space dimension and axis) for the force and moment feedback.
The directions can either use proxy based feedback (the feedback force is computed from the position error between the haptic device and the robot),
or direct force feedback where the sensed robot force is provided as an input to the haptic controller.
Its attributes are:
	- reductionFactorForce: the reduction factor for the force feedback. Default is 1.0.
	- reductionFactorMoment: the reduction factor for the moment feedback. Default is 1.0.
	- proxyForceSpaceDimension: the space dimension for the proxy force space 
		(0 means direct force feedback in all directions, 3 means proxy based feedback in all directions). Default is 0.
	- proxyMomentSpaceDimension: the space dimension for the proxy moment space. 
		(0 means direct moment feedback in all directions, 3 means proxy based feedback in all directions). Default is 0.
	- proxyForceAxis: the axis for the force feedback. Default is "0 0 1".
	- proxyMomentAxis: the axis for the moment feedback. Default is "0 0 1".
-->
<forceFeedback reductionFactorForce="1.0"
	reductionFactorMoment="1.0"
	proxyForceSpaceDimension="0"
	proxyMomentSpaceDimension="0"
	proxyForceAxis="0 0 1"
	proxyMomentAxis="0 0 1" />
```