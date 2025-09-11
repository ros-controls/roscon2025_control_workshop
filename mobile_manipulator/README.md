# Example Mixed Hardware Interfaces (mock + real hardware) Example

## Running the Example with Mock Hardware

1. Start and open an interactive shell to the Workshop container
```bash
docker exec -it ros2_control_roscon25 bash
```

2. Start the Zenoh router
```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

3. On the host open a second interactive terminal to the Workshop container and start our example robot.
You should see a turtlebot with a Piper arm attached to the top ready for simulation with `mock_hardware`.
```bash
docker exec -it ros2_control_roscon25 bash
ros2 launch wbot_bringup wbot_manipulator.launch.xml mock_hardware:=true
```

In this configuration there are 2 mock hardware components running, the mobile base `diff drive` and `manipulator` interfaces.
To introspect them we can run the following command in a terminal inside the container
Start an interactive terminal to the Workshop container
```bash
docker exec -it ros2_control_roscon25 bash
```
and query the Controller Manager with
```bash
ros2 control list_hardware_components
```

which should show you the following output...
```bash
Hardware Component 1
	name: wbot_arm_piper_control
	type: system
	plugin name: mock_components/GenericSystem
	state: id=3 label=active
	read/write rate: 100 Hz
	is_async: False
	command interfaces
		wbot_arm_joint_1/position [available] [claimed]
		wbot_arm_joint_2/position [available] [claimed]
		wbot_arm_joint_3/position [available] [claimed]
		wbot_arm_joint_4/position [available] [claimed]
		wbot_arm_joint_5/position [available] [claimed]
		wbot_arm_joint_6/position [available] [claimed]
		wbot_arm_gripper_joint/position [available] [claimed]
Hardware Component 2
	name: wbot_base_control
	type: system
	plugin name: mock_components/GenericSystem
	state: id=3 label=active
	read/write rate: 100 Hz
	is_async: False
	command interfaces
		wbot_wheel_left_joint/velocity [available] [claimed]
		wbot_wheel_right_joint/velocity [available] [claimed]
```

From the above we can see the `wbot_arm_piper_control` component has 6 joints plus a gripper that each offer a position command interface that has been `claimed` and is running the `mock_components/GenericSystem` plugin.
The base has 2 joints that each offer a velocity interface that are also `claimed` and running a different `mock_components/GenericSystem` plugin.

To see who `claimed` them we can ask the Controller Manager what controllers are running, but we would like the verbose output so we can see what each controller has claimed.
```bash
ros2 control list_controllers -v
```

and you will see some of the following output
```bash
joint_trajectory_controller joint_trajectory_controller/JointTrajectoryController  active
	update_rate: 100 Hz
	is_async: False
	claimed interfaces:
		wbot_arm_joint_1/position
		wbot_arm_joint_2/position
		wbot_arm_joint_3/position
		wbot_arm_joint_4/position
		wbot_arm_joint_5/position
		wbot_arm_joint_6/position
diff_drive_base_controller  diff_drive_controller/DiffDriveController              active
	update_rate: 100 Hz
	is_async: False
	claimed interfaces:
		wbot_wheel_left_joint/velocity
		wbot_wheel_right_joint/velocity
gripper_controller          parallel_gripper_action_controller/GripperActionController  active
	update_rate: 100 Hz
	is_async: False
	claimed interfaces:
		wbot_arm_gripper_joint/position
```

So the `JointTrajectoryController` is commanding the `wbot_arm_piper_control` hardware, the `DiffDriveController` is commanding the `wbot_base_control` hardware and the `gripper_controller` is commanding one joint from the `wbot_arm_piper_control` hardware.

## Running the Example with mixed Real + Mock Hardware

Lets say we want to develop against our mobile base hardware but do not have access to a physical manipulator.
ros2_control makes this easy by allowing developers to choose which hardware instance is run per device at launch time.
For example you can see in [wbot_manipulator_macro.xacro](../zenoh_host/wbot_bringup/launch/wbot_manipulator.launch.xml#L25) I have hard coded the arm to always use mock hardware regardless of the xacro parameter `mock_hardware`.
This allows me to switch between simulating the diff drive base with mock or real hardware.

To see this in action run we will run our [ESP32 embedded-mobile-base](../embedded-mobile-base/) hardware and then launch our robot with the manipulator still running mock_hardware.

1. Connect the ESP32 module flashed with embedded-mobile-base
2. Start the Zenoh router
```bash
ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/[::]:7447","serial//dev/ttyACM0#baudrate=460800"]' ros2 run rmw_zenoh_cpp rmw_zenohd
```
3. Launch the wbot_bringup with the `mock_hardware` argument set to `false`
```bash
ros2 launch wbot_bringup wbot_manipulator.launch.xml mock_hardware:=false
```
4. List the hardware components information
```bash
ros2 control list_hardware_components
```

Where you can see `wbot_base_control` is now running the `JointStateTopicSystem` plugin to communicate with the embedded device.
```bash
Hardware Component 1
	name: wbot_arm_piper_control
	type: system
	plugin name: mock_components/GenericSystem
	state: id=3 label=active
	read/write rate: 100 Hz
	is_async: False
	command interfaces
		wbot_arm_joint_1/position [available] [claimed]
		wbot_arm_joint_2/position [available] [claimed]
		wbot_arm_joint_3/position [available] [claimed]
		wbot_arm_joint_4/position [available] [claimed]
		wbot_arm_joint_5/position [available] [claimed]
		wbot_arm_joint_6/position [available] [claimed]
		wbot_arm_gripper_joint/position [available] [claimed]
Hardware Component 2
	name: wbot_base_control
	type: system
	plugin name: joint_state_topic_hardware_interface/JointStateTopicSystem
	state: id=3 label=active
	read/write rate: 100 Hz
	is_async: False
	command interfaces
		wbot_wheel_left_joint/velocity [available] [claimed]
		wbot_wheel_right_joint/velocity [available] [claimed]
```

## Moving the robot
Now that we are running the base against it's "real" hardware interface and the arm is running the generic mock system we can ask them to move around and entire system should operate as if hardware were available to command and move.

### DiffDriveController
To drive the base around start a twist publisher that send commands to the `DiffDriveController` which calculates wheel velocity commands and sends them to the ESP embedded hardware.
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

<img src="wbot_teleop.gif">

### JointTrajectoryController
The `JointTrajectoryController` offers 2 different interfaces to move its joints.
There is an action interface where it takes [FollowJointTrajectory](https://github.com/ros-controls/control_msgs/blob/master/control_msgs/action/FollowJointTrajectory.action) action requests. (This is what MoveIt is configured to use by default)
```bash
ros2 action info /joint_trajectory_controller/follow_joint_trajectory
# Action: /joint_trajectory_controller/follow_joint_trajectory
# Action servers: 1
#     /joint_trajectory_controller
```
Or there is a topic command interface
```bash
ros2 topic info -v /joint_trajectory_controller/joint_trajectory
# Type: trajectory_msgs/msg/JointTrajectory
#
# Subscription count: 1
#
# Node name: joint_trajectory_controller
# Topic type: trajectory_msgs/msg/JointTrajectory

```

To test the topic interface lets send the arm to a different pose and give it a few seconds to get there.
```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
  joint_names: [wbot_arm_joint_1, wbot_arm_joint_2, wbot_arm_joint_3, wbot_arm_joint_4, wbot_arm_joint_5, wbot_arm_joint_6, ],
  points: [
    { positions: [0.0, 0.85, -0.75, 0.0, 0.5, 0.0], time_from_start: { sec: 2 } },
  ]
}" -1
```

and back home
```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
  joint_names: [wbot_arm_joint_1, wbot_arm_joint_2, wbot_arm_joint_3, wbot_arm_joint_4, wbot_arm_joint_5, wbot_arm_joint_6, ],
  points: [
    { positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: { sec: 1 } },
  ]
}" -1
```
<img src="wbot_manipulator_move.gif">

### GripperActionController
The last controller that is running is there to open and close the gripper.
The `GripperActionController` takes action requests on the `/gripper_controller/gripper_cmd` topic where the target position can be specified.

Open:
```bash
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/ParallelGripperCommand "{command: {name: [wbot_arm_gripper_joint], position: [0.03]}}"
```
Closed:
```bash
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/ParallelGripperCommand "{command: {name: [wbot_arm_gripper_joint], position: [0.0]}}"
```

<img src="wbot_gripper.gif">
