Please provide us ROSCon UK [feedback here](https://forms.gle/1QM9HWDUekFCRc7b7)

# ROSCon 2025 ros2_control Workshop

## Docker

### Prerequisite
1. Install [docker compose](https://docs.docker.com/compose/install/linux/#install-using-the-repository).

### Docker Pull
1. Clone this repo and navigate to the directory `<path of cloned repo>/zehoh_host`

1. Get the latest container build with `docker compose pull`

#### If pulling fails, build
1. Build the Docker image with: `docker compose build`

It is possible you cannot pull due to network access or using an architecture for which there is no build available online.

### Docker Run
1. To run applications that have a GUI inside Docker (i.e. Rviz or Plotjuggler) we have to allow access to the screen by running: `xhost +` on the host PC once per machine startup.

2. To start the container run:
```
cd <path of cloned repo>/zehoh_host
xhost + && docker compose up -d
```

3. To open an interactive shell to the running container run: `docker exec -it ros2_control_roscon25 bash`

Handy alias to add to the `.bashrc`:
```
echo 'alias rc="docker exec -it ros2_control_roscon25 bash"' >> ~/.bashrc
```

4. To verify you have the container up at any time, you can run `docker ps`, you should see something similar:
```
CONTAINER ID   IMAGE                                                     COMMAND                  CREATED        STATUS        PORTS     NAMES
47852bf550b2   ghcr.io/ros-controls/roscon2025_control_workshop:latest   "/ros_entrypoint.sh …"   1 hours ago   Up 1 hours             ros2_control_roscon25
```

## Task 1:


1. Once in the container, let's start the zenoh daemon: `z`. This is also an alias, don't worry.

2. Open a new terminal, `rc`, then in the new shell inside the container, run `ros2 launch wbot_bringup wbot.launch.xml`. This will bring up `rviz` with a simulated robot in there.

3. Open a new terminal, `rc`, then run `rqt_graph` & inspect the graph. Close `rqt_graph` but keep the terminal open.

4. Make sure `rviz` is visible, let's teleop this bot: `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true`

5. Let's try the CLI tools too! Close the teleop tool and let's run a few commands
```
ros2 control list_controllers
ros2 control list_controller_types
ros2 control list_hardware_components -v
ros2 control list_hardware_interfaces
```

6. Close all terminals.

## Task 2:

1. Take your ESP32 and plug a data-capable USB-C cable into the port labeled "COM" (see on the back).
2. Verify that the device shows up on your laptop. Open a new terminal and run `ls /dev/ttyACM*` or `ls /dev/ttyUSB*`. We have an alias in the container expecting `/dev/ttyACM0` but we can override it.
3. Go to the container now, `rc`, and `z` should start up the zenoh daemon with serial device support.
In case you have a different device path than `/dev/ttyACM0`, override it by running `echo "alias z='ZENOH_CONFIG_OVERRIDE=\"listen/endpoints=[\\\"tcp/[::]:7447\\\",\\\"serial//dev/YOUR_DEVICE_PATH#baudrate=460800\\\"]\" ros2 run rmw_zenoh_cpp rmw_zenohd'" >> /root/.bashrc`. Now you can `source ~/.bashrc` and `z` should work fine. Since we won't take the container down, your setup should be fine for the rest of the day.
4. Let's inspect what we have running at the moment.
```
ros2 topic list
ros2 topic echo ...
ros2 topic pub {JSON}
```

#### Scenario

#### What to focus on

#### Running the example




## Embedded projects:

#### [Sine Wave Twist Publisher](twist_publisher/README.md)

#### [Joint State Publisher](jointstate_publisher/README.md)
