Please provide us [feedback here](TODO) ## TODO

# ROSCon 2025 ros2_control Workshop

## Docker

### Prerequisite
1. Install [docker compose](https://docs.docker.com/compose/install/linux/#install-using-the-repository).

1. On the host computer download and install [VS Code](https://code.visualstudio.com/download)

1. Open VS Code and install the [PlatformIO](https://platformio.org/install/ide?install=vscode) Extension

### Docker Build
1. Clone this repo and navigate to the directory `<path of cloned repo>/zehoh_host`

1. Build the Docker image with: `docker compose build`

### Docker Run
1. To run applications that have a GUI inside Docker (i.e. Rviz or Plotjuggler) we have to allow access to the screen by running: `xhost +` on the host PC once per machine startup.

1. To start the container run: `docker compose up -d`

1. To open an interactive shell to the running container run: `docker exec -it ros2_control_roscon25 bash`


## Task 1:

####

## Task 2:

#### Scenario

#### What to focus on

#### Running the example




## Embedded projects:

#### [Sine Wave Twist Publisher](twist_publisher/README.md)

#### [Joint State Publisher](jointstate_publisher/README.md)
