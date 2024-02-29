This tutorial helps with setup and launch Autoware Humble with CARLA 0.9.15 simulator in Docker environment. Reports and improvement suggestions are very welcome.

## Requirements

* Ubuntu
   * This tutorial was tested on Ubuntu 20.04, but the same should be possible on other Ubuntu distributions
* PC x86/64bit with GPU installed 

### Update and install git

Before you go to the following sections update your system and install git.

```
sudo apt-get -y update
sudo apt-get -y install git
```

## Section 1 Set up a development environment

This step describes how to install [Autoware Universe](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/docker-installation/) from `humble` branch via **Docker Installation** and how to install [`carla-autoware-bridge`](https://github.com/Robotics010/carla_autoware_bridge) to allow Autoware-CARLA communication.

### 1.1 Install Autoware Universe

Clone `autowarefoundation/autoware` repo and move to the directory.

```
git clone https://github.com/autowarefoundation/autoware.git -b humble
cd autoware
```

### 1.2 Install the dependencies

Caution! Pay attention to what cuda and nvidia driver you need and do accordinally. If you just execute the following setup shell script without understanding it, you may end up with an invalid nvidia/cuda drivers setup.

Install the dependencies using the provided Ansible script

```
./setup-dev-env.sh docker
```

Download required artifacts

```
./setup-dev-env.sh universe --download-artifacts
```

### 1.3 Prepare map of the CARLA town

Create the autoware_map directory for map data

```
mkdir ~/autoware_map
```

Clone [autoware-contents](https://bitbucket.org/carla-simulator/autoware-contents/src/master/) to a temp directory, then copy `maps/point_cloud_maps/Town01.pcd` and `maps/vector_maps/lanelet2/Town01.osm` to `~/autoware_map/carla-town-1/` and rename those to `pointcloud_map.pcd` and `lanelet2_map.osm` accordingly.

## Section 2 CARLA installation

### 2.1 CARLA server installation

Download [CARLA server 0.9.15](https://carla.readthedocs.io/en/0.9.15/start_quickstart/#carla-installation) as `CARLA_0.9.15.tar.gz` from the [release page](https://github.com/carla-simulator/carla/releases/tag/0.9.15/).

Extract it to `~/CARLA_0.9.15` folder.

### 2.2 CARLA client installation and test CARLA setup (optional)

Install [carla python client 0.9.15](https://carla.readthedocs.io/en/0.9.15/start_quickstart/#install-client-library) via pip:

```
pip3 install carla==0.9.15
```

Launch carla server:

```
cd ~/CARLA_0.9.15
./CarlaUE4.sh
```

Check it with, by executing the following in a terminal:

```
cd ~/CARLA_0.9.15/PythonAPI/examples
python3 -m pip install -r requirements.txt
python3 generate_traffic.py
```

Then in other terminal execute the following:

```
cd ~/CARLA_0.9.15/PythonAPI/examples
python3 manual_control.py
```

## Section 3 CARLA autoware bridge installation

### 3.1 Set up a workspace


Pull the docker image

```
docker pull ghcr.io/autowarefoundation/autoware-universe:latest-cuda
```

Start the docker contatiner

```
rocker --nvidia --x11 --user --volume $HOME/autoware --volume $HOME/autoware_map --volume $HOME/autoware_data --network host -- ghcr.io/autowarefoundation/autoware-universe:latest-cuda
cd autoware
```

Warning, by default an exiting from a terminal, where you've started rocker will stop and delete docker contatiner.

### 3.2 Add repositories required for Autoware and CARLA communication

Add `ros-bridge`, `carla_autoware_bridge`, `carla_tesla_model3_description`, `carla_launch` and `carla_control_launch` repos to `autoware.repos` list.

Go to autoware/autoware.repos files and use your text editor to add the following new repositories.

```
  # carla
  carla/ros-bridge:
    type: git
    url: https://github.com/Robotics010/ros-bridge.git
    version: master
  carla/astuff_sensor_msgs:
    type: git
    url: https://github.com/astuff/astuff_sensor_msgs.git
    version: master
  carla/carla_autoware_bridge:
    type: git
    url: https://github.com/Robotics010/carla_autoware_bridge.git
    version: master
  carla/carla_tesla_model3_description:
    type: git
    url: https://github.com/Robotics010/carla_tesla_model3_description.git
    version: master
  carla/carla_launch:
    type: git
    url: https://github.com/Robotics010/carla_launch.git
    version: master
  carla/carla_control_launch:
    type: git
    url: https://github.com/Robotics010/carla_control_launch.git
    version: master
```

[`Robotics010/ros-bridge`](https://github.com/Robotics010/ros-bridge) is a fork from [`carla-simulator/ros-bridge`](https://github.com/carla-simulator/ros-bridge) and have [some changes](https://github.com/Robotics010/ros-bridge/blob/b183848fc5fa35a35a6f3381466ea245f14cfc29/CHANGELOG.md#fork-changes), that were required by Autoware.

### 3.3 Clone repositories and install dependencies

Create the src directory and clone repositories into it. Autoware uses [vcstool](https://github.com/dirk-thomas/vcstool) to construct workspaces.

```
mkdir src
vcs import --recursive src < autoware.repos
```

Install dependent ROS packages.

```
sudo apt update
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

And build the workspace.

```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 3.4 CARLA client installation (inside Docker)

Install [carla python client 0.9.15](https://carla.readthedocs.io/en/0.9.15/start_quickstart/#install-client-library) via pip:

```
pip3 install carla==0.9.15
```

## Section 3 Launching ad hoc simulation

Here is a list of steps and commands to launch Autoware Universe ad hoc simulation with CARLA.

### 3.1 Launch CARLA server (at Host)

```
cd ~/CARLA_0.9.15
./CarlaUE4.sh
```

You may find these arguments useful while executing CARLA:
* `-prefernvidia` - use NVIDIA GPU for hardware acceleration
* `-carla-rpc-port=3000` - use other than 2000 default port for RPC service's port
* `-quality-level=Low` - use low quality level mode for a minimal video memory consumption

### 3.2 Launch carla_autoware_bridge (in Docker)

Launch `carla_autoware_bridge`, which spawns ego vehicle as well. 

```
<!-- exec bash in container -->
source ~/autoware/install/setup.bash
ros2 launch carla_autoware_bridge carla_autoware_demo.launch.py
```

Here you can add the following arguments

* `port:=3000` to switch to a different CARLA port for it's RPC port
* `timeout:=10` to increase waiting time of loading a CARLA town before raising error
* `view:=true` to show a third-person-view window

### 3.3 Launch Autoware Universe

And launch Autoware software stack with the following commands:

```
source ~/autoware/install/setup.bash
ros2 launch carla_launch e2e_simulator.launch.xml map_path:=$HOME/autoware_map/carla-town-1 vehicle_model:=carla_tesla_model3 sensor_model:=sample_sensor_kit
```

At this step your desktop should look like:

![state_after_start](images/state_after_start.png)

### 3.4 Set start location

Set ego vehicle start location using 2D Pose Estimate tool (highlighted by red color)

![pose_estimate_tool](images/pose_estimate_tool.png)

Optionally you can attach current view to the vehicle by selecting `base_link` as `Target Frame`.

![target_frame](images/target_frame.png)

### 3.5 Send route and engage

Finally send target location and allow engaging vehicle

![engaging_vehicle](images/engaging_vehicle.png)

## See also

* Go to [Troubleshooting](troubleshooting.md) section in order to fix some known problems.
