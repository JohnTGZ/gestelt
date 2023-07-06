# radxa_utils
This package contains useful scripts and launch files to setup and test the radxa communications.

# Setting up the Radxa

```bash

# Install tools
sudo pip3 install pyamlboot

# Use lsusb command to check if radxa is connected
lsusb
# Use dmesg to find the device filepath
sudo dmesg

# To erase the eMMC 
# https://wiki.radxa.com/Zero/install/eMMC_erase
sudo boot-g12.py radxa-zero-erase-emmc.bin

# Get images from https://wiki.radxa.com/Zero/downloads
xz -v --decompress IMAGE_COMPRESSED

# Flash the image using balena etcher

# Proceed on activating WIFI using https://wiki.radxa.com/Zero/Ubuntu
# Root username and pasword is rock/rock
sudo su
nmcli r wifi on
nmcli dev wifi
nmcli dev wifi connect "wifi_name" password "wifi_password"

# Install ROS at http://wiki.ros.org/noetic/Installation/Ubuntu

# Run setup script
./radxa_setup.sh

# Copy .bashrc configuration file over
export ROS_DISTRO="noetic"
source /opt/ros/noetic/setup.bash
alias sros="source /opt/ros/noetic/setup.bash"
alias sws="source devel/setup.bash"
alias sbash="source ~/.bashrc"

# Set up network
# http://wiki.ros.org/ROS/NetworkSetup
export ROS_MASTER_URI=http://MASTER_IP:11311
export ROS_HOSTNAME=OWN_IP
export ROS_IP=OWN_IP

# Convenience function
alias pull_repo="git -C ~/gestelt_ws/src/gestelt/ pull"
alias catkin_make_release="catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_BLACKLIST_PACKAGES="rviz_plugins;manual_take_over;odom_visualization;pose_utils;uav_utils;drone_detect""
```

# Testing communications
Assuming we just have 2 machines A (192.168.31.173) and B (192.168.31.166) connected to the same LAN.
They all have to have the same ROS_MASTER_URI. In this case, the ROS_MASTER_URI is "http://192.168.31.173:11311". A is the talker and B is the listener.


```bash
# Use ifconfig to figure out their IPs within the LAN

# On machine A:
roslaunch radxa_utils talker.launch ros_master_uri:=http://192.168.31.173:11311 ros_ip:=192.168.31.173

# On machine B:
roslaunch radxa_utils listener.launch ros_master_uri:=http://192.168.31.173:11311 ros_ip:=192.168.31.166
```

Setting environment variables manually
```bash
# Matchine A
export ROS_MASTER_URI=http://192.168.31.173:11311
export ROS_HOSTNAME=192.168.31.173
export ROS_IP=192.168.31.173

# Matchine B
export ROS_MASTER_URI=http://192.168.31.173:11311
export ROS_HOSTNAME=192.168.31.166
export ROS_IP=192.168.31.166
```

# Quick Start

## Single computer test
```bash
# On the central flight control computer
cd ~/gestelt_ws/src/gestelt/gestelt_bringup/scripts
./radxa_central_sitl.sh http://192.168.31.173:11311 192.168.31.173

# On the UAV (ID 0)
cd ~/gestelt_ws/src/gestelt/gestelt_bringup/scripts
./radxa_uav.sh 0 http://192.168.31.173:11311 192.168.31.173
```

## Computer and radxa
```bash
# On the central flight control computer
cd ~/gestelt_ws/src/gestelt/gestelt_bringup/scripts
./radxa_central_sitl.sh http://192.168.31.173:11311 192.168.31.173

# On the UAV (ID 0)
cd ~/gestelt_ws/src/gestelt/gestelt_bringup/scripts
./radxa_uav.sh 0 http://192.168.31.173:11311 192.168.31.166
```

# TODO
Communication between different machines happen on a wifi network

1. (PX4 SITL + Gazebo on PC) <-> (Egoplanner on Radxa)   
    -   Data
        - Bandwidth: Using `iperf`
            - Bandwidth:
                - TCP: 57.3 Mbits/sec 
                - UDP: 90Mbits/sec - 101 Mbits/sec
        - Latency: Using `sudo mtr --no-dns --report --report-cycles 60 IP_ADDR`. 
            - Latency averages 14.8 ms. Best is 8.7 ms, Worst is 32.1 ms.
        - CPU Usage: Using `htop`
            - it is shown to use around (55%, 30%, 30%, 30%) for the Radxa's 4 cores
    - Sources of large bandwidth
        - Depth camera sensor data
    - Potential solutions to reduce bandwidth required
        - Compress depth image before sending over. Or only send point clouds over (Which have been compressed)
2. (PX4 HITL) + (Gazebo + Egoplanner on PC)
    - Can be done with Flywoo F405S
    - Cannot be done with PX4 FMUV2 (Pixhawk 1 FCU). FCU's flash memory is too low to take off additional HITL modules.
    
3. (PX4 HITL) <-> (Gazebo on PC) <-> (Egoplanner on Radxa)

# Issues
1. Mesh file location is different on the host computer than on the UAV
    - Easy fix: Use the file path for the central computer
2. Having PX4 SITL on the central computer will result in latency for the PVA commands sent to the drone
    - Use PX4 HITL with radxa