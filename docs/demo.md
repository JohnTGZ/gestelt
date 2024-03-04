# Demo
These are instructions for demo in the Vicon room with multiple drones.

## Pre-flight checks
1. FCU Set-up
    - Make sure important flight params have been flashed properly.
        - MAV_SYS_ID MUST be unique for each drone
    - Calibrate sensors. 
    - Make sure transmitter is working (visually inspect in QGroundControl "radio" as you move the gimbals)
    - Do motor tests

2. Onboard computer
    - Test the Onboard computer to FCU connection via MAVROS node.

3. Simulation
    - Do test runs of the simulated environment.

4. Pre-flight
    - Make sure drone is receiving location via Vicon
    - Make sure the same map is being received.

5. Flight
    - Test take-off and landing.
    - Test planning without physical obstacles.
    - Test planning with physical obstacles.

## Bill of materials
1. Turn on Vicon computer. Make sure vicon software is up and running.
2. Batteries x 5 (charged).
3. Laptop (charged).
4. Radio Controller (Charged).
5. Drones x 2.
6. Huge monitor for presentation plugged to Laptop.

## Set-up 
1. Make sure drone is connected to the right wifi network
```bash
sudo nmcli dev wifi connect "wifi_name"
```

### Offboard computer (Drone)
```bash 
# Drone 0 (192.168.31.205)
ssh rock@192.168.31.205
uav_startup 0 
# Drone 1 (192.168.31.150)
ssh rock@192.168.31.150
uav_startup 1
```

### Host PC
1. Start ROSCore
2. Start Vicon central
```bash 
cd_scripts && cd vicon
./vicon_central.sh
```s

### Commands
```bash
# Land the drone
rostopic pub /traj_server_event std_msgs/Int8 "data: 1" --once
# Switch to hover mode
rostopic pub /traj_server_event std_msgs/Int8 "data: 3" --once
# Emergency stop
rostopic pub /traj_server_event std_msgs/Int8 "data: 4" --once
```

### Copying bringup files
```bash
# Radxa 0
scp -r /home/john/gestelt_ws/src/gestelt/gestelt_bringup/ rock@192.168.31.205:/home/rock/gestelt_ws/src/gestelt/
# Radxa 1
scp -r /home/john/gestelt_ws/src/gestelt/gestelt_bringup/ rock@192.168.31.150:/home/rock/gestelt_ws/src/gestelt/
```

# Simulation tests

## On central computer
```bash
cd_scripts && cd demo
./demo_ctl.sh
# When ready, launhc the mission
```

## To simulate the actual drone
```bash
cd_scripts && cd demo
./gz_sim_single_uav_demo_off_0.sh
./gz_sim_single_uav_demo_off_1.sh
```
