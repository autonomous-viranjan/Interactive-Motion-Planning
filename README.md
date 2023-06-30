# Interactive Motion Planning

- Real-time Software-in-the-loop simulation in CARLA with Human-in-the-loop capability.
- MPC written in C++ and controller written in Python.
- ROS (Noetic) for communication.

## How to run:
`catkin_make` the `ros_ws`. Modify `CMakeLists.txt` as required.

### Running with human driven NV
1. Run the CARLA server `./CarlaUE4.sh -prefernvidia`
2. Run ros master `roscore`
3. In the ROS workspace source `source devel/setup.bash` in three new terminals
4. Run Ego vehicle via `rosrun test Ego_vehicle.py`
5. Run Planner via `rosrun test planner`
6. Run Neighbor vehicle via `rosrun test NV.py`

(Instructions for setting up wheels for driving available in NV.py)

### Running with predefined NV
1. Run the CARLA server `./CarlaUE4.sh`
2. Spawn predefined NV. `cd ~/CARLA_0.9.14/PythonAPI/LaneChangeMPC/ros_ws/src/test/src` and run `python3 stopped_veh.py` for example
3. Run ros master `roscore`
4. In the ROS workspace source `source devel/setup.bash` in two new terminals
5. Run Ego vehicle via `rosrun test Ego_vehicle.py`
6. Run Planner via `rosrun test planner`
