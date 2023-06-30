# Experiment 1 
- MIQP MPC based formulation for high-level motion planning
  - Auto Lane Selection
- PID controller for low-level control.

## To run:
`catkin_make` the `Interactive-Motion-Planning` workspace. Modify `CMakeLists.txt` as required.

### Running with human driven NV
1. Run the CARLA server `./CarlaUE4.sh -prefernvidia`
2. Run ros master `roscore`
3. In the ROS workspace `cd ~/CARLA_0.9.14/PythonAPI/Interactive-Motion-Planning` source `source devel/setup.bash` in three new terminals
4. Run Ego vehicle via `rosrun expt1 Ego_vehicle.py`
5. Run Planner via `rosrun expt1 planner`
6. Run Neighbor vehicle via `rosrun expt1 NV.py`

(Instructions for setting up wheels for driving available in NV.py)

### Running with predefined NV
1. Run the CARLA server `./CarlaUE4.sh`
2. Spawn predefined NV. `cd ~/CARLA_0.9.14/PythonAPI/Interactive-Motion-Planning/src/expt1/src` and run `python3 stopped_veh.py` for example
3. Run ros master `roscore`
4. In the ROS workspace source `source devel/setup.bash` in two new terminals
5. Run Ego vehicle via `rosrun expt1 Ego_vehicle.py`
6. Run Planner via `rosrun expt1 planner`
