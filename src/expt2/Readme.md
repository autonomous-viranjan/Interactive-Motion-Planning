# Experiment 2
Mandatory lane change (MLC) scenario: a stationary truck in ego's lane. Ego vehicle needs to negotiate with human driven vehicle to change lane.
- MIQP MPC for high-level motion planning
  - Auto Lane Selection
  - **Joint optimization** (MIQP MPC) for over the horizon predictions
- PID controller for low-level control

## To run:
`catkin_make` the `Interactive-Motion-Planning` workspace. Modify `CMakeLists.txt` as required.

### Running with human driven NV
1. Run the CARLA server `./CarlaUE4.sh -prefernvidia`
2. Run ros master `roscore`
3. In the ROS workspace `cd ~/CARLA_0.9.14/PythonAPI/Interactive-Motion-Planning` and `source devel/setup.bash` in three new terminals
4. Run Ego vehicle client via `rosrun expt2 ego_vehicle.py`
5. Run Neighbor vehicle client via `rosrun expt2 NV.py`
6. Run stopped vehicle client via `rosrun expt2 stopped_vehicle.py`
7. Run Planner via `rosrun expt2 planner2`

[Instructions for setting up wheels for driving are available in NV.py]
