# Experiment 5
Mandatory lane change (MLC) scenario: a stationary truck in ego's lane. Ego vehicle needs to negotiate with human driven vehicle to change lane.
- aiMPC for high-level motion planning
  - NV cost terms: proximity and acceleration
      - High proximity imputed weight implies high interaction
      - High acceleration (minimization) imputed weight implies low interaction
  - Auto Lane Selection
- PID controller for low-level control

## To run:
`catkin_make` the `Interactive-Motion-Planning` workspace. Modify `CMakeLists.txt` as required.

### Running with human driven NV
1. Run the CARLA server `./CarlaUE4.sh -prefernvidia`
2. Run ros master `roscore`
3. In the ROS workspace `cd ~/CARLA_0.9.14/PythonAPI/Interactive-Motion-Planning` and `source devel/setup.bash` in three new terminals
4. Run Ego vehicle client via `rosrun expt5 ego_vehicle.py`
5. Run stopped vehicle client via `rosrun expt5 stopped_vehicle.py`
6. Run Neighbor vehicle client via `rosrun expt5 NV.py`
7. Run Planner via `rosrun expt5 planner5`

#### G920 setup
- To set up G920 ensure `pyLinuxWheel` is running. jstest-gtk is used to get axis numbers used in `NV.py`
- if using multiple computers, `rosrun expt5 g920.py` to connect G920 via ROS to `NV_.py`
