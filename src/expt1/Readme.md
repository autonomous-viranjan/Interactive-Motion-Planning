# Experiment 1
Mandatory lane change (MLC) scenario: a stationary truck in ego's lane. Ego vehicle needs to negotiate with human driven vehicle to change lane.
- MIQP MPC for high-level motion planning
  - Auto Lane Selection
  - **Constant velocity NV model** for over the horizon predictions
- PID controller for low-level control

## To run:
`catkin_make` the `Interactive-Motion-Planning` workspace. Modify `CMakeLists.txt` as required.

### Running with human driven NV
1. Run the CARLA server `./CarlaUE4.sh -prefernvidia`
2. Run ros master `roscore`
3. In the ROS workspace `cd ~/CARLA_0.9.14/PythonAPI/Interactive-Motion-Planning` and `source devel/setup.bash` in three new terminals
4. Run Ego vehicle client via `rosrun expt1 ego_vehicle.py`
5. Run Neighbor vehicle client via `rosrun expt1 NV.py`
6. Run stopped vehicle client via `rosrun expt1 stopped_vehicle.py`
7. Run Planner via `rosrun expt1 planner1`

#### G920 setup
- To set up G920 ensure `pyLinuxWheel` is running. jstest-gtk is used to get axis numbers used in `NV.py`
- if using multiple computers, `rosrun expt5 g920.py` to connect G920 via ROS to `NV_.py`
