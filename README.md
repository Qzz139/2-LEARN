**English** | [简体中文](README_cn.md)

# RoboMaster EP Ground-Level Bottle Pick-and-Place Simulation

Runs Gazebo 11 + ROS 2 Foxy + MoveIt 2 on a Jetson Orin. With the chassis fixed to the ground, a two-axis robotic arm picks up an upright bottle at a fixed point A, moves it to point B on the ground, and releases it. The scene contains no pedestal.

The project reuses EP meshes and Xacro files from `jeguzzi/robomaster_ros` and retains the upstream MIT license. The bottle is a rigid-body proxy approximating a 550 ml bottle; its dimensions, empty mass, water volume, and friction are configurable. **Simulation results cannot establish that a PET bottle will not deform or that the physical robot can grasp a full bottle.**

The original course assignment specifies a tabletop. The ground-level setup is a user-requested scenario change and requires the instructor's acceptance. See [Course Requirements and Coverage (Chinese)](docs/course-requirements.md), [Model Limitations (Chinese)](docs/model.md), [Ground-Level Bottle Validation (Chinese)](docs/bottle-validation.md), and [Physical Robot Preparation Plan (Chinese)](docs/real-control-plan.md).

## Physical Robot Preparation: Communication Check

[Physical Robot Workflow (Chinese)](docs/real-pick.md): first record the four positions with `bash scripts/run_real.sh --teach home/pick/place/safe` (choose one name per invocation). Then preview the sequence with `bash scripts/run_real.sh`, or add `--execute` to run “return HOME → grasp at A → release at B → return HOME.” Motion is refused without calibration; parameters are automatically saved locally to `config/real_pick.local.json`. This entry point uses the SDK directly and is not yet integrated with a shared ROS 2 interface for the physical robot. The existing simulation is unchanged, and the complete physical robot workflow has not yet been validated.

After connecting the Jetson to the EP hotspot, run `python3 scripts/check_ep_connection.py`. This checks SDK communication, version information, and robotic arm position feedback without sending motion commands. Results are saved to `work/ep-connection.json`. See [Physical Robot Connection Notes (Chinese)](docs/real-connection.md) for instructions, the observed coordinate decoding anomaly, and next steps.

## Project Structure

```text
src/
  robomaster_description/  Reused upstream EP meshes, Xacro files, and license
  ep_gazebo_control/       Simulated motor drivers supporting standard joint trajectory actions
  ep_simulation/          Model, planar IK, bottle configuration, scene, task, and launch files
  ep_moveit_config/       MoveIt/OMPL, controller, and RViz configuration
scripts/                  Build, run, and stop entry points
config/                   Physical robot connection parameters (no passwords)
tests/                    Topology, kinematics, reachability, and scene tests
docs/                     Course requirements, sources, validation data, and physical robot plans
work/                     Logs and trajectory results (not committed to Git)
build/ install/ log/      colcon caches (not committed to Git)
```

## Installation and Build

On Ubuntu 20.04 / ROS 2 Foxy, with Gazebo 11 already installed:

```bash
sudo apt-get install -y --no-remove \
  python3-colcon-common-extensions python3-yaml \
  ros-foxy-xacro ros-foxy-robot-state-publisher \
  ros-foxy-rviz2 ros-foxy-moveit ros-foxy-gazebo-ros-pkgs \
  ros-foxy-gazebo-ros2-control ros-foxy-controller-manager \
  ros-foxy-joint-state-broadcaster ros-foxy-joint-trajectory-controller
bash scripts/build.sh
```

## Start the Full System and Task with One Launch

Run on the Jetson, after ensuring that no other session of this project is using the simulation ports:

```bash
cd /home/robot/projects/2-LEARN
export DISPLAY=:2
export XAUTHORITY=/home/robot/.Xauthority
bash scripts/run_sim.sh run_task:=true cycles:=1
```

The same underlying `ep_simulation sim.launch.py` starts Gazebo, MoveIt, controllers, state nodes, RViz, and the task node. Use `gui:=false rviz:=false` to disable visualization, or `cycles:=5` for five acceptance runs. For automated background validation, use `python3 scripts/session.py start --headless --run-task --cycles 5`; results are written to `work/bottle-five.json`. Press Ctrl+C to shut down a foreground session.

Background sessions and manual task execution:

```bash
python3 scripts/session.py start --display :2
bash scripts/pick.sh --cycles 1 --output work/bottle-result.json
python3 scripts/session.py status
python3 scripts/session.py stop
```

For consecutive acceptance runs, use `bash scripts/pick.sh --cycles 5 --output work/bottle-five.json`. Five A→B transfers take place in the same simulation world. Between runs, the arm physically carries the bottle back from B→A. These four resets are recorded separately and do not count toward the five acceptance runs; objects are never teleported, and the world is not restarted. If a task fails, subsequent actions stop rather than continuing blind grasp attempts to reach the requested count.

## Configure the Bottle and Points A/B

`src/ep_simulation/config/bottle.json` is the shared configuration source:

- Approximate bottle height: 227 mm; diameter: 64 mm. Update these values after measuring the actual batch of bottles.
- Default assumed empty mass: 25 g; water volume: 150 ml; total mass: 175 g. Using 150 ml does not guarantee that the bottle will not deform.
- Point A: x=275 mm; point B: x=320 mm. The y coordinate is fixed within the arm's plane of motion.
- Grasp the bottle 105 mm above the ground and lift it by 50 mm. During placement, adjust the height for observed grasp slippage so that the bottle bottom has approximately 2 mm of clearance before release and settling onto the ground.
- `open_half_gap_m` is the half-opening on one side, not the total gripper opening.

The configuration uses world coordinates and meters. `model.ik(x,z)` converts the target into two independent joint angles. Reachability and coupled joint limits are checked before startup, and every waypoint in each MoveIt trajectory is checked again. The fixed chassis cannot move the bottle sideways out of the arm's plane.

Restart the simulation after changing the default configuration. When using a separate configuration file, the launch argument `bottle_config:=/absolute/path/bottle.json` and the manual task argument `--config /absolute/path/bottle.json` must reference the same file. With `run_task:=true`, the configuration is shared automatically.

## Outputs and Limitations

[Download the Simulation Demo Video](docs/videos/ground-bottle-demo.mp4): a complete A→B pick-and-place sequence recorded on 2026-09-10, including homing, grasping, lifting, transfer, release, and return. See the corresponding [Trajectory and Execution Results](docs/validation/bottle/recorded-demo.json). The video is a desktop recording at its original speed; Gazebo's low rendering frame rate causes visible stuttering. See [Video Notes (Chinese)](docs/videos/README.md) for details.

`work/pick_result.json` stores the configuration, results for each run and reset, actual bottle position and tilt, trajectory joint positions/velocities/accelerations/timestamps, and errors. Session logs are stored in `work/sim.log`. Each acceptance run requires an actual lift of ≥25 mm, an error at B of ≤12 mm, bottle tilt of ≤10°, and drift of ≤3 mm over a one-second stability sampling period. Failed runs return a nonzero exit code.

MoveIt carries the bottle collision object with the gripper. In Gazebo, the grasp relies on gripper contact friction without a fixed attachment. Robot inertia, linkages, and the gripper remain simplified proxies and cannot replace full dynamics or damage validation. The shared ROS 2 interface for the physical robot has not yet been implemented; the direct SDK entry point is described above.

The simulation defaults to ROS domain 187 with local-only communication and Gazebo port 11355; it does not connect to a physical EP. Do not also source ROS 1 Noetic in the same terminal.

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
python3 -m unittest discover -s tests -v
```

Legacy validation with a block on a pedestal is retained in `docs/validation.md` (Chinese) and represents only the older version's results.

## Course Assignment Requirements

<img width="590" height="766" alt="Original course assignment requirements (Chinese)" src="https://github.com/user-attachments/assets/9053b3c5-37d4-46f7-9a4b-58b216af0e4b" />

See [Course Requirements (Chinese)](docs/course-requirements.md) for a text transcription and scope notes.
