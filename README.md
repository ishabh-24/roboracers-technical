# Penn RoboRacer Technical Challenge: LiDAR Wall Follower

A ROS 2 (Humble) wall-following node for the AutoDRIVE RoboRacer practice
track. The car keeps itself centred between the walls with a PID loop on a
lookahead-projected wall distance, blends in follow-the-gap steering as corners
close in, and runs a speed controller that brakes for the hairpins and opens up
on the straight.

## Results

| | |
|---|---|
| **Video (unlisted)** | TODO_YOUTUBE_LINK |
| **Fastest clean lap** | 11.8 s (HUD best lap in the video; laps 2 to 4 of the recording are 11.8, 11.8 and 12.0 s with the collision counter at 0) |
| Consecutive clean laps | 3 in the video, 7 in the validation run below |

Validation run with the default parameters, straight from `ros2 run` with no
overrides (collision counter stayed at 0 for the whole run):

| Lap | Time (s) | Lap | Time (s) |
|---|---|---|---|
| 1 | 11.85 | 5 | 12.06 |
| 2 | 11.84 | 6 | 11.90 |
| 3 | 11.87 | 7 | 12.04 |
| 4 | 11.89 | | |

The track is about 44 m long and only about 1.4 m wide, with a 19 m straight,
two 180° hairpins and a zigzag of three tight corners.

## What is in this repo

| Path | Purpose |
|---|---|
| `autodrive_devkit/autodrive_roboracer/wall_follow_node.py` | The wall follower (the only new node) |
| `autodrive_devkit/setup.py` | Registers `wall_follow_node` as a console script |
| `autodrive_devkit/scripts/` | Helpers used while tuning: `ctl.sh` (start/stop/reset), `monitor.py` (lap and collision readout), `recorder.py` (logs scans and pose to `.npz`) |
| `autodrive_devkit/autodrive_roboracer/autodrive_bridge.py` | Simulator bridge from AutoDRIVE, unmodified |
| `autodrive_devkit/autodrive_roboracer/teleop_keyboard.py` | Keyboard teleop from AutoDRIVE, unmodified |
| `autodrive_devkit.Dockerfile`, `autodrive_devkit.sh` | Container used to build and run the package |

## How the controller works

All of the logic lives in `WallFollowController` inside `wall_follow_node.py`.
It is plain numpy with no ROS state, so it can be exercised outside ROS (it was
first checked in a small offline LiDAR simulator before touching the real one).

### Steering

1. **Scan cleaning.** NaN, inf and sub-minimum readings become the sensor's
   maximum range.
2. **Wall geometry (two-beam method).** For each side the node takes the beam
   perpendicular to the car (`b`, at ±90°) and a beam rotated `theta_deg`
   towards the front (`a`). The angle between heading and wall is
   `alpha = atan2(a·cosθ − b, a·sinθ)`, the current distance is `b·cos(alpha)`
   and the distance `lookahead` metres down the road is
   `d + lookahead·sin(alpha)`. Each beam is the mean of a small window so one
   noisy return does not jerk the wheel.
3. **Error.** In `center` mode the error is half the difference between the
   left and right projected distances. `left` and `right` modes hold
   `desired_distance` from one wall. Positive error always means "steer left".
4. **PID.** `kp`, `kd` (with a low-pass filtered derivative, since scans
   arrive at 60 to 100 Hz) and optional `ki` give a steering angle.
5. **Follow-the-gap blend.** The scan is run through a disparity extender
   (every range jump is widened by the car's safety radius so a beam that only
   grazes a wall edge is not treated as free), a bubble is cut around the
   nearest point, and the centre of the deepest run of beams is the gap
   direction. Its weight is `gap_min_weight` on open track and rises to 1 as
   the free distance ahead drops below `gap_full_dist`, so hairpins are turned
   into rather than driven into.
6. The blended angle is divided by the 0.52 rad full-lock angle, clipped to
   `[-1, 1]` and lightly low-passed.

### Speed

The target speed is the minimum of four limits, then clamped to
`[v_min, v_max]`:

- `v_stop = sqrt(2 · brake_decel · (front − stop_margin))`: the car must be able
  to stop before the wall ahead (front is the minimum range in a ±5° cone).
- `v_turn = sqrt(turn_speed² + 2 · brake_decel · (front − turn_dist))`: it must
  be down to `turn_speed` by the time the wall is `turn_dist` away, so braking
  is finished before the turn-in. This is what makes the hairpins repeatable.
- `v_corner = sqrt(lat_accel · R)`, where `R` is the smaller of the radius for
  the current steering angle and the pure-pursuit radius needed to reach the
  gap point.
- `v_max`.

Throttle is feed-forward (`v_target / speed_per_throttle`, the simulator does
roughly 25 m/s per unit of throttle) plus a proportional term on the measured
speed, and switches to `brake_throttle` when the car is more than `brake_band`
above target. The measured speed is the odometry twist, used purely as a
speedometer; steering never looks at anything but the LiDAR. Set
`use_odom_speed:=false` to run from the internal first-order speed model
instead.

### Safety

- **Recovery.** If the car is ever blocked (front below `block_dist` and
  stopped, or closer than `emergency_dist`) it reverses for `reverse_time`
  seconds steering away from the gap, so the nose swings towards the open
  side, then resumes.
- **Watchdog.** If no scan arrives for `watchdog_timeout` seconds the node
  publishes zero throttle and steering.
- The node logs one status line per second (error, front distance, wall
  distances, gap direction, blend weight, speed vs target, steering, throttle).

### Topics

| Direction | Topic | Type |
|---|---|---|
| Subscribe | `/autodrive/roboracer_1/lidar` | `sensor_msgs/LaserScan` |
| Subscribe | `/autodrive/roboracer_1/odom` | `nav_msgs/Odometry` (speed only) |
| Publish | `/autodrive/roboracer_1/throttle_command` | `std_msgs/Float32` |
| Publish | `/autodrive/roboracer_1/steering_command` | `std_msgs/Float32` |

## Running it

Prerequisites: Docker and the IROS 2026 practice simulator for your OS.

Build the image and start the container (macOS / Apple Silicon shown):

```bash
docker build --platform linux/amd64 -t penn-challenge -f autodrive_devkit.Dockerfile .
docker run --platform linux/amd64 --name penn-challenge --rm -it -p 4567:4567 \
  -v "$(pwd)/autodrive_devkit":/home/autodrive_devkit/src/autodrive_devkit \
  --entrypoint /bin/bash penn-challenge
```

Inside the container, build and start the bridge:

```bash
cd /home/autodrive_devkit
colcon build
source install/setup.bash
ros2 launch autodrive_roboracer bringup_headless.launch.py
```

Open the simulator, set IP `127.0.0.1` and port `4567`, click Connect, and
switch Driving Mode to Autonomous. In a second terminal:

```bash
docker exec -it penn-challenge bash
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
ros2 run autodrive_roboracer wall_follow_node
```

Do not run `teleop_keyboard` at the same time.

## Tuning

Every value in the `DEFAULTS` dictionary at the top of the node is a ROS 2
parameter. Override at launch:

```bash
ros2 run autodrive_roboracer wall_follow_node --ros-args -p v_max:=6.0 -p lat_accel:=4.0
```

or change it live while the car is driving:

```bash
ros2 param set /wall_follow_node lat_accel 4.0
```

What was learned on the practice track:

| Setting | Clean laps | Best lap | Notes |
|---|---|---|---|
| `v_max 4.5, lat_accel 4` | yes | 13.9 s | first clean configuration |
| `v_max 6, lat_accel 3.5` | yes | 13.3 s | after adding the turn-in rule and speed feedback |
| `v_max 8, lat_accel 4, turn_speed 1.8` | yes | 12.2 s | |
| `v_max 10, lat_accel 5, turn_speed 2.0` | **yes (default)** | **11.8 s** | straight is acceleration-limited at about 8 m/s |
| `v_max 10, lat_accel 6, turn_speed 2.0` | no | | clips the outer wall exiting the hairpins |
| `v_max 12, lat_accel 6, turn_speed 2.3` | no | | same |

The hairpins are the binding constraint: the car's effective turning radius at
1.5 to 2 m/s is about 0.8 m, which only just fits a lane-centre-to-lane-centre
U-turn of 1.2 m, so braking must be complete before the turn-in (the `turn_*`
parameters) and the exit speed is capped by `lat_accel`.

`scripts/ctl.sh` wraps the start/stop/reset cycle used while tuning
(`ctl.sh reset` publishes the bridge's reset topic so no clicking in the
simulator is needed), `scripts/monitor.py` prints lap count, lap times and the
collision counter once a second, and `scripts/recorder.py` logs scans, pose
and commands to an `.npz` for offline analysis.
