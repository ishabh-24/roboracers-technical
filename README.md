# Penn RoboRacer Technical Challenge: LiDAR Wall Follower

## Contents
This repo contains my solution to the Penn RoboRacer technical challenge: a ROS 2 node that drives the AutoDRIVE RoboRacer around the practice track using only its LiDAR. The autodrive_devkit/ package is the AutoDRIVE starter code (bridge, keyboard teleop, launch files, BSD-licensed by AutoDRIVE) and the new part is autodrive_roboracer/wall_follow_node.py. The node reads the 1080-beam scan, measures the distance and angle to the left and right walls with a two-beam estimate, and steers with a PID loop that keeps the car centred between them. As the free distance ahead shrinks it blends in a follow-the-gap term, so it turns into the hairpins instead of running at the wall. Throttle comes from a target speed that is the minimum of what the car can brake to before the wall ahead, what it can take through the upcoming turn, and a top-speed cap, tracked using the car’s odometry speed. If the car ever gets wedged against a wall it backs up briefly and continues. With the default settings it runs clean laps in about 11.8 s.

## Results

| | |
|---|---|
| **Video (unlisted)** | [Youtube Link](https://youtu.be/JCiQsIOpolM) |
| **Fastest clean lap** | 11.8 s (HUD best lap in the video; laps 2 to 4 of the recording are 11.8, 11.8 and 12.0 s with the collision counter at 0) |
