# multi-floor-lio-sam v0.5 Update

## Version Progress

  -----------------------------------------------------------------------
  Version                 Status                  Description
  ----------------------- ----------------------- -----------------------
  v0.5                    Completed               Nav2 terminal pose
                                                  optimization, DWB
                                                  parameter tuning,
                                                  repeated navigation
                                                  experiments

  v0.6                    Next                    Multi-floor map
                                                  management, elevator
                                                  topology and floor
                                                  switching

  Future                  Planned                 Multi-robot shared map,
                                                  task allocation and
                                                  coordinated navigation
  -----------------------------------------------------------------------

## Current System Status

Completed pipeline:

LIO-SAM 3D mapping -\> GlobalMap saving -\> NDT localization -\> TF
transformation -\> Nav2 navigation -\> terminal accuracy evaluation

Completed: - Gazebo simulation - 3D LiDAR and IMU simulation - LIO-SAM
mapping - NDT localization - TF chain - Nav2 navigation - DWB controller
optimization - Terminal pose evaluation

## Nav2 Terminal Pose Optimization

The initial navigation test achieved goal reaching, but terminal yaw
error was around 14 degrees.

Optimization:

DWB:

``` yaml
RotateToGoal.scale: 50.0
```

Goal checker:

Before:

``` yaml
yaw_goal_tolerance: 0.25
```

After:

``` yaml
yaw_goal_tolerance: 0.05
```

The terminal yaw error was reduced to approximately 3 degrees.

## Repeated Navigation Experiments

Five repeated experiments were performed.

Average results:

  Metric                   Result
  ------------------------ -------------------
  Navigation time          about 42 s
  Initial planned length   5.768 m
  Actual path length       about 6.35 m
  Path ratio               about 1.10
  Average velocity         about 0.15 m/s
  Position error           about 20 cm
  Yaw error                about 2.8 degrees

The results verify stable single-floor navigation and provide the basis
for future elevator docking and multi-floor experiments.

## Next Steps

1.  Multi-floor map management.
2.  Elevator topology and floor switching state machine.
3.  Multi-robot shared map navigation.
4.  Task allocation and coordinated path planning.

Current milestone: v0.5 Single-floor LIO-SAM + NDT localization + Nav2
optimization completed.
