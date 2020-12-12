# ros2_sorting

## Autonomously sort objects with a Clearpath Robotics Jackal in ROS2

- [x] Create Gazebo world for testing
  - Why is the lighting different every time Gazebo is launched?
- [x] Jackal sdf for Gazebo

  - Was struggling to chain links and joints together in an understandable way.  Joints to links other than `chassis_link` always seem to attach to the center of the chassis.  This is because poses of joints and links are specified differently in URDF and SDF.  This was fixed by using the `relative_to` attribute in poses.  This will be needed whenever the position is specified in the `<joint>` element.
  - I had to move all of the meshes into ~/.gazebo/models/jackal/meshes so that Gazebo could find them
  -  This task is mainly "translating" clearpath's Jackal urdf to sdf.  Frustratingly, the two formats are extremely close but slightly different
  -  Ended up ditching the ZED camera and adding a simple camera on top of the laser.  This camera will eventually be on a revolute joint to pan around.
  -  Wheels seem to be detached, but this was fixed once the drive plugin was added.
  -  Launch with `ros2 launch jackal_gazebo jackal_gazebo.launch.py`
 
  - [x] Add plugins for ROS2 connection to SDF

    - [x] Wheels + jackal_teleop

      - run with: `ros2 run jackal_teleop jackal_teleop`

    - [x] LIDAR
    - [x] Camera
    - [x] IMU

- [x] Jackal urdf for Rviz, jackal_description
  - Could urdf be expanded with `<gazebo>` tags to replace the sdf?
- [x] Navigation stack
- [ ] Object ID
  - [x] Method to view camera feed
    - Rviz works, but it required two changes. 
      1. The QOS of the camera topic has to be set manually in Rviz to match the QOS of the published topic.  See [this](https://answers.ros.org/question/359390/ros2-rviz2-and-rqt-do-not-shown-published-image/) answer.
      2. It requires the navigation stack in order to have a transform from the `/map` frame to the frame of the camera.  This seems silly.  The image is just pixel values, which I should be able to view without any transform information.
  - [ ] Try other QOS settings to try to alleviate stuttering
    - Reading: [1](https://design.ros2.org/articles/qos.html), [2](https://design.ros2.org/articles/qos_deadline_liveliness_lifespan.html)
  - [ ] Method to process camera feed
    - Need ability to manipulate pixels and feed pixel data into CV model
    - image_pipeline should provide this
  - [ ] CV Model training / testing
- [ ] Arm
  - [ ] arm added to sdf and/or urdf
  - [ ] arm remote control
  - [ ] arm autonomous control
- [x] SMACH w/ ROS2
  - I think this is unnecessarily complicated (see sorting package item).  There is no ros2 smach_viewer yet, which was (in my opinion) the best part of smach anyway.  Userdata remapping seems redundant since ROS can already do message passing.  Also, ROS2 lifecycle nodes seem very similar to smach states (though they aren't in rclpy yet).  Could also try behavior trees, which are implemented as a part of navigation2.
  - Also looked into `plansys2`, but it requires learning a new language (PDDL) and doesn't seem to have any way to visualize the plan.
  - Using a [fork](https://github.com/lesire/executive_smach/tree/ros2-eloquent) of the `executive_smach` package for ROS2 (eloquent).  smach_viewer has not been ported yet.
  - ROS2 smach uses `RosState` from the `smach_ros` package as the standard state, so that a node can be passed in the constructor
- [ ] sorting package
  - Alternative to smach where states are just regular nodes and transitions are replaced with topics, services and actions
  - smach w/ ROS2 was extremely difficult partially due to lack of visualization, but also due to difficulty with getting multiple callbacks running in parallel.
  - Will this allow visualization with rqt? Maybe
    - [ ] Base implementation
      - This includes minimal implementations of all the nodes, topics, services and actions.
    - [x] Study nav2_bringup launch files and write a custom one
      - Should only launch desired nodes, but enough for a complete navigation stack
    - [x] Replace FollowWaypoints action with navigate_to_pose
      - Requires a behavior tree, which can be found in `/opt/ros/foxy/share/nav2_bt_navigator/behavior_trees/`
    - [ ] Use a parameter to load the behavior tree.
      - Reading: [1](https://design.ros2.org/articles/ros_parameters.html), [2](https://index.ros.org/doc/ros2/Tutorials/Using-Parameters-In-A-Class-Python/)
    - [x] Make all clients wait for servers in their node's `__init__` method and replace all `wait_for_service` and `wait_for_server` with `service_is_ready` and `server_is_ready` checks.
    - [ ] Simulate "enemies" which have a required standoff distance
      - nav2 has "Keepout Zones" which may be useful for this
    - [ ] Learn more about `MultithreadedExecutor` and `ReentrantCallbackGroup`
      - When are they required?  What happens when they are needed but not used?  What specific behaviors do they enable?
    - [ ] Find better way to manage waiting for actions to complete
      - Using `future.set_result()` in various callbacks is confusing.
      - Actions still need to be interruptable
    - [ ] Save map and reload
    - [ ] Add check if nav goal is off the costmap add intermediate waypoints
- [x] Use `/spawn_entity` service to spawn jackal in gazebo
- [x] Investigate usage of docker with ROS1/2.  Will this help with collaboration?
  - Answer: potentially.  This will ensure everyone is running their workspace on top of identical installs.  However, it also requires additional upfront learning from new team members.
  - Gazebo took some more work to figure out, but must be possible (e.g. how to get display).  This was solved with running a gzserver in the container, and a gzclient on the host.  Need to use `docker run` with the `--net host` option so that the server communicates directly with ports on the host
  - How to get rviz/rqt to run in the container but interact via the host?  Need to install libgl1-mesa on the image and add some arguments to docker run
- [ ] Create system diagram

## Misc
  - Action goal statuses, from `/opt/ros/foxy/include/action_msgs/msg/detail/goal_status__struct.h`:
    | id  | status    |
    | --- | --------- |
    | 0   | unknown   |
    | 1   | accepted  |
    | 2   | executing |
    | 3   | canceling |
    | 4   | succeeded |
    | 5   | canceled  |
    | 6   | aborted   |
    
   Why are these not in the ros documentation? | unknown

## Current run process
from `ros2_sorting/`

term 1:
```
$ bash docker/run.bash
$ ... (local workspace will be built)
$ ros2 launch jackal_gazebo jackal_gazebo.launch.py
```

term 2:
```
$ gzclient --verbose
```

term 3:
```
$ docker exec -it ros2_sorting bash
$ ros2 launch jackal_navigation2 sorting_nav2.launch.py
```

term 4:
```
$ docker exec -it ros2_sorting bash
$ ros2 launch sorting sort_demo.launch.py
```


If needed (sometimes if you change dependencies), build fresh docker image with:
```
$ bash docker/build.bash foxy .
```

If you find yourself rebuilding the full docker image more than once or twice, run the following command occasionally to clear out old images.
```
$ docker system prune -f
```

You can rebuild the workspace by restarting the container, or with :
```
~/ros_ws$ colcon build --symlink-install
~/ros_ws$ source install/setup.bash
```