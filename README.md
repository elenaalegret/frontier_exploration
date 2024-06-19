# frontier_exploration

Thread safe frontier exploration package based on fast and efficient Wavefront Frontier Detection.

## Resolving dependencies

Use `rosdep` to resolve ROS dependencies


## Integration Changes

Modifications have been made to integrate two new subscribers for the `explore_and_find` package. 
These changes allow the `frontier_exploration` package to work seamlessly with the `explore_and_find` node, enhancing functionality for external control and object detection. For more details, visit our [explore_and_find repository](https://github.com/elenaalegret/explore_and_find).


## References

```
@misc{topiwala2018frontier,
      title={Frontier Based Exploration for Autonomous Robot}, 
      author={Anirudh Topiwala and Pranav Inani and Abhishek Kathpal},
      year={2018},
      eprint={1806.03581},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

## Demo using Turtlebot3

- **Shell #1** : Gazebo

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

- **Shell #2** : SLAM + RViz

```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

- **Shell #3** : move_base

```bash
roslaunch turtlebot3_navigation move_base.launch 
```

- **Shell #4** : Frontier exploration

```bash
roslaunch frontier_exploration explore_costmap.launch
```
