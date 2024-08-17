[![Build and Test (humble)](https://github.com/MarqRazz/MoveIt.BT/actions/workflows/format.yaml/badge.svg)](https://github.com/MarqRazz/MoveIt.BT/actions/workflows/format.yaml.yaml)

# MoveIt.BT

!!UNDER CONSTRUCTION!!

This ROS 2 package is designed to make building, combining and executing [MoveIt](https://moveit.picknik.ai/main/index.html) based Behaviors easy and reusable.

An example launch file is included that starts the Behavior server and loads a list of plugins and BehaviorTrees from `yaml` file:
``` bash
ros2 launch action_server_bt_samples action_server_bt.launch.xml
```

As the server starts up it will print out the name of the Action followed by the plugins and BehaviorTrees have been loaded.
The specific ones important to this demo are:
```
[sample_bt_executor-9] [bt_action_server 1723905390.146525490]: Loaded ROS Plugin: libmtc_plugin.so
[sample_bt_executor-9] [bt_action_server 1723905390.148748207]: Loaded BehaviorTree: test_mtc.xml

```

To run a simple example motion:
``` bash
ros2 action send_goal /behavior_server btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: TestMtc}"
```

## Building from source

1. Clone this repository and its required dependencies (Once the upstream repositories have been updated to be compatible with this repository this step will be removed):
```bash
git clone https://github.com/MarqRazz/MoveIt.BT.git
vcs import < MoveIt.BT/moveit_bt.repos
```

2. Install dependencies with rosdep:
```bash
rosdep install --from-paths . --ignore-src --rosdistro "$ROS_DISTRO" -y
```

3. Build the colcon workspace with:
```bash
colcon build
```
