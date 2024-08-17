[![Build and Test (humble)](https://github.com/MarqRazz/ActionServer/actions/workflows/build_and_test.yaml/badge.svg)](https://github.com/MarqRazz/ActionServer/actions/workflows/build_and_test.yaml)

# ActionServer.BT

This ROS 2 package is designed to make building, combining and executing [BehaviorTree.CPP](https://www.behaviortree.dev/docs/intro) based Behaviors easy and reusable.
It includes an Action Server that is able to register plugins and Trees/Subtrees so that a user can execute any known BehaviorTree by simply sending the name of it to the server.

An example launch file is included that starts the server and loads a list of plugins and BehaviorTrees from `yaml` file:
``` bash
ros2 launch action_server_bt_samples action_server_bt.launch.xml
```

As the server starts up it will print out the name of the Action followed by the plugins and BehaviorTrees have been loaded.
```
[action_server_bt]: Starting Action Server: behavior_server
[action_server_bt]: Loaded Plugin: libdummy_nodes_dyn.so
[action_server_bt]: Loaded Plugin: libmovebase_node_dyn.so
[action_server_bt]: Loaded Plugin: libcrossdoor_nodes_dyn.so
[action_server_bt]: Loaded ROS Plugin: libsleep_plugin.so
[action_server_bt]: Loaded BehaviorTree: door_closed.xml
[action_server_bt]: Loaded BehaviorTree: cross_door.xml
```

To call ActionServer.BT from the command line:
``` bash
ros2 action send_goal /behavior_server action_server_bt_msgs/action/ActionTree "{target_tree: CrossDoor}"
```
> *NOTE:* the name of the Action Server is customizable and is renamed in `action_server_bt_sample.yaml`

You can also try a Behavior that is a ROS Action or Service client itself.
Please see the [BehaviorTree.CPP](https://www.behaviortree.dev/docs/ros2_integration) documentation on building Behaviors based on ROS interfaces.
```bash
ros2 action send_goal /behavior_server action_server_bt_msgs/action/ActionTree "{target_tree: SleepActionSample}"
```

## Building from source

1. Clone this repository and its required dependencies (Once the upstream repositories have been updated to be compatible with this repository this step will be removed):
```bash
git clone https://github.com/MarqRazz/ActionServer.BT.git
vcs import < ActionServer.BT/action_server_bt.repos
```

2. Install dependencies with rosdep:
```bash
rosdep install --from-paths . --ignore-src --rosdistro "$ROS_DISTRO" -y
```

3. Build the colcon workspace with:
```bash
colcon build
```
