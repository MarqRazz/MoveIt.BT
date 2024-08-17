# MoveIt.BT

Under Construction
:construction:

# Starting the Robot

To start the Panda Robot with MoveIt, Rviz and a Behavior server with MTC behavior plugins loaded:
```bash
ros2 launch mtc_samples sample_mtc_behaviors.launch.xml
```

To call the basic example BT.xml.
```bash
ros2 action send_goal /behavior_server btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: TestMtc}"
```

## Working with Groot2

```
sudo apt install fuse libfuse2
```
install Groot 2

or just run the appimg
