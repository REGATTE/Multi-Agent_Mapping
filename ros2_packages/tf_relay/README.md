# tf_relay

Package to prefix robot namespace to all transform frames. It can update the tf for all the robots spawned. 

## Run

To run the package,

```bash
ros2 run tf_relay tf_relay_main --ros-args -p robot_namespaces:="[robot_1, robot_2]"
```

*Replace the robot_1 and robot_2 with the spawned robot namespaces*

## Topics

| Namespace Prefix | Input Topics          | Output Topics             | Message Type              |
|------------------|-----------------------|---------------------------|---------------------------|
| `<namespace>`    | `<namespace>/tf`      | `<namespace>/relayed/tf`      | `tf2_msgs/msg/TFMessage` |
| `<namespace>`    | `<namespace>/tf_static` | `<namespace>/relayed/tf_static` | `tf2_msgs/msg/TFMessage` |
