# tf_relay

Package to prefix robot namespace to all transform frames. It can update the tf for all the robots spawned. 

## Run

To run the package individually:

```bash
ros2 run tf_relay tf_relay_main --ros-args -p robot_namespaces:="[robot_1, robot_2]"
```

To launch the package, there are 2 options:

- 1. Use the saved json file from the extension
    ```bash
    ros2 launch tf_relay tf_relay.launch.py mode:=json
    ```

- 2. Manually give the namesapces
    ```bash
    ros2 launch tf_relay tf_relay.launch.py mode:=param robot_namespaces:="["robot_1", "robot_2"]"
    ```

*Replace the robot_1 and robot_2 with the spawned robot namespaces*

## Topics

| Namespace Prefix | Input Topics          | Output Topics             | Message Type              |
|------------------|-----------------------|---------------------------|---------------------------|
| `<namespace>`    | `<namespace>/tf`      | `<namespace>/relayed/tf`      | `tf2_msgs/msg/TFMessage` |
| `<namespace>`    | `<namespace>/tf_static` | `<namespace>/relayed/tf_static` | `tf2_msgs/msg/TFMessage` |
