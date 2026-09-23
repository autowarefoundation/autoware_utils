# autoware_utils_launch

This package provides following features for Autoware as an extension to [launch](https://github.com/ros2/launch) and [launch_ros](https://github.com/ros2/launch_ros).

- Actions
  - GlobalParameters
- Substitutions
  - CurrentRosNamespace

## GlobalParameters

The autoware_global_parameters action loads the ROS parameter file and sets it to the global parameters.
Specify a parameter file path or a path list for the path attribute.

### Sample launch

```xml
<launch>
    <autoware_global_parameters paths="foo.param.yaml"/>
    <autoware_global_parameters paths="[bar.param.yaml, baz.param.yaml]"/>
</launch>
```

## CurrentRosNamespace

The current-ros-namespace substitution returns the current ROS namespace set by the push-ros-namespace action.
This is useful when you need to specify the full name of a topic or node name.

### Sample launch

```xml
<launch>
    <push-ros-namespace namespace="foo"/>
    <log message="$(current-ros-namespace)/test"/>
    <push-ros-namespace namespace="bar"/>
    <log message="$(current-ros-namespace)/test"/>
</launch>
```

### Sample output

```bash
[INFO] [launch.user]: /foo/test
[INFO] [launch.user]: /foo/bar/test
```
