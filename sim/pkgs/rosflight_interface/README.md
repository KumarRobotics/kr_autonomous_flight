# rosflight_interface

This node translates `kr_mav_msgs/SO3Command` to `rosflight_msgs/Command`.

#### `rosflight_msgs` requirement

This node requires `rosflight_msgs` to be present when building. If `rosflight_msgs` is not found when building the first time, a warning is given and nothing in this package is built. If `rosflight_msgs` is installed after this, force a recheck by adding `--force-cmake` to the `catkin_make`/`catkin build` command.
