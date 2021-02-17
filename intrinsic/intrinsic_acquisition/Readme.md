###Acquisition system for calibration data.

Generates data for intrinsic calibration.

Run with `roslaunch intrinsic_acquisition intrinsic_acquisition.launch`.

Requires a task description to start, an example of which is included in `intrinsic_acquisition/data/task_description.yml`. This requires a mapping of camera axes to mill axes. For help in determining this, `roslaunch intrinsic_acquisition intrinsic_axes_helper.launch`.

Produces files in the format used by `dotboard_detection`.

Interfaces with a "mill" in two ways. The first is manually- it will prompt the human to move it, and then press enter.
The second is through ROS. The utility looks for a service called `motion_command` which is of type [tutrlesim/Spawn](http://docs.ros.org/en/melodic/api/turtlesim/html/srv/Spawn.html).
Something that moves the "mill" automatically should offer this service, move to the given x, y, "theta"=z coordinates when the service is called, and then return an empty string. This will cause the program to bypass the manual moving system and photograph the target at this position.
