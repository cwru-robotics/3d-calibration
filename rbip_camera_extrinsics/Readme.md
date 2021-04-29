### RBIP Camera Position Calibration File

## Description
Matches a collection of pixel-space checkerboard points visualized from a single camera position to a corresponding collection of Cartesian points in the RBIP frame. The position of the camera with respect to the RBIP frame is then returned.
This system assumes the camera will be positioned in one place throughout the entire collection process; move the target grid and take multiple snapshots to collect more than one grid's worth of data.

The system expects an arbitrary number of data files in $ROS_WS/calibration_temp_files/, all of which will have the form `image#_robot_keypoints.csv`. Each should be paired with a file called `image#_rect_keypoints.csv`.
The rect files should include _all consecutive checkerboard points_ detected photographically in a given snapshot. The format is `R, U, V` where R is a squential field that increases from 0, U is the horizontal position of the detection and V the vertical.
The robot files should include _some_ points identified in cartesian space using the robot. The format is `N, X, Y`, where N is the position of the point on the grid, X is its x-coordinate, and Y the y-coordinate. The z-coordinate is assumed to be zero.

The `roadprintz_camera_calibration/calib_coordinator_w_visual_servoing` package produces data in a format this package can read.

The system will then output goodness-of-fit data and a launch file relating the camera position (at the time the snapshots were taken) to the RBIP frame. It is called `roadprintz_launch/launch/stella_camera_optical_frame.launch`. A date-stamped backup file will also be created.

To run, `roslaunch rbip_camera_extrinsics/calibrate.launch `.
