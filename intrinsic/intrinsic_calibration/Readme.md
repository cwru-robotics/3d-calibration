###Intrinsic Calibration Final Step

This program calculates the final camera intrinsics based on collected data.

Run it by calling `roslaunch intrinsic_calibration calibrate.launch`.

It expects to find a set of calibration points with the format \[pixel location of spot (2 values)\] \[position of target in meters (3 values)\] \[position of spot on target in meters (2 values)\]. It also expects a default initialization file for the target positions and camera intrinsics.

These types of files are generated automatically by `intrinsic_acquisiton` and `dotboard_detection`, and are stored as `intrinsic_acquisiton/data/intrinsic_detections.csv` and `intrinsic_acquisiton/data/task_description.yml` respectively.
