###Intrinsic Calibration Detector And Post-Processor

Takes raw image and positional data and converts it into numerical forms the intrinsic calibrator can use (dropped as a csv file).

Run with `roslaunch dotboard_detection dots_detect.launch`.

Images are named in the following format: `img\_**x**\_**y**\_**z**.png`. **x**, **y**, and **z** are the coordinates of the target in the mill frame, given in meters with decimal points replaced hy the character `p`: for instance, an image describing the target at mill displacement 0.1m, 2.2m, 3.3m would have the file name `img\_0p1\_2p2\_3p3.png`.

These images are generated automatically by `intrinsic_acquire`.
