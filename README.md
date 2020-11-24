# roadprintz_cam_cal_tes

`sudo apt-get install libceres1`

Extrinsic calibration is accomplished through the extrinsic_toplevel package.

`rosrun extrinsic_toplevel extrinsic_publish_service` publishes a static transform between `/forearm` and `/cam_calibrated`. This should be run whenever you are using the camera. It also offers the `/update_extrinsic` service which is called automatically by the actual calibration utility and just tells the system that a YAML file has changed to include new calibration data.

`rosrun extrinsic_toplevel extrinsic_exec_service` does nothing automatically other than offering a `/calibrate_extrinsic` service. This is called with no arguments, and runs the entire calibration procedure including image acquisition, target detection, and optimization. The results are written to a file and also returned as the service result. The user can then decide based on that information whether to call `/update_extrinsic`.
