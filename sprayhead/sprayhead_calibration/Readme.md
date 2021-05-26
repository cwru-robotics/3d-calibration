## Sprayhead Calibration Routine - Horizontal Lines

Detects twelve (always twelve) horizontal yellow lines, and outputs how much the detections have differed from guesses. Usesa ros service.

# To Run

`rosrun sprayhead_calibration sprayhead_cal_srv`

Send a service call of type `sprayhead_calibration/sprayhead_calibrate`. An example is included below:

~
rosservice call /analyze_sprayhead_image "image_file: '/home/tes77/image_trial_A.png'
guesses: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" 
~
