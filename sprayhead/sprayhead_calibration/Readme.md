## Sprayhead Calibration Routine - Horizontal Lines

Detects six (always six) horizontal yellow rectangles, and the heights of their top and bottom borders. Outputs how much the detections have differed from guesses. Uses a ros service.

# To Run

`rosrun sprayhead_calibration sprayhead_hor_cal_srv`

Send a service call of type `sprayhead_calibration/sprayhead_calibrate` to `/analyze_sprayhead_image_horizontal`. An example is included below:

~
rosservice call /analyze_sprayhead_image_horizontal "image_file: '/home/tes77/image_trial_A.png'
guesses: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" 
~

## Sprayhead Calibration Routine - Vertical Lines

Detects six (always six) horizontal yellow rectangles, and the distances of their ends. Outputs how much the detections have differed from guesses. Uses a ros service.

# To Run

`rosrun sprayhead_calibration sprayhead_ver_cal_srv`

Send a service call of type `sprayhead_calibration/sprayhead_calibrate` to `/analyze_sprayhead_image_vertical`. They are listed bottom-to-top, left-toright. An example is included below:

~
rosservice call /analyze_sprayhead_image_vertial "image_file: '/home/tes77/image_trial_A.png'
guesses: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" 
~
