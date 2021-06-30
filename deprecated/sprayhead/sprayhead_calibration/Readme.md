## Sprayhead Calibration Routine - Horizontal Lines

Detects eight (always eight) horizontal painted rectangles, and the heights of their top and bottom borders. Outputs how much the detections have differed from guesses. Uses a ros service.

The file provided as an argument contains the centers, in pixel coordinates, of each of the rectangles. There should be eight lines, with two entries each (the (u, v) of each rectangle.

# To Run

`roslaunch sprayhead_calibration offer_services.launch`

Send a service call of type `sprayhead_calibration/sprayhead_calibrate` to `/analyze_sprayhead_image_horizontal`. An example is included below:

~
rosservice call /analyze_sprayhead_image_horizontal "image_file: '/home/tes77/image_trial_A.png'
guesses: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" 
~

## Sprayhead Calibration Routine - Vertical Lines

Detects eight (always eight) vertical painted rectangles, and the distances of their ends. Outputs how much the detections have differed from guesses. Uses a ros service.

The file provided as an argument contains the centers, in pixel coordinates, of each of the rectangles. There should be eight lines, with two entries each (the (u, v) of each rectangle.

# To Run

`roslaunch sprayhead_calibration offer_services.launch`

Send a service call of type `sprayhead_calibration/sprayhead_calibrate` to `/analyze_sprayhead_image_vertical`. They are listed bottom-to-top, left-toright. An example is included below:

~
rosservice call /analyze_sprayhead_image_vertial "image_file: '/home/tes77/image_trial_A.png'
guesses: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" 
~
