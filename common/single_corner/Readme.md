To test library, `roslaunch signle_corner test_single_corner.launch`. This will run a test main.


Data goes in the `/data` folder. ATM, the test main looks for a single photo called data/checkercross.png.


~~~
cv::Point2d detect_single_corner(const cv::Mat& img);	
~~~
Reads in an image and identifies the pixel location of a corner.
