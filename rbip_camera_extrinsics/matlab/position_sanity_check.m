space_points = [
    csvread('image0_robot_keypoints.csv');
    csvread('image1_robot_keypoints.csv');
    csvread('image2_robot_keypoints.csv');
];
space_points = space_points(:, 2:3);
space_points = [space_points zeros(size(space_points, 1), 1)];

image_points = [
    csvread('image0_rect_keypoints.csv');
    csvread('image1_rect_keypoints.csv');
    csvread('image2_rect_keypoints.csv');
];
image_points = image_points(:, 2:3);

figure('Name', 'Input Data in Base Frame');
scatter3(space_points(:, 1), space_points(:, 2), space_points(:, 3));
hold on;
quiver3(0, 0, 0, 1, 0, 0, 'r');
quiver3(0, 0, 0, 0, 1, 0, 'g');
quiver3(0, 0, 0, 0, 0, 1, 'b');
axis equal;


%Quoted directly from calibration program output.
BASE_to_CAMERA = inv([
	0.055691	0.996213	0.066775	1.565591;
	0.997315	-0.058689	0.043804	0.245988;
	0.047557	0.064156	-0.996806	2.893154;
	0			0			0			1
]);

o_cam = BASE_to_CAMERA * [0; 0; 0; 1];
x_cam = (BASE_to_CAMERA * [1; 0; 0; 1]) - o_cam;
y_cam = (BASE_to_CAMERA * [0; 1; 0; 1]) - o_cam;
z_cam = (BASE_to_CAMERA * [0; 0; 1; 1]) - o_cam;
quiver3(o_cam(1), o_cam(2), o_cam(3), x_cam(1), x_cam(2), x_cam(3), 'r');
quiver3(o_cam(1), o_cam(2), o_cam(3), y_cam(1), y_cam(2), y_cam(3), 'g');
quiver3(o_cam(1), o_cam(2), o_cam(3), z_cam(1), z_cam(2), z_cam(3), 'b');
hold off;

space_points_adjusted = [space_points ones(size(space_points, 1), 1)]';
space_points_camera = BASE_to_CAMERA * space_points_adjusted;

figure('Name', 'Data Points in Camera Frame');
scatter3(space_points_camera(1, :), space_points_camera(2, :), space_points_camera(3, :));
hold on;
quiver3(0, 0, 0, 1, 0, 0, 'r');
quiver3(0, 0, 0, 0, 1, 0, 'g');
quiver3(0, 0, 0, 0, 0, 1, 'b');
axis equal;
hold off;

projected_pixels = nan(size(space_points, 1), 2);
%These are, again, quoted directly from the calibrator program.
fx = 1637.367343;
fy = 1638.139550;
cx = 2000;
cy = 1050;
for i = 1 : size(space_points, 1)
	x_plane = space_points_camera(1, i) / space_points_camera(3, i);
	y_plane = space_points_camera(2, i) / space_points_camera(3, i);
	
	projected_pixels(1, i) = cx + x_plane * fx;
	projected_pixels(2, i) = cy + y_plane * fy;
end

figure('Name', 'Corresponding Pixels');
scatter(projected_pixels(1, :), projected_pixels(2, :));
hold on;
scatter(image_points(:, 1), image_points(:, 2));
hold off;