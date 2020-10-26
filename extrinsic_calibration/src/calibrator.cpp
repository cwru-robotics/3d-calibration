#include <fstream>
#include <iostream>

#include <Eigen/Eigen>

#include "yaml-cpp/yaml.h"

#include <cc_utils/cc_utils.h>

//Distance from center to one edge of marker
//Markers are 6in across so 3in -> meters
//TODO Eventually this will need to be parametrized
#define ARUCO_MARKER_SIZE 0.0762


class CalibrationEntry{
public:
	//Giant stupid constructor chain.
	//The fact that it takes two functions to pass the exact same arguments into the exact same variables is Ceres' fault; not mine.
	CalibrationEntry(
		const double image_pixels_in[2],
		
		const double TARGET_to_POINT_translation_in[2],
		const bool   target_is_top_in,
		
		const double FOREARM_to_BASE_translation_in[3],
		const double FOREARM_to_BASE_rotation_in[9],
		
		const double f_x_in, const double f_y_in, const double c_x_in, const double c_y_in
	){
		image_pixels[0] = image_pixels_in[0];
		image_pixels[1] = image_pixels_in[1];
		
		TARGET_to_POINT_translation[0] = TARGET_to_POINT_translation_in[0];
		TARGET_to_POINT_translation[1] = TARGET_to_POINT_translation_in[1];
		target_is_top = target_is_top_in;
		
		FOREARM_to_BASE_translation[0] = FOREARM_to_BASE_translation_in[0];
		FOREARM_to_BASE_translation[1] = FOREARM_to_BASE_translation_in[1];
		FOREARM_to_BASE_translation[2] = FOREARM_to_BASE_translation_in[2];
		for(int i = 0; i < 9; i++){
			FOREARM_to_BASE_rotation[i] = FOREARM_to_BASE_rotation_in[i];
		}
		
		f_x = f_x_in;
		f_y = f_y_in;
		c_x = c_x_in;
		c_y = c_y_in;
	}
	static ceres::CostFunction* Create(
		const double image_pixels_in[2],
		
		const double TARGET_to_POINT_translation_in[2],
		const bool   target_is_top_in,
		
		const double FOREARM_to_BASE_translation_in[3],
		const double FOREARM_to_BASE_rotation_in[9],
		
		const double f_x_in, const double f_y_in, const double c_x_in, const double c_y_in
	){
		return new ceres::AutoDiffCostFunction<CalibrationEntry, 2,//Residual output comes first.
		//	Top translation		Top rotation	Bottom translation	bottom rotation		camera translation	camera rotation
			3,			3,		3,			3,			3,			3
		>(new CalibrationEntry (//Just pass all the arguments in in the same order.
			image_pixels_in,
		
			TARGET_to_POINT_translation_in,
			target_is_top_in,
		
			FOREARM_to_BASE_translation_in,
			FOREARM_to_BASE_rotation_in,
		
			f_x_in, f_y_in, c_x_in, c_y_in
		));
	}


	//Constant member vars
	//Perpoints
	double image_pixels[2];
	double TARGET_to_POINT_translation[2];
	bool   target_is_top;
	double FOREARM_to_BASE_translation[3];
	double FOREARM_to_BASE_rotation[9];
	//Globals (which need to be passed perpoint anyway)
	double f_x, f_y, c_x, c_y;
	
	
	template<typename T> bool operator()(//TODO Why are all these const / should all these be const?
		const T* BASE_to_TOP_translation, const T* BASE_to_TOP_rotation,
		const T* BASE_to_BOT_translation, const T* BASE_to_BOT_rotation,
		
		const T* CAM_to_FOREARM_translation, const T* CAM_to_FOREARM_rotation,
		
		T* residual
	) const {
		/*std::cout << "INITIAL BTT T:\n";
		std::cout << BASE_to_TOP_translation[0] << "\n";
		std::cout << BASE_to_TOP_translation[1] << "\n";
		std::cout << BASE_to_TOP_translation[2] << "\n\n";
		std::cout << "INITIAL BTT R:\n";
		std::cout << BASE_to_TOP_rotation[0] << "\n";
		std::cout << BASE_to_TOP_rotation[1] << "\n";
		std::cout << BASE_to_TOP_rotation[2] << "\n\n";
		std::cout << "INITIAL BTB T:\n";
		std::cout << BASE_to_BOT_translation[0] << "\n";
		std::cout << BASE_to_BOT_translation[1] << "\n";
		std::cout << BASE_to_BOT_translation[2] << "\n\n";
		std::cout << "INITIAL BTB R:\n";
		std::cout << BASE_to_BOT_rotation[0] << "\n";
		std::cout << BASE_to_BOT_rotation[1] << "\n";
		std::cout << BASE_to_BOT_rotation[2] << "\n\n";
		std::cout << "INITIAL CAM T:\n";
		std::cout << CAM_to_FOREARM_translation[0] << "\n";
		std::cout << CAM_to_FOREARM_translation[1] << "\n";
		std::cout << CAM_to_FOREARM_translation[2] << "\n\n";
		std::cout << "INITIAL CAM R:\n";
		std::cout << CAM_to_FOREARM_rotation[0] << "\n";
		std::cout << CAM_to_FOREARM_rotation[1] << "\n";
		std::cout << CAM_to_FOREARM_rotation[2] << "\n\n";*/
	
	
		//CAM_to_POINT = CAM_to_FOREARM * FOREARM_to_BASE * BASE_to_TARGET * TARGET_to_POINT
		
		
		//0: TARGET_to_POINT
		T TARGET_to_POINT [3] = {
			T(TARGET_to_POINT_translation[0]),
			T(TARGET_to_POINT_translation[1]),
			T(0.0)
		};
		
		/*std::cout << "TARGET TO POINT:\n";
		std::cout << TARGET_to_POINT[0] << "\n";
		std::cout << TARGET_to_POINT[1] << "\n";
		std::cout << TARGET_to_POINT[2] << "\n\n";*/
		
		
		//1: BASE_to_POINT = BASE_to_TARGET * TARGET_to_POINT
		T BASE_to_TARGET_t [3];
		T BASE_to_TARGET_r [3];
		for(int i = 0; i < 3; i++){
			if(target_is_top){
				BASE_to_TARGET_t[i] = BASE_to_TOP_translation[i];
				BASE_to_TARGET_r[i] = BASE_to_TOP_rotation[i];
			} else{
				BASE_to_TARGET_t[i] = BASE_to_BOT_translation[i];
				BASE_to_TARGET_r[i] = BASE_to_BOT_rotation[i];
			}
		}
		T BASE_to_POINT [3];
		cc_utils::transformPoint_euler(BASE_to_TARGET_t, BASE_to_TARGET_r, TARGET_to_POINT, BASE_to_POINT);
		
		/*std::cout << "BASE TO POINT:\n";
		std::cout << BASE_to_POINT[0] << "\n";
		std::cout << BASE_to_POINT[1] << "\n";
		std::cout << BASE_to_POINT[2] << "\n\n";*/
		
		
		//2: FOREARM_to_POINT = FOREARM_to_BASE * BASE_to_POINT
		T FOREARM_to_BASE_t [3] = {
			T(FOREARM_to_BASE_translation[0]),
			T(FOREARM_to_BASE_translation[1]),
			T(FOREARM_to_BASE_translation[2])
		};
		T FOREARM_to_BASE_r [9] = {
			T(FOREARM_to_BASE_rotation[0]),
			T(FOREARM_to_BASE_rotation[1]),
			T(FOREARM_to_BASE_rotation[2]),
			T(FOREARM_to_BASE_rotation[3]),
			T(FOREARM_to_BASE_rotation[4]),
			T(FOREARM_to_BASE_rotation[5]),
			T(FOREARM_to_BASE_rotation[6]),
			T(FOREARM_to_BASE_rotation[7]),
			T(FOREARM_to_BASE_rotation[8])
		};
		
		T FOREARM_to_POINT [3];
		cc_utils::transformPoint_rm(FOREARM_to_BASE_t, FOREARM_to_BASE_r, BASE_to_POINT, FOREARM_to_POINT);
		
		/*std::cout << "FOREARM TO POINT:\n";
		std::cout << FOREARM_to_POINT[0] << "\n";
		std::cout << FOREARM_to_POINT[1] << "\n";
		std::cout << FOREARM_to_POINT[2] << "\n\n";*/
		
		
		//3: CAM_to_POINT = CAM_to_FOREARM * FOREARM_to_POINT
		T CAM_to_FOREARM_t [3] = {
			T(CAM_to_FOREARM_translation[0]),
			T(CAM_to_FOREARM_translation[1]),
			T(CAM_to_FOREARM_translation[2])
		};
		T CAM_to_FOREARM_r [3] = {
			T(CAM_to_FOREARM_rotation[0]),
			T(CAM_to_FOREARM_rotation[1]),
			T(CAM_to_FOREARM_rotation[2])
		};
		
		T CAM_to_POINT [3];
		cc_utils::transformPoint_euler(CAM_to_FOREARM_t, CAM_to_FOREARM_r, FOREARM_to_POINT, CAM_to_POINT);
		
		/*std::cout << "CAM TO POINT:\n";
		std::cout << CAM_to_POINT[0] << "\n";
		std::cout << CAM_to_POINT[1] << "\n";
		std::cout << CAM_to_POINT[2] << "\n";*/
		
		T u, v;
		cc_utils::project(
			CAM_to_POINT[0], CAM_to_POINT[1], CAM_to_POINT[2],
			
			T(f_x), T(f_y), T(c_x), T(c_y),
			T(0), T(0), T(0), T(0), T(0),//No distortion in rectified image.
		
			T(0), T(0),//TODO Get rid of these.
		
			u, v
		);
		
		//printf("Real u, v: (%f, %f)\n", image_pixels[0], image_pixels[1]);
		
		cc_utils::add_to_visualization(cc_utils::val(u), cc_utils::val(v), image_pixels[0], image_pixels[1]);
		
		//std::getchar();
		
		residual[0] = u - T(image_pixels[0]);
		residual[1] = v - T(image_pixels[1]);
		
		return true;
	}
};


int main(int argc, char** argv) {
	if(argc < 4){
		printf("\e[33mUsage: rosrun extrinsic_calibration calibrator /path/to/data.csv /path/to/initial/guesses.yml /path/to/camera/intrinsics.yml [path/to/output.launch]\e[39m\n");
		return 0;
	}
	
	char * output;
	if(argc < 5){
		printf("\nNo designated output file given, calibrated data will be displayed and then discarded.\n");
		output = argv[3];
	} else {
		output = NULL;
	}
	
	
	//Read in the camera intrinsics
	YAML::Node intrinsic_file;
	try{
		intrinsic_file = YAML::LoadFile(argv[3]);
	} catch(YAML::BadFile e){//If file is not extant and well-formed...
		printf("\e[39mCamera intrinsics file \"%s\" does not exist or contains syntax errors.\e[31m\n", argv[3]);
		return 0;
	}
	double fx, fy, cx, cy;
	//Rectified images so no need for distortion.
	try{
		fx = intrinsic_file["fx"].as<double>();
		fy = intrinsic_file["fy"].as<double>();
		cx = intrinsic_file["cx"].as<double>();
		cy = intrinsic_file["cy"].as<double>();
	} catch(YAML::RepresentationException e){
		printf("\e[39mIntrinsic parse exception \"%s\".\e[31m\n", e.what());
		return 0;
	}
	printf("\nSuccessfully initialized intrinsics from %s:\n", argv[3]);
	printf("\tfx:%f\n\tfy:%f\n\tcx:%f\n\tcy:%f\n\n", fx, fy, cx, cy);
	
	
	//Read in the initial guess
	YAML::Node guess_file;
	try{
		guess_file = YAML::LoadFile(argv[2]);
	} catch(YAML::BadFile e){//If file is not extant and well-formed...
		printf("\e[39mGuess file \"%s\" does not exist or contains syntax errors.\e[31m\n", argv[2]);
		return 0;
	}
	double c_x, c_y, c_z, c_r, c_p, c_w;
	double t_x, t_y, t_z, t_r, t_p, t_w;
	double b_x, b_y, b_z, b_r, b_p, b_w;
	try{
		c_x = guess_file["cam_x"].as<double>();
		c_y = guess_file["cam_y"].as<double>();
		c_z = guess_file["cam_z"].as<double>();
		c_r = guess_file["cam_r"].as<double>();
		c_p = guess_file["cam_p"].as<double>();
		c_w = guess_file["cam_w"].as<double>();
		
		t_x = guess_file["top_x"].as<double>();
		t_y = guess_file["top_y"].as<double>();
		t_z = guess_file["top_z"].as<double>();
		t_r = guess_file["top_r"].as<double>();
		t_p = guess_file["top_p"].as<double>();
		t_w = guess_file["top_w"].as<double>();
		
		b_x = guess_file["bot_x"].as<double>();
		b_y = guess_file["bot_y"].as<double>();
		b_z = guess_file["bot_z"].as<double>();
		b_r = guess_file["bot_r"].as<double>();
		b_p = guess_file["bot_p"].as<double>();
		b_w = guess_file["bot_w"].as<double>();
	} catch(YAML::RepresentationException e){
		printf("\e[39mGuess parse exception \"%s\".\e[31m\n", e.what());
		return 0;
	}
	printf("\nSuccessfully initialized guesses from %s:\n", argv[2]);
	printf("\tCamera xyz = (%f %f %f)\t\t rpy = (%f %f %f)\n", c_x, c_y, c_z, c_r, c_p, c_w);
	printf("\tTop marker xyz = (%f %f %f)\t\t rpy = (%f %f %f)\n", t_x, t_y, t_z, t_r, t_p, t_w);
	printf("\tBottom marker xyz = (%f %f %f)\t rpy = (%f %f %f)\n", b_x, b_y, b_z, b_r, b_p, b_w);
	
	
	//Load in calibration data
	std::ifstream data_file;
	data_file.open(argv[1]);
	if(!data_file){
		printf("\e[39mCould not find data file \"%s\".\e[31m\n", argv[1]);
		return 0;
	}
	
	std::vector<Eigen::Affine3d>	arm_locations;
	std::vector<bool>		topnesses;
	std::vector<Eigen::Vector2d>	corner_locations;
	std::vector<Eigen::Vector2d>	pixel_locations;
	
	double max_u = 0;
	double max_v = 0;
	
	int n = 0;
	char line [255];
	while(data_file.getline(line, 255)){
		n++;
		std::string line_s = std::string(line);
		if(std::count(line_s.begin(), line_s.end(), ',') != 19){
			printf("\e[39mBad file format. Line %d (%s).\e[31m\n", n, line);
			return 0;
		}
		
		double position_array_form [16];
		int top_number;
		int corner_number;
		Eigen::Vector2d pixel_location;
		
		sscanf(line, "%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %d, %d, %lf, %lf",
		
			&position_array_form[0 ], &position_array_form[1 ], &position_array_form[2 ], &position_array_form[3 ],
			&position_array_form[4 ], &position_array_form[5 ], &position_array_form[6 ], &position_array_form[7 ],
			&position_array_form[8 ], &position_array_form[9 ], &position_array_form[10], &position_array_form[11],
			&position_array_form[12], &position_array_form[13], &position_array_form[14], &position_array_form[15],
			
			&top_number, &corner_number,
			&pixel_location.x(), &pixel_location.y()
		);
		
		Eigen::Affine3d a;
		Eigen::Matrix4d m;
		m <<
			position_array_form[0 ], position_array_form[1 ], position_array_form[2 ], position_array_form[3 ],
			position_array_form[4 ], position_array_form[5 ], position_array_form[6 ], position_array_form[7 ],
			position_array_form[8 ], position_array_form[9 ], position_array_form[10], position_array_form[11],
			position_array_form[12], position_array_form[13], position_array_form[14], position_array_form[15]
		;
		a.matrix() = m;
		arm_locations.push_back(a/*.inverse()*/);//We want FOREARM_to_BASE but TF saves BASE_to_FOREARM.
		
		topnesses.push_back(top_number == 17);
		
		switch(corner_number){
			case 0://Top left	-x -y
				corner_locations.push_back(Eigen::Vector2d(-ARUCO_MARKER_SIZE, -ARUCO_MARKER_SIZE));
			break;
			case 1://Top right	-x +y
				corner_locations.push_back(Eigen::Vector2d(-ARUCO_MARKER_SIZE,  ARUCO_MARKER_SIZE));
			break;
			case 2://Bottom right	+x +y
				corner_locations.push_back(Eigen::Vector2d( ARUCO_MARKER_SIZE,  ARUCO_MARKER_SIZE));
			break;
			case 3://Bottom left	+x -y
				corner_locations.push_back(Eigen::Vector2d( ARUCO_MARKER_SIZE, -ARUCO_MARKER_SIZE));
			break;
		}
		
		pixel_locations.push_back(pixel_location);
		//Since the resolution of the rectified image is surprisingly
		//hard to get, we just take the maximum of the data and use that.
		if(pixel_location.x() > max_u){
			max_u = pixel_location.x();
		}
		if(pixel_location.y() > max_v){
			max_v = pixel_location.y();
		}
	}
	printf("\nRead in \e[1m%d\e[0m entries from %s.\n\n", n, argv[1]);
	//std::cout << arm_locations[0].matrix() << "\n";
	
	
	//Build the optimization problem
	ceres::Problem problem;
	ceres::Solver::Options options;
	
	
	//Initialize the unknown values from their defaults.
	double BASE_to_TOP_t [3] = {t_x, t_y, t_z};
	double BASE_to_TOP_r [3] = {cc_utils::rtod(t_r), cc_utils::rtod(t_p), cc_utils::rtod(t_w)};
	double BASE_to_BOT_t [3] = {b_x, b_y, b_z};
	double BASE_to_BOT_r [3] = {cc_utils::rtod(b_r), cc_utils::rtod(b_p), cc_utils::rtod(b_w)};
	
	//Camera calibration is most useful externally as FOREARM_to_CAM,
	//but for the math to work here we want CAM_to_FOREARM. So we will convert.
	Eigen::Affine3d a;
	a =
		Eigen::AngleAxisd(c_r, Eigen::Vector3d::UnitZ()) *
   		Eigen::AngleAxisd(c_p, Eigen::Vector3d::UnitY()) *
   		Eigen::AngleAxisd(c_w, Eigen::Vector3d::UnitX());
   	a.translation() = Eigen::Vector3d(c_x, c_y, c_z);
   	
   	Eigen::Affine3d b = a.inverse();
   	Eigen::Vector3d ea = b.rotation().eulerAngles(2, 1, 0);
   	
	double CAM_to_FOREARM_t [3] = {
		b.translation().x(),
		b.translation().y(),
		b.translation().z(),
	};
	double CAM_to_FOREARM_r [3] = {
		cc_utils::rtod(ea.x()),
		cc_utils::rtod(ea.y()),
		cc_utils::rtod(ea.z())
	};
	
	
	
	//Initialize visualization.
	cc_utils::init_visualization(std::ceil(max_u) + 10, std::ceil(max_v) + 10, pixel_locations, options);

	
	//Pack the problem.
	for(int i = 0; i < n; i++){
		double pixels_as_array [2] = { pixel_locations[i].x(), pixel_locations[i].y() };
		
		double target_as_array [2] = { corner_locations[i].x(), corner_locations[i].y() };
		
		double arm_translation_as_array [3] = {
			arm_locations[i].translation().x(),
			arm_locations[i].translation().y(),
			arm_locations[i].translation().z()
		};
		double arm_rotation_as_array [9] = {
			arm_locations[i].linear()(0,0), arm_locations[i].linear()(0,1), arm_locations[i].linear()(0,2),
			arm_locations[i].linear()(1,0), arm_locations[i].linear()(1,1), arm_locations[i].linear()(1,2),
			arm_locations[i].linear()(2,0), arm_locations[i].linear()(2,1), arm_locations[i].linear()(2,2)
		};
		
		
		ceres::CostFunction *cost_function = CalibrationEntry::Create(
			pixels_as_array,
			
			target_as_array,
			topnesses[i],
			
			arm_translation_as_array,
			arm_rotation_as_array,
			
			fx, fy, cx, cy
		);
		
		
		problem.AddResidualBlock(cost_function, NULL,
			BASE_to_TOP_t, BASE_to_TOP_r,
			BASE_to_BOT_t, BASE_to_BOT_r,
		
			CAM_to_FOREARM_t, CAM_to_FOREARM_r
		);
	}
	
	
	//Bound the rotations.
	cc_utils::bound_rotation(problem, BASE_to_TOP_r);
	cc_utils::bound_rotation(problem, BASE_to_BOT_r);
	cc_utils::bound_rotation(problem, CAM_to_FOREARM_r);
	
	
	//Run the solver!
	options.minimizer_progress_to_stdout = true;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.max_num_iterations = 1000;
	ceres::Solver::Summary summary;
    	ceres::Solve(options, &problem, &summary);
    	
    	
    	//Convert the camera-to-forearm transform back into forearm-to-camera for easy comparison
    	a = 
		Eigen::AngleAxisd(cc_utils::dtor(CAM_to_FOREARM_r[0]), Eigen::Vector3d::UnitZ()) *
   		Eigen::AngleAxisd(cc_utils::dtor(CAM_to_FOREARM_r[1]), Eigen::Vector3d::UnitY()) *
   		Eigen::AngleAxisd(cc_utils::dtor(CAM_to_FOREARM_r[2]), Eigen::Vector3d::UnitX());
   	a.translation() = Eigen::Vector3d(CAM_to_FOREARM_t[0], CAM_to_FOREARM_t[1], CAM_to_FOREARM_t[2]);
   	
   	b = a.inverse();
   	ea = b.rotation().eulerAngles(2, 1, 0);
   		
   	double c_x_out = b.translation().x();
   	double c_y_out = b.translation().y();
   	double c_z_out = b.translation().z();
   	double c_r_out = ea.x();
   	double c_p_out = ea.y();
   	double c_w_out = ea.z();
   	
   	
   	//Output goodness-of-fit data
    	printf("\nCalibration complete.\n\n");
    	
    	printf("\e[36mRMS value is \e[35m%f px\e[36m.\n", cc_utils::rms());
    	
   	double drift_top_m = sqrt(
   		pow(BASE_to_TOP_t[0] - t_x, 2) +
   		pow(BASE_to_TOP_t[1] - t_y, 2) +
   		pow(BASE_to_TOP_t[2] - t_z, 2)
   	);
   	double drift_top_r = sqrt(
   		pow(cc_utils::dtor(BASE_to_TOP_r[0]) - t_r, 2) +
   		pow(cc_utils::dtor(BASE_to_TOP_r[1]) - t_p, 2) +
   		pow(cc_utils::dtor(BASE_to_TOP_r[2]) - t_w, 2)
   	);
   	double drift_bot_m = sqrt(
   		pow(BASE_to_BOT_t[0] - b_x, 2) +
   		pow(BASE_to_BOT_t[1] - b_y, 2) +
   		pow(BASE_to_BOT_t[2] - b_z, 2)
   	);
   	double drift_bot_r = sqrt(
   		pow(cc_utils::dtor(BASE_to_BOT_r[0]) - b_r, 2) +
   		pow(cc_utils::dtor(BASE_to_BOT_r[1]) - b_p, 2) +
   		pow(cc_utils::dtor(BASE_to_BOT_r[2]) - b_w, 2)
   	);
   	
   	printf("\e[36mDrift for top target is \e[35m%f\e[36mm / \e[35m%f\e[36mrad.\n", drift_top_m, drift_top_r);
   	printf("\e[36mDrift for bot target is \e[35m%f\e[36mm / \e[35m%f\e[36mrad.\n", drift_bot_m, drift_bot_r);
   	
   	//Output calibration data.
   	std::printf("\nFOREARM to CAMERA:\n");
	std::printf("\tx = \e[35m%f\e[36m\ty = \e[35m%f\e[36m\tz = \e[35m%f\e[36m\n", c_x_out, c_y_out, c_z_out);
	std::printf("\tr = \e[35m%f\e[36m\tp = \e[35m%f\e[36m\tw = \e[35m%f\e[36m\n", c_r_out, c_p_out, c_w_out);
	std::cout << "\n\e[35m" << b.matrix() << "\e[36m\n";
   	
   	//Output calibration data.
   	std::printf("\nBASE to TOP TARGET:\n");
	std::printf(
		"\tx = \e[35m%f\e[36m\ty = \e[35m%f\e[36m\tz = \e[35m%f\e[36m\n",
		BASE_to_TOP_t[0], BASE_to_TOP_t[1], BASE_to_TOP_t[2]
	);
	std::printf(
		"\tr = \e[35m%f\e[36m\tp = \e[35m%f\e[36m\tw = \e[35m%f\e[36m\n",
		cc_utils::dtor(BASE_to_TOP_r[0]), cc_utils::dtor(BASE_to_TOP_r[1]), cc_utils::dtor(BASE_to_TOP_r[2])
	);
   	
   	//Output calibration data.
   	std::printf("\nBASE to BOTTOM TARGET:\n");
	std::printf(
		"\tx = \e[35m%f\e[36m\ty = \e[35m%f\e[36m\tz = \e[35m%f\e[36m\n",
		BASE_to_BOT_t[0], BASE_to_BOT_t[1], BASE_to_BOT_t[2]
	);
	std::printf(
		"\tr = \e[35m%f\e[36m\tp = \e[35m%f\e[36m\tw = \e[35m%f\e[39m\n",
		cc_utils::dtor(BASE_to_BOT_r[0]), cc_utils::dtor(BASE_to_BOT_r[1]), cc_utils::dtor(BASE_to_BOT_r[2])
	);
	
	if(output == NULL){
		printf("\nNo output file specified. Calibration data will not be saved.\n\n");
		return 0;
	}
	
	

	return 0;
}
