#include <fstream>
#include <iostream>

#include <Eigen/Eigen>

#include "yaml-cpp/yaml.h"

#include <cc_utils/cc_utils.h>


class CalibrationEntry{
public:
	//Giant stupid constructor chain.
	//The fact that it takes two functions to pass the exact same arguments into the exact same variables is Ceres' fault; not mine.
	CalibrationEntry(
		const double image_pixels_in[2],
		const double MILL_to_SLED_translation_in[3],
		const double TARGET_to_POINT_translation_in[3],
		const int r_x_in, const int r_y_in
	){
		image_pixels[0] = image_pixels_in[0];
		image_pixels[1] = image_pixels_in[1];
		
		MILL_to_SLED_translation[0] = MILL_to_SLED_translation_in[0];
		MILL_to_SLED_translation[1] = MILL_to_SLED_translation_in[1];
		MILL_to_SLED_translation[2] = MILL_to_SLED_translation_in[2];
		
		TARGET_to_POINT_translation[0] = TARGET_to_POINT_translation_in[0];
		TARGET_to_POINT_translation[1] = TARGET_to_POINT_translation_in[1];
		TARGET_to_POINT_translation[2] = TARGET_to_POINT_translation_in[2];
		
		r_x = r_x_in;
		r_y = r_y_in;
	}
	static ceres::CostFunction* Create(
		const double image_pixels_in[2],
		const double MILL_to_SLED_translation_in[3],
		const double TARGET_to_POINT_translation_in[3],
		
		const int r_x_in, const int r_y_in
		
	){
		return new ceres::AutoDiffCostFunction<CalibrationEntry, 2,//Residual output comes first.
		//	target rotation		base translation	base rotation	projection	distortion
			3,			3,			3,		4,		5
		>(new CalibrationEntry (//Just pass all the arguments in in the same order.
			image_pixels_in, MILL_to_SLED_translation_in, TARGET_to_POINT_translation_in, r_x_in, r_y_in
		));
	}
	
	
	//Constant member vars
	//Perpoints
	double image_pixels[2];
	double MILL_to_SLED_translation[3];
	double TARGET_to_POINT_translation[3];
	//Globals
	int r_x, r_y;
	
	template<typename T> bool operator()(//TODO Why are all these const / should all these be const?
		const T* SLED_to_TARGET_rotation,
		const T* CAM_to_MILL_translation, const T* CAM_to_MILL_rotation,
		const T* projection, const T* distortion,
		const T* z_distort_x, const T* z_distort_y,
		
		T* residual
	) const {
		//1: Transform target points into camera frame
		//	CAM_to_POINT = CAM_to_BASE * BASE_to_TIP * TIP_to_TARGET * TARGET_to_POINT
	
		//	1a: TARGET_to_POINT
		T TARGET_to_POINT [3];
		TARGET_to_POINT[0] = T(TARGET_to_POINT_translation[0]);
		TARGET_to_POINT[1] = T(TARGET_to_POINT_translation[1]);
		TARGET_to_POINT[2] = T(TARGET_to_POINT_translation[2]);
	
		/*std::cout << "TARGET TO POINT\n";
		std::cout << TARGET_to_POINT[0] << "\n";
		std::cout << TARGET_to_POINT[1] << "\n";
		std::cout << TARGET_to_POINT[2] << "\n\n";*/
		
		//	1b: SLED_to_TARGET * TARGET_to_POINT
		T SLED_to_POINT [3];
		const T no_translation[3] = {T(0.0), T(0.0), T(0.0)};
		cc_utils::transformPoint_euler(no_translation, SLED_to_TARGET_rotation, TARGET_to_POINT, SLED_to_POINT);
		
		/*std::cout << "SLED TO POINT\n";
		std::cout << SLED_to_POINT[0] << "\n";
		std::cout << SLED_to_POINT[1] << "\n";
		std::cout << SLED_to_POINT[2] << "\n\n";*/
		
		//	1c: MILL_to_SLED * SLED_to_TARGET * TARGET_to_POINT
		//MILL to SLED is just a translation
		T MILL_to_POINT [3] = {
			T(MILL_to_SLED_translation[0] + MILL_to_SLED_translation[2] * sin(z_distort_x[0])) + SLED_to_POINT[0],
			T(MILL_to_SLED_translation[1] + MILL_to_SLED_translation[2] * sin(z_distort_y[0])) + SLED_to_POINT[1],
			T(MILL_to_SLED_translation[2] * cos(z_distort_y[0]) * cos(z_distort_x[0])) + SLED_to_POINT[2]
		};
		
		/*std::cout << "MILL TO POINT\n";
		std::cout << MILL_to_POINT[0] << "\n";
		std::cout << MILL_to_POINT[1] << "\n";
		std::cout << MILL_to_POINT[2] << "\n\n";*/
		
		//	1d: CAM_to_MILL * MILL_to_SLED * SLED_to_TARGET * TARGET_to_POINT
		T CAM_to_POINT [3];
		cc_utils::transformPoint_euler(CAM_to_MILL_translation, CAM_to_MILL_rotation, MILL_to_POINT, CAM_to_POINT);
		
		/*std::cout << "CAM TO POINT\n";
		std::cout << CAM_to_POINT[0] << "\n";
		std::cout << CAM_to_POINT[1] << "\n";
		std::cout << CAM_to_POINT[2] << "\n\n";*/
		
		T u, v;
		cc_utils::project(
			CAM_to_POINT[0], CAM_to_POINT[1], CAM_to_POINT[2],
			
			//fx		fy		cx		cy
			projection[0], projection[1], projection[2], projection[3],
			distortion[0], distortion[1], distortion[2], distortion[3], distortion[4],
		
			T(r_x), T(r_y),
		
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
		printf("\e[33mUsage: rosrun intrinsic_calibration calibrator /path/to/data.csv /path/to/initial/position.yml /path/to/initial/intrinsics.yml [path/to/output.yml]\e[39m\n");
		return 0;
	}
	
	char * output;
	if(argc < 5){
		printf("\nNo designated output file given, calibrated data will be written directly to the initialization file.\n");
		output = argv[3];
	} else {
		output = argv[4];
	}
	
	//Read in initialization info
	YAML::Node position_file;
	try{
		position_file = YAML::LoadFile(argv[2]);
	} catch(YAML::BadFile e){//If file is not extant and well-formed...
		printf("\e[39mPosition file \"%s\" does not exist or contains syntax errors.\e[31m\n", argv[2]);
		return 0;
	}
	double CtM_init_x, CtM_init_y, CtM_init_z, CtM_init_r, CtM_init_p, CtM_init_w;
	double trg_init_r, trg_init_p, trg_init_w;
	int resolution_x, resolution_y;
	try{
		//Camera to mill
		double MtC_init_x = position_file["mill_to_camera_guess_x"].as<double>();
		double MtC_init_y = position_file["mill_to_camera_guess_y"].as<double>();
		double MtC_init_z = position_file["mill_to_camera_guess_z"].as<double>();
		
		double MtC_init_r = position_file["mill_to_camera_guess_r"].as<double>();
		double MtC_init_p = position_file["mill_to_camera_guess_p"].as<double>();
		double MtC_init_w = position_file["mill_to_camera_guess_w"].as<double>();
		
		//Turn the user-friendly (and Gazebo-friendly) mill-to-camera guess into the necessary camera-to-mill format.
		double CtM_init_x, CtM_init_y, CtM_init_z,	CtM_init_r, CtM_init_p, CtM_init_w;
		cc_utils::invert_eul(
			MtC_init_x, MtC_init_y, MtC_init_z,	MtC_init_r, MtC_init_p, MtC_init_w,
			CtM_init_x, CtM_init_y, CtM_init_z,	CtM_init_r, CtM_init_p, CtM_init_w
		);
   		
   		
		//Millhead to target
		trg_init_r = position_file["sled_to_target_guess_r"].as<double>();
		trg_init_p = position_file["sled_to_target_guess_p"].as<double>();
		trg_init_w = position_file["sled_to_target_guess_w"].as<double>();
		
		
		//Pixel values
		resolution_x = position_file["res_x"].as<int>();
		resolution_y = position_file["res_y"].as<int>();
		
	} catch(YAML::RepresentationException e){
		printf("\e[39mPosition parse exception \"%s\".\e[31m\n", e.what());
		return 0;
	}
	printf("\nSuccessfully initialized positions from %s.\n", argv[2]);
	
	YAML::Node intrinsic_file;
	try{
		intrinsic_file = YAML::LoadFile(argv[3]);
	} catch(YAML::BadFile e){//If file is not extant and well-formed...
		printf("\e[39mInitial intrinsic file \"%s\" does not exist or contains syntax errors.\e[31m\n", argv[3]);
		return 0;
	}
	double fx_init, fy_init, cx_init, cy_init;
	double k1_init, k2_init, k3_init, p1_init, p2_init;
	try{
		fx_init = intrinsic_file["fx"].as<double>();
		fy_init = intrinsic_file["fy"].as<double>();
		cx_init = intrinsic_file["cx"].as<double>();
		cy_init = intrinsic_file["cy"].as<double>();
		
		k1_init = intrinsic_file["k1"].as<double>();
		k2_init = intrinsic_file["k2"].as<double>();
		k3_init = intrinsic_file["k3"].as<double>();
		p1_init = intrinsic_file["p1"].as<double>();
		p2_init = intrinsic_file["p2"].as<double>();
	} catch(YAML::RepresentationException e){
		printf("\e[39mIntrinsic parse exception \"%s\".\e[31m\n", e.what());
		return 0;
	}
	printf("Successfully initialized intrinsics from %s.\n", argv[3]);
	
	//Read in data
	std::ifstream data_file;
	data_file.open(argv[1]);
	if(!data_file){
		printf("\e[39mCould not find data file \"%s\".\e[31m\n", argv[1]);
		return 0;
	}
	
	std::vector<Eigen::Vector2d> pixels;
	std::vector<Eigen::Vector2d> target_coordinates;
	std::vector<Eigen::Vector3d> mill_coordinates;
	int n = 0;
	char line [128];
	while(data_file.getline(line, 128)){
		n++;
		std::string line_s = std::string(line);
		if(std::count(line_s.begin(), line_s.end(), ',') != 6){
			printf("\e[39mBad file format. Line %d (%s).\e[31m\n", n, line);
			return 0;
		}
		
		Eigen::Vector2d px_entry;
		Eigen::Vector2d target_entry;
		Eigen::Vector3d mill_entry;
		sscanf(line, "%lf, %lf, %lf, %lf, %lf, %lf, %lf",
			&px_entry.x(), &px_entry.y(),
			&target_entry.x(), &target_entry.y(),
			&mill_entry.x(), &mill_entry.y(), &mill_entry.z()
		);
		
		pixels.push_back(px_entry);
		target_coordinates.push_back(target_entry);
		mill_coordinates.push_back(mill_entry);
	}
	printf("Read in \e[1m%d\e[0m entries from %s.\n\n", n, argv[1]);
	
	//Build the optimization problem
	ceres::Problem problem;
	ceres::Solver::Options options;
	
	//Initialize the unknown values from their defaults.
	double SLED_to_TARGET_r [3] = {cc_utils::rtod(trg_init_r), cc_utils::rtod(trg_init_p), cc_utils::rtod(trg_init_w)};
	double CAM_to_MILL_t [3] = {CtM_init_x, CtM_init_y, CtM_init_z};
	double CAM_to_MILL_r [3] = {cc_utils::rtod(CtM_init_r), cc_utils::rtod(CtM_init_p), cc_utils::rtod(CtM_init_w)};
	

	double projection[4] = {
		//fx		fy		cx		cy
		fx_init,	fy_init,	cx_init,	cy_init
	};
	
	double distortion[5] = {k1_init, k2_init, k3_init, p1_init, p2_init};
	
	double z_distort_x = 0.0;
	double z_distort_y = 0.0;
	
	//Set up visualization.
	cc_utils::init_visualization(resolution_y, resolution_x, pixels, options);
	
	for(int i = 0; i < n; i++){
		//Add the perpoint constants.
		double pixels_array[2] = {pixels[i].x(), pixels[i].y()};
		double mill_array[3] = {mill_coordinates[i].x(), mill_coordinates[i].y(), mill_coordinates[i].z()};
		double target_array[3] = {target_coordinates[i].x(), target_coordinates[i].y(), 0};
		
		ceres::CostFunction *cost_function = CalibrationEntry::Create(
			pixels_array,
			mill_array,
			target_array,
			//And the universal constants.
			resolution_y, resolution_x
		);
		
		//And then the global parameters to optimize
		problem.AddResidualBlock(cost_function, NULL,
			SLED_to_TARGET_r,
			CAM_to_MILL_t, CAM_to_MILL_r,
			projection, distortion,
			&z_distort_x, &z_distort_y
		);
	}
	
	//Bound the two rotations.
	cc_utils::bound_rotation(problem, SLED_to_TARGET_r);
	cc_utils::bound_rotation(problem, CAM_to_MILL_r);
	
	//Run the solver!
	options.minimizer_progress_to_stdout = true;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.max_num_iterations = 1000;
	ceres::Solver::Summary summary;
    	ceres::Solve(options, &problem, &summary);
    	
    	//Convert the camera-to-mill transform back into mill-to-camera for easy comparison
    	double MtC_x, MtC_y, MtC_z,	MtC_r, MtC_p, MtC_w;
	Eigen::Affine3d b = cc_utils::invert_eul(
		CAM_to_MILL_t[0], CAM_to_MILL_t[1], CAM_to_MILL_t[2],
		cc_utils::dtor(CAM_to_MILL_r[0]), cc_utils::dtor(CAM_to_MILL_r[0]), cc_utils::dtor(CAM_to_MILL_r[0]),
		
		MtC_x, MtC_y, MtC_z,	MtC_r, MtC_p, MtC_w
	);
    	
    	//Calculate the RMS and display the values:
    	printf("\nCalibration complete.\n");
    	printf("\e[36mRMS value is \e[35m%f px\e[36m.\n", cc_utils::rms());
    	
    	std::printf("SLED to TARGET:\n");
	std::printf(
		"\tr = \e[35m%f\e[36m\tp = \e[35m%f\e[36m\tw = \e[35m%f\e[36m\n", 
		cc_utils::dtor(SLED_to_TARGET_r[0]),
		cc_utils::dtor(SLED_to_TARGET_r[1]),
		cc_utils::dtor(SLED_to_TARGET_r[2])
	);
	
	std::printf("CAMERA to MILL:\n");
	std::printf("\tx = \e[35m%f\e[36m\ty = \e[35m%f\e[36m\tz = \e[35m%f\e[36m\n", MtC_x, MtC_y, MtC_z);
	std::printf("\tr = \e[35m%f\e[36m\tp = \e[35m%f\e[36m\tw = \e[35m%f\n\n", MtC_r, MtC_p, MtC_w);
	std::printf("\t%f\t%f\t%f\t%f\n", b.matrix()(0, 0), b.matrix()(0, 1), b.matrix()(0, 2), b.matrix()(0, 3));
	std::printf("\t%f\t%f\t%f\t%f\n", b.matrix()(1, 0), b.matrix()(1, 1), b.matrix()(1, 2), b.matrix()(1, 3));
	std::printf("\t%f\t%f\t%f\t%f\n", b.matrix()(2, 0), b.matrix()(2, 1), b.matrix()(2, 2), b.matrix()(2, 3));
	std::printf("\t%f\t%f\t%f\t%f\e[36m\n", b.matrix()(3, 0), b.matrix()(3, 1), b.matrix()(3, 2), b.matrix()(3, 3));
	
	std::printf("INTRINSICS:\n");
	std::printf(
		"\tfx = \e[35m%f\e[36m\tfy = \e[35m%f\e[36m\t cx = \e[35m%f\e[36m\t cy = \e[35m%f\e[36m\n",
		//fx		fy		cx		cy
		projection[0], projection[1], projection[2], projection[3]
	);
	std::printf(
		"\tk1 = \e[35m%f\e[36m\tk2 = \e[35m%f\e[36m\t k3 = \e[35m%f\e[36m\t p1 = \e[35m%f\e[36m\t p2 = \e[35m%f\e[39m\n",
		distortion[0], distortion[1], distortion[2], distortion[3], distortion[4]
	);
	
	YAML::Node output_node;
	output_node["fx"] = projection[0];
	output_node["fy"] = projection[1];
	output_node["cx"] = projection[2];
	output_node["cy"] = projection[3];
	output_node["k1"] = distortion[0];
	output_node["k2"] = distortion[1];
	output_node["k3"] = distortion[2];
	output_node["p1"] = distortion[3];
	output_node["p2"] = distortion[4];
	std::ofstream e_out;
	e_out.open(output);
	e_out << output_node;
	e_out.close();
	printf("\nSaved results to %s.\n\n\n", output);
	
	printf("\n\n Zx distort is %f, Zy distort is %f\n\n", z_distort_x, z_distort_y);

	return 0;
}
