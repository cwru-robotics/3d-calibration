#include <fstream>
#include <iostream>
#include <math.h>
#include <unistd.h>

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
		const int r_x_in, const int r_y_in,
		const double cx_in, const double cy_in
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
		
		c_x = cx_in;
		c_y = cy_in;
	}
	static ceres::CostFunction* Create(
		const double image_pixels_in[2],
		const double MILL_to_SLED_translation_in[3],
		const double TARGET_to_POINT_translation_in[3],
		
		const int r_x_in, const int r_y_in,
		const double c_x_in, const double c_y_in
		
	){
		return new ceres::AutoDiffCostFunction<CalibrationEntry, 2,//Residual output comes first.
		//	target rotation	base translation	base rotation	projection	distortion
			3,			3,			3,		2,		5
		>(new CalibrationEntry (//Just pass all the arguments in in the same order.
			image_pixels_in, MILL_to_SLED_translation_in, TARGET_to_POINT_translation_in, r_x_in, r_y_in, c_x_in, c_y_in
		));
	}
	
	
	//Constant member vars
	//Perpoints
	double image_pixels[2];
	double MILL_to_SLED_translation[3];
	double TARGET_to_POINT_translation[3];
	//Globals
	int r_x, r_y;
	double c_x, c_y;
	
	template<typename T> bool operator()(//TODO Why are all these const / should all these be const?
		const T* SLED_to_TARGET_rotation,
		const T* CAM_to_MILL_translation, const T* CAM_to_MILL_rotation,
		const T* projection, const T* distortion,
		
		T* residual
	) const {
		//1: Transform target points into camera frame
		//	CAM_to_POINT = CAM_to_BASE * BASE_to_TIP * TIP_to_TARGET * TARGET_to_POINT
	
		//	1a: TARGET_to_POINT
		T TARGET_to_POINT [3];
		TARGET_to_POINT[0] = T(TARGET_to_POINT_translation[0]);
		TARGET_to_POINT[1] = T(TARGET_to_POINT_translation[1]);
		TARGET_to_POINT[2] = T(TARGET_to_POINT_translation[2]);
	
		//std::cout << "TARGET TO POINT\n";
		//std::cout << cc_utils::val(TARGET_to_POINT[0]) << "\n";
		//std::cout << cc_utils::val(TARGET_to_POINT[1]) << "\n";
		//std::cout << cc_utils::val(TARGET_to_POINT[2]) << "\n\n";
		
		//	1b: SLED_to_TARGET * TARGET_to_POINT
		T SLED_to_POINT [3];
		const T no_translation[3] = {T(0.0), T(0.0), T(0.0)};
		cc_utils::transformPoint_euler(no_translation, SLED_to_TARGET_rotation/*no_translation*/, TARGET_to_POINT, SLED_to_POINT);
		
		//std::cout << "SLED TO POINT\n";
		//std::cout << cc_utils::val(SLED_to_POINT[0]) << "\n";
		//std::cout << cc_utils::val(SLED_to_POINT[1]) << "\n";
		//std::cout << cc_utils::val(SLED_to_POINT[2]) << "\n\n";
		
		//	1c: MILL_to_SLED * SLED_to_TARGET * TARGET_to_POINT
		//MILL to SLED is just a translation
		T MILL_to_POINT [3] = {
			T(MILL_to_SLED_translation[0] + SLED_to_POINT[0]),
			T(MILL_to_SLED_translation[1] + SLED_to_POINT[1]),
			T(MILL_to_SLED_translation[2] + SLED_to_POINT[2])
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
			projection[0], projection[1], T(c_x), T(c_y),
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

	for(int i = 0; i < argc; i++){
		printf("%s\n", argv[i]);
	}

	if(argc < 5){
		printf("\e[33mUsage: rosrun intrinsic_calibration calibrator /path/to/data.csv /path/to/initial/position.yml /path/to/initial/intrinsics.yml camera-name [-o path/to/output.yml] [-p path/to/position.yml]\e[39m\n");
		return 0;
	}
	
	//Read in initialization info
	YAML::Node position_file;
	try{
		position_file = YAML::LoadFile(argv[2]);
	} catch(YAML::BadFile e){//If file is not extant and well-formed...
		printf("\e[39mInitial position file \"%s\" does not exist or contains syntax errors.\e[31m\n", argv[2]);
		return 0;
	}
	double MtC_init_x, MtC_init_y, MtC_init_z, MtC_init_r, MtC_init_p, MtC_init_w;
	double trg_init_r, trg_init_p, trg_init_w;
	int resolution_x, resolution_y;
	try{
		//Camera to mill
		MtC_init_x = position_file["mill_to_camera_x"].as<double>();
		MtC_init_y = position_file["mill_to_camera_y"].as<double>();
		MtC_init_z = position_file["mill_to_camera_z"].as<double>();
		
		MtC_init_r = position_file["mill_to_camera_r"].as<double>();
		MtC_init_p = position_file["mill_to_camera_p"].as<double>();
		MtC_init_w = position_file["mill_to_camera_w"].as<double>();
	
		//Millhead to target
		trg_init_r = position_file["sled_to_target_r"].as<double>();
		trg_init_p = position_file["sled_to_target_p"].as<double>();
		trg_init_w = position_file["sled_to_target_w"].as<double>();
		
		
		//Pixel values
		resolution_x = position_file["resolution_u"].as<int>();
		resolution_y = position_file["resolution_v"].as<int>();
		
	} catch(YAML::Exception e){
		printf("\e[39mPosition parse exception \"%s\" in %s.\e[31m\n", e.what(), argv[2]);
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
		cx_init = intrinsic_file["cx"].as<double>();//These two values are not used (replaced by the calculation of c_fixed), but I am keeping them around for consistency with the base files.
		cy_init = intrinsic_file["cy"].as<double>();
		
		k1_init = intrinsic_file["k1"].as<double>();
		k2_init = intrinsic_file["k2"].as<double>();
		k3_init = intrinsic_file["k3"].as<double>();
		p1_init = intrinsic_file["p1"].as<double>();
		p2_init = intrinsic_file["p2"].as<double>();
	} catch(YAML::Exception e){
		printf("\e[39mIntrinsic parse exception \"%s\".\e[31m\n", e.what());
		return 0;
	}
	
	double cx_fixed = (double)resolution_x / 2.0;
	double cy_fixed = (double)resolution_y / 2.0;
	
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
	
	char * output;
	if(getopt(argc, argv, "o:") != -1){
		output = optarg;
	} else{
		printf("\nNo designated output file given, calibrated data will be written directly to the initialization file.\n");
		output = argv[3];
	}
	
	char * position = NULL;
	if(getopt(argc, argv, "p:") != -1){
		position = optarg;
	}
	
	//Build the optimization problem
	ceres::Problem problem;
	ceres::Solver::Options options;
	
	//Initialize the unknown values from their defaults.
	double SLED_to_TARGET_r [3] = {cc_utils::rtod(trg_init_r), cc_utils::rtod(trg_init_p), cc_utils::rtod(trg_init_w)};
	double CAM_to_MILL_t [3] = {MtC_init_x, MtC_init_y, MtC_init_z};
	double CAM_to_MILL_r [3] = {cc_utils::rtod(MtC_init_r), cc_utils::rtod(MtC_init_p), cc_utils::rtod(MtC_init_w)};

	double projection[2] = {
		//fx		fy		cx		cy
		fx_init,	fy_init//,	cx_init,	cy_init	Projection center is now fixed.
	};
	
	double distortion[5] = {k1_init, k2_init, k3_init, p1_init, p2_init};
	
	//Set up visualization.
	cc_utils::init_visualization(resolution_x, resolution_y, pixels, options);
	
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
			resolution_y, resolution_x,
			cx_fixed, cy_fixed
		);
		
		//And then the global parameters to optimize
		problem.AddResidualBlock(cost_function, NULL,
			SLED_to_TARGET_r,
			CAM_to_MILL_t, CAM_to_MILL_r,
			projection, distortion
		);
	}
	
	//Bound the two rotations.
	cc_utils::bound_rotation(problem, SLED_to_TARGET_r);
	cc_utils::bound_rotation(problem, CAM_to_MILL_r);
	
	//Bound the focal lengths to + values
	problem.SetParameterLowerBound(projection, 0, 0.0);
	problem.SetParameterLowerBound(projection, 1, 0.0);
	
	//Run the solver!
	options.minimizer_progress_to_stdout = true;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.max_num_iterations = 1000;
	ceres::Solver::Summary summary;
    	ceres::Solve(options, &problem, &summary);
    	

    	//Convert the camera-to-mill transform back into mill-to-camera for easy comparison
	double MtC_x = CAM_to_MILL_t[0];
	double MtC_y = CAM_to_MILL_t[1];
	double MtC_z = CAM_to_MILL_t[2];
	double MtC_r = cc_utils::dtor(CAM_to_MILL_r[0]);
	double MtC_p = cc_utils::dtor(CAM_to_MILL_r[1]);
	double MtC_w = cc_utils::dtor(CAM_to_MILL_r[2]);
	
	Eigen::Affine3d b;
	b =
		Eigen::AngleAxisd(MtC_w, Eigen::Vector3d::UnitZ()) *
   		Eigen::AngleAxisd(MtC_p, Eigen::Vector3d::UnitY()) *
   		Eigen::AngleAxisd(MtC_r, Eigen::Vector3d::UnitX());
   	b.translation() = Eigen::Vector3d(MtC_x, MtC_y, MtC_z);
	
    	//Calculate the RMS and display the values:
    	printf("\nCalibration complete.\n");
    	double rms = cc_utils::rms();
    	printf("\e[36mRMS value is \e[35m%f px\e[36m.\n", rms);
    	
    	if(std::isnan(rms)){
    		printf("Solver did not converge. Won't save results.\n");
    		return 0;
    	}
    	
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
	
	printf("In long form that is %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
		b.matrix()(0, 0), b.matrix()(0, 1), b.matrix()(0, 2), b.matrix()(0, 3),
		b.matrix()(1, 0), b.matrix()(1, 1), b.matrix()(1, 2), b.matrix()(1, 3),
		b.matrix()(2, 0), b.matrix()(2, 1), b.matrix()(2, 2), b.matrix()(2, 3),
		b.matrix()(3, 0), b.matrix()(3, 1), b.matrix()(3, 2), b.matrix()(3, 3)
	);
	
	std::printf("INTRINSICS:\n");
	std::printf(
		"\tfx = \e[35m%f\e[36m\tfy = \e[35m%f\e[36m\t cx = \e[35m%f\e[36m\t cy = \e[35m%f\e[36m\n",
		//fx		fy		cx		cy
		projection[0], projection[1], cx_fixed, cy_fixed
	);
	std::printf(
		"\tk1 = \e[35m%f\e[36m\tk2 = \e[35m%f\e[36m\t k3 = \e[35m%f\e[36m\t p1 = \e[35m%f\e[36m\t p2 = \e[35m%f\e[39m\n",
		distortion[0], distortion[1], distortion[2], distortion[3], distortion[4]
	);
	
	YAML::Node output_node;
	output_node["fx"] = projection[0];
	output_node["fy"] = projection[1];
	output_node["cx"] = cx_fixed;
	output_node["cy"] = cy_fixed;
	output_node["k1"] = distortion[0];
	output_node["k2"] = distortion[1];
	output_node["k3"] = distortion[2];
	output_node["p1"] = distortion[3];
	output_node["p2"] = distortion[4];
	std::ofstream e_out;
	e_out.open(output);
	e_out << output_node;
	
	e_out << "\nimage_width: " << resolution_x << "\n";
	e_out << "image_height: " << resolution_y << "\n";
	e_out << "camera_name: endo_cam_l\n";
	e_out << "camera_matrix:\n";
	e_out << "  rows: 3\n";
	e_out << "  cols: 3\n";
	e_out << "  data: [" << projection[0] << ", 0.0, " << cx_fixed << ", 0.0, " << projection[1] << ", " << cy_fixed << ", 0.0, 0.0, 1.0]\n";
	e_out << "distortion_model: plumb_bob\n";
	e_out << "distortion_coefficients:\n";
	e_out << "  rows: 1\n";
	e_out << "  cols: 5\n";
	e_out << "  data: [" << distortion[0] << ", " << distortion[1] << ", " << distortion[3] << ", " << distortion[4] << ", " << distortion[2] << "]\n";
	e_out << "rectification_matrix:\n";
	e_out << "  rows: 3\n";
	e_out << "  cols: 3\n";
	e_out << "  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]\n";
	e_out << "projection_matrix:\n";
	e_out << "  rows: 3\n";
	e_out << "  cols: 4\n";
	e_out << "  data: [" << projection[0] << ", 0.0, " << cx_fixed << ", 0.0, 0.0, " << projection[1] << ", " << cy_fixed << ", 0.0, 0.0, 0.0, 1.0, 0.0]\n";
	
	e_out.close();
	printf("\nSaved results to %s.\n\n\n", output);
	
	if(position != NULL){
		std::ofstream e_out = std::ofstream(position);
		e_out << "translation: [" << MtC_x << ", " << MtC_y << ", " << MtC_z << "]\n";
		e_out << "rpy: [" << MtC_r << ", " << MtC_p << ", " << MtC_w << "]\n";
		e_out << "matrix: [" <<
			b.matrix()(0, 0) << ", " << b.matrix()(0, 1) << ", " << b.matrix()(0, 2) << ", " << b.matrix()(0, 3) << ", " <<
			b.matrix()(1, 0) << ", " << b.matrix()(1, 1) << ", " << b.matrix()(1, 2) << ", " << b.matrix()(1, 3) << ", " <<
			b.matrix()(2, 0) << ", " << b.matrix()(2, 1) << ", " << b.matrix()(2, 2) << ", " << b.matrix()(2, 3) << ", " <<
			b.matrix()(3, 0) << ", " << b.matrix()(3, 1) << ", " << b.matrix()(3, 2) << ", " << b.matrix()(3, 3) << "]\n"
		;
		
		//Same format as came in:
		e_out << "mill_to_camera_x: " << MtC_x << "\n";
		e_out << "mill_to_camera_y: " << MtC_y << "\n";
		e_out << "mill_to_camera_z: " << MtC_z << "\n";
		
		e_out << "mill_to_camera_r: " << MtC_r << "\n";
		e_out << "mill_to_camera_p: " << MtC_p << "\n";
		e_out << "mill_to_camera_w: " << MtC_w << "\n";
		
		
		
		e_out << "sled_to_target_r: " << cc_utils::dtor(SLED_to_TARGET_r[0]) << "\n";
		e_out << "sled_to_target_p: " << cc_utils::dtor(SLED_to_TARGET_r[1]) << "\n";
		e_out << "sled_to_target_w: " << cc_utils::dtor(SLED_to_TARGET_r[2]) << "\n";
		
		e_out << "resolution_u: " << resolution_x << "\n";
		e_out << "resolution_v: " << resolution_y << "\n";
		e_out.close();
	}

	return 0;
}
