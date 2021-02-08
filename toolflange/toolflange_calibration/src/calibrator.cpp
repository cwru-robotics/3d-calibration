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
		const double point_in[3],
		double FLANGE_to_WRIST_translation_in[3],
		double FLANGE_to_WRIST_rotation_in[9]
	){
		point[0] = point_in[0];
		point[1] = point_in[1];
		point[2] = point_in[2];
		
		FLANGE_to_WRIST_translation[0] = FLANGE_to_WRIST_translation_in[0];
		FLANGE_to_WRIST_translation[1] = FLANGE_to_WRIST_translation_in[1];
		FLANGE_to_WRIST_translation[2] = FLANGE_to_WRIST_translation_in[2];
		
		for(int i = 0; i < 9; i++){
			FLANGE_to_WRIST_rotation[i] = FLANGE_to_WRIST_rotation_in[i];
		}
	}
	static ceres::CostFunction* Create(
		const double point_in[3],
		double FLANGE_to_WRIST_translation_in[3],
		double FLANGE_to_WRIST_rotation_in[9]
		
	){
		return new ceres::AutoDiffCostFunction<CalibrationEntry, 3,//Residual output comes first.
		
		//	Camera translation	Camera rotation		point translation
			3,			3,			3
		
		>(new CalibrationEntry (//Just pass all the arguments in in the same order.
			point_in, FLANGE_to_WRIST_translation_in, FLANGE_to_WRIST_rotation_in
		));
	}
	
	
	//Constant member vars
	//Perpoints
	double point[3];
	double FLANGE_to_WRIST_translation[3];
	double FLANGE_to_WRIST_rotation[9];
	
	template<typename T> bool operator()(//TODO Why are all these const / should all these be const?
		const T* CAM_to_FLANGE_translation, const T* CAM_to_FLANGE_rotation,
		const T* WRIST_to_POINT_translation,
		
		T* residual
	) const {
		//1: Transform target points into camera frame
		//	CAM_to_POINT = CAM_to_FLANGE * FLANGE_to_WRIST * WRIST_to_POINT
	
		//	1a: WRIST_to_POINT
		T WRIST_to_POINT [3];
		WRIST_to_POINT[0] = T(WRIST_to_POINT_translation[0]);
		WRIST_to_POINT[1] = T(WRIST_to_POINT_translation[1]);
		WRIST_to_POINT[2] = T(WRIST_to_POINT_translation[2]);
	
		/*std::cout << "WRIST TO POINT\n";
		std::cout << WRIST_to_POINT[0] << "\n";
		std::cout << WRIST_to_POINT[1] << "\n";
		std::cout << WRIST_to_POINT[2] << "\n\n";*/
		
		//	1b: FLANGE_to_WRIST * WRIST_to_POINT
		T FLANGE_to_WRIST_r[9];
		for(int i = 0; i < 9; i++){
			FLANGE_to_WRIST_r[i] = T(FLANGE_to_WRIST_rotation[i]);
		}
		T FLANGE_to_WRIST_t [3] = {
			T(FLANGE_to_WRIST_translation[0]),
			T(FLANGE_to_WRIST_translation[1]),
			T(FLANGE_to_WRIST_translation[2])
		};
		T FLANGE_to_POINT[3];
		cc_utils::transformPoint_rm(FLANGE_to_WRIST_t, FLANGE_to_WRIST_r, WRIST_to_POINT, FLANGE_to_POINT);
		
		/*std::cout << "FLANGE TO POINT\n";
		std::cout << FLANGE_to_POINT[0] << "\n";
		std::cout << FLANGE_to_POINT[1] << "\n";
		std::cout << FLANGE_to_POINT[2] << "\n\n";*/
		
		//	1c: CAM_to_FLANGE * FLANGE_to_WRIST * WRIST_to_POINT
		T CAM_to_POINT [3];
		cc_utils::transformPoint_euler(CAM_to_FLANGE_translation, CAM_to_FLANGE_rotation, FLANGE_to_POINT, CAM_to_POINT);
		
		/*std::cout << "CAM TO POINT\n";
		std::cout << CAM_to_POINT[0] << "\n";
		std::cout << CAM_to_POINT[1] << "\n";
		std::cout << CAM_to_POINT[2] << "\n\n";*/
		
		//cc_utils::add_to_visualization(cc_utils::val(u), cc_utils::val(v), image_pixels[0], image_pixels[1]);
		
		residual[0] = CAM_to_POINT[0] - T(point[0]);
		residual[1] = CAM_to_POINT[1] - T(point[1]);
		residual[2] = CAM_to_POINT[2] - T(point[2]);
		
		return true;
	}
};

int main(int argc, char** argv) {
	if(argc < 3){
		printf("\e[33mUsage: rosrun toolflange_calibration calibrator /path/to/data.csv /path/to/initial/position.yml [path/to/output.yml]\e[39m\n");
		return 0;
	}
	
	char * output;
	if(argc < 4){
		printf("\nNo designated output file given, calibrated data will be written directly to the initialization file.\n");
		output = argv[2];
	} else {
		output = argv[3];
	}
	
	//Read in initialization info
	YAML::Node position_file;
	try{
		position_file = YAML::LoadFile(argv[2]);
	} catch(YAML::BadFile e){//If file is not extant and well-formed...
		printf("\e[39mPosition file \"%s\" does not exist or contains syntax errors.\e[31m\n", argv[2]);
		return 0;
	}
	double CtF_init_x, CtF_init_y, CtF_init_z, CtF_init_r, CtF_init_p, CtF_init_w;
	double WtP_init_x, WtP_init_y, WtP_init_z;
	try{
		//Flange to camera
		double FtC_init_x = position_file["cam_x"].as<double>();
		double FtC_init_y = position_file["cam_y"].as<double>();
		double FtC_init_z = position_file["cam_z"].as<double>();
		
		double FtC_init_r = position_file["cam_r"].as<double>();
		double FtC_init_p = position_file["cam_p"].as<double>();
		double FtC_init_w = position_file["cam_w"].as<double>();
		
		//Turn the user-friendly (and Gazebo-friendly) flange-to-camera guess into the necessary camera-to-flange format.
		cc_utils::invert_eul(
			FtC_init_x, FtC_init_y, FtC_init_z,	FtC_init_r, FtC_init_p, FtC_init_w,
			CtF_init_x, CtF_init_y, CtF_init_z,	CtF_init_r, CtF_init_p, CtF_init_w
		);
		
		//Flange to camera
		WtP_init_x = position_file["wtp_x"].as<double>();
		WtP_init_y = position_file["wtp_y"].as<double>();
		WtP_init_z = position_file["wtp_z"].as<double>();
	} catch(YAML::RepresentationException e){
		printf("\e[39mPosition parse exception \"%s\".\e[31m\n", e.what());
		return 0;
	}
	printf("\nSuccessfully initialized positions from %s.\n", argv[2]);
	
	//Read in data
	std::ifstream data_file;
	data_file.open(argv[1]);
	if(!data_file){
		printf("\e[39mCould not find data file \"%s\".\e[31m\n", argv[1]);
		return 0;
	}
	
	std::vector<Eigen::Affine3d> flange_positions;
	std::vector<Eigen::Vector3d> point_coordinates;
	int n = 0;
	char line [256];
	while(data_file.getline(line, 256)){
		std::string line_s = std::string(line);
		if(std::count(line_s.begin(), line_s.end(), ',') != 18){
			printf("\e[39mBad file format. Line %d (%s).\e[31m\n", n, line);
			return 0;
		}
		
		Eigen::Vector3d point_coordinate;
		double position_array_form [16];
		
		sscanf(line, "%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf",
			
			&position_array_form[0 ], &position_array_form[1 ], &position_array_form[2 ], &position_array_form[3 ],
			&position_array_form[4 ], &position_array_form[5 ], &position_array_form[6 ], &position_array_form[7 ],
			&position_array_form[8 ], &position_array_form[9 ], &position_array_form[10], &position_array_form[11],
			&position_array_form[12], &position_array_form[13], &position_array_form[14], &position_array_form[15],
		
			&point_coordinate.x(), &point_coordinate.y(),  &point_coordinate.z()
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
		
		flange_positions.push_back(a);
		point_coordinates.push_back(point_coordinate);
		n++;
	}
	printf("Read in \e[1m%d\e[0m entries from %s.\n\n", n, argv[1]);
	
	//Build the optimization problem
	ceres::Problem problem;
	ceres::Solver::Options options;
	
	//Initialize the unknown values from their defaults.
	double CAM_to_FLANGE_t [3] = {CtF_init_x, CtF_init_y, CtF_init_z};
	double CAM_to_FLANGE_r [3] = {cc_utils::rtod(CtF_init_r), cc_utils::rtod(CtF_init_p), cc_utils::rtod(CtF_init_w)};
	
	double WRIST_to_POINT_t [3] = {WtP_init_x, WtP_init_y, WtP_init_z};

	
	//Set up visualization.
	//cc_utils::init_visualization(resolution_y, resolution_x, pixels, options);
	
	for(int i = 0; i < n; i++){
		//Add the perpoint constants.
		double point_as_array[3] = {
			point_coordinates[i].x(),
			point_coordinates[i].y(), 
			point_coordinates[i].z()
		};
		double flange_translation_as_array [3] = {
			flange_positions[i].translation().x(),
			flange_positions[i].translation().y(),
			flange_positions[i].translation().z()
		};
		double flange_rotation_as_array [9] = {
			flange_positions[i].linear()(0,0), flange_positions[i].linear()(0,1), flange_positions[i].linear()(0,2),
			flange_positions[i].linear()(1,0), flange_positions[i].linear()(1,1), flange_positions[i].linear()(1,2),
			flange_positions[i].linear()(2,0), flange_positions[i].linear()(2,1), flange_positions[i].linear()(2,2)
		};
		
		
		ceres::CostFunction *cost_function = CalibrationEntry::Create(
			point_as_array,
			flange_translation_as_array,
			flange_rotation_as_array
		);
		
		//And then the global parameters to optimize
		problem.AddResidualBlock(cost_function, NULL,
			CAM_to_FLANGE_t, CAM_to_FLANGE_r,
			WRIST_to_POINT_t
		);
	}
	
	//Bound the rotation.
	cc_utils::bound_rotation(problem, CAM_to_FLANGE_r);
	
	//Run the solver!
	options.minimizer_progress_to_stdout = true;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.max_num_iterations = 1000;
	ceres::Solver::Summary summary;
    	ceres::Solve(options, &problem, &summary);
    	
    	//Convert the camera-to-flange transform back into mill-to-camera for easy comparison
    	double FtC_x, FtC_y, FtC_z,	FtC_r, FtC_p, FtC_w;
	Eigen::Affine3d b = cc_utils::invert_eul(
		CAM_to_FLANGE_t[0], CAM_to_FLANGE_t[1], CAM_to_FLANGE_t[2],
		cc_utils::dtor(CAM_to_FLANGE_r[0]), cc_utils::dtor(CAM_to_FLANGE_r[0]), cc_utils::dtor(CAM_to_FLANGE_r[0]),
		
		FtC_x, FtC_y, FtC_z,	FtC_r, FtC_p, FtC_w
	);
    	
    	//Calculate the RMS and display the values:
    	printf("\nCalibration complete.\n");
    	//printf("\e[36mRMS value is \e[35m%f px\e[36m.\n", cc_utils::rms());
    	
    	std::printf("FLANGE to CAMERA:\n");
	std::printf("\tx = \e[35m%f\e[36m\ty = \e[35m%f\e[36m\tz = \e[35m%f\e[36m\n", FtC_x, FtC_y, FtC_z);
	std::printf(
		"\tr = \e[35m%f\e[36m\tp = \e[35m%f\e[36m\tw = \e[35m%f\e[36m\n", 
		FtC_r,
		FtC_p,
		FtC_w
	);
	
	/*std::printf("CAMERA to MILL:\n");
	std::printf("\tx = \e[35m%f\e[36m\ty = \e[35m%f\e[36m\tz = \e[35m%f\e[36m\n", MtC_x, MtC_y, MtC_z);
	std::printf("\tr = \e[35m%f\e[36m\tp = \e[35m%f\e[36m\tw = \e[35m%f\n\n", MtC_r, MtC_p, MtC_w);
	std::printf("\t%f\t%f\t%f\t%f\n", b.matrix()(0, 0), b.matrix()(0, 1), b.matrix()(0, 2), b.matrix()(0, 3));
	std::printf("\t%f\t%f\t%f\t%f\n", b.matrix()(1, 0), b.matrix()(1, 1), b.matrix()(1, 2), b.matrix()(1, 3));
	std::printf("\t%f\t%f\t%f\t%f\n", b.matrix()(2, 0), b.matrix()(2, 1), b.matrix()(2, 2), b.matrix()(2, 3));
	std::printf("\t%f\t%f\t%f\t%f\e[36m\n", b.matrix()(3, 0), b.matrix()(3, 1), b.matrix()(3, 2), b.matrix()(3, 3));*/

	return 0;
}
