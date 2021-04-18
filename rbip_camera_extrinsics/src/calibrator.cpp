#include <fstream>
#include <iostream>
#include <ctime>

#include <boost/filesystem.hpp>

#include <Eigen/Eigen>

#include "yaml-cpp/yaml.h"

#include <cc_utils/cc_utils.h>

//Y dis not a standard function??
std::string ReplaceString(
	std::string subject,
	const std::string& search,
	const std::string& replace
) {
	size_t pos = 0;
	while ((pos = subject.find(search, pos)) != std::string::npos) {
		subject.replace(pos, search.length(), replace);
		pos += replace.length();
	}
	return subject;
}


class CalibrationEntry{
public:
	//Giant stupid constructor chain.
	//The fact that it takes two functions to pass the exact same arguments into the exact same variables is Ceres' fault; not mine.
	CalibrationEntry(
		const double image_pixels_in[2],
		const double target_point_in[3],
		
		const double f_x_in, const double f_y_in, const double c_x_in, const double c_y_in
	){
		image_pixels[0] = image_pixels_in[0];
		image_pixels[1] = image_pixels_in[1];
		
		target_point[0] = target_point_in[0];
		target_point[1] = target_point_in[1];
		target_point[2] = target_point_in[2];
		
		f_x = f_x_in;
		f_y = f_y_in;
		c_x = c_x_in;
		c_y = c_y_in;
	}
	static ceres::CostFunction* Create(
		const double image_pixels_in[2],
		const double target_point_in[3],
		
		const double f_x_in, const double f_y_in, const double c_x_in, const double c_y_in
	){
		return new ceres::AutoDiffCostFunction<CalibrationEntry, 2,//Residual output comes first.
		//	camera translation	camera rotation
			3,			3
		>(new CalibrationEntry (//Just pass all the arguments in in the same order.
			image_pixels_in,
			target_point_in,
		
			f_x_in, f_y_in, c_x_in, c_y_in
		));
	}


	//Constant member vars
	//Perpoints
	double image_pixels[2];
	double target_point[3];
	//Globals (which need to be passed perpoint anyway)
	double f_x, f_y, c_x, c_y;
	
	
	template<typename T> bool operator()(//TODO Why are all these const / should all these be const?
		const T* CAM_to_RBIP_translation, const T* CAM_to_RBIP_rotation,
		
		T* residual
	) const {
		//CAM_to_POINT = CAM_to_FOREARM TARGET_POINT
		
		
		//0: TARGET_to_POINT
		T TARGET_POINT [3] = {
			T(target_point[0]),
			T(target_point[1]),
			T(target_point[2]),
		};
		
		/*std::cout << "TARGET POINT\n";
		std::cout << "\t" << TARGET_POINT[0] << "\n";
		std::cout << "\t" << TARGET_POINT[1] << "\n";
		std::cout << "\t" << TARGET_POINT[2] << "\n";*/
		
		//1: BASE_to_POINT = BASE_to_TARGET * TARGET_to_POINT
		T CAM_to_POINT [3];
		cc_utils::transformPoint_euler(CAM_to_RBIP_translation, CAM_to_RBIP_rotation, TARGET_POINT, CAM_to_POINT);
		
		
		/*std::cout << "CAMERA POINT\n";
		std::cout << "\t" << CAM_to_POINT[0] << "\n";
		std::cout << "\t" << CAM_to_POINT[1] << "\n";
		std::cout << "\t" << CAM_to_POINT[2] << "\n";*/
		
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
		output = NULL;
	} else {
		output = argv[4];
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
	try{
		c_x = guess_file["cam_x"].as<double>();
		c_y = guess_file["cam_y"].as<double>();
		c_z = guess_file["cam_z"].as<double>();
		c_r = guess_file["cam_r"].as<double>();
		c_p = guess_file["cam_p"].as<double>();
		c_w = guess_file["cam_w"].as<double>();
		
	} catch(YAML::RepresentationException e){
		printf("\e[39mGuess parse exception \"%s\".\e[31m\n", e.what());
		return 0;
	}
	printf("\nSuccessfully initialized guesses from %s:\n", argv[2]);
	printf("\tCamera xyz = (%f %f %f)\t\t rpy = (%f %f %f)\n", c_x, c_y, c_z, c_r, c_p, c_w);
	
	
	//Load in calibration data
	std::vector<Eigen::Vector3d>	target_locations;
	std::vector<Eigen::Vector2d>	pixel_locations;
	
	double max_u = 0;
	double max_v = 0;
	
	std::string folder = std::string(argv[1]);
	folder = folder.substr(0, folder.find_last_of("/\\"));
	printf("\nLooking for data in %s.\n\n", folder.c_str());
	
	boost::filesystem::path image_folder_path = boost::filesystem::path(folder);
	for(
		boost::filesystem::directory_entry& x
		:
		boost::filesystem::directory_iterator(image_folder_path)
	){
		if(x.path().native().find("robot") != std::string::npos){
			std::string fname_robot = x.path().native();
			printf("Reading from %s\n", fname_robot.c_str());
			
			std::string fname_common = fname_robot.substr(0, fname_robot.find_last_of("/\\"));
			std::string fname_name = fname_robot.substr(fname_robot.find_last_of("/\\"));
			std::string fname_image = fname_common + ReplaceString(fname_name, "robot", "rect");

			std::ifstream image_file(fname_image);
			if(!image_file){
				printf("\tFound robot file but no image file %s\n", fname_image.c_str());
				continue;
			}
			char line[255];
			std::vector<Eigen::Vector2d> all_px;
			bool badpix = false;
			while(image_file.getline(line, 255)){
				std::string line_s = std::string(line);
				if(std::count(line_s.begin(), line_s.end(), ',') != 2){
					printf("\t\e[39mBad pixel file format. Line %lu (%s).\e[31m\n", all_px.size(), line);
					badpix = true;
					break;
				}
				Eigen::Vector2d pixel;
				double blah;
				sscanf(line, "%lf, %lf, %lf", &blah, &pixel.x(), &pixel.y());
				all_px.push_back(pixel);
			}
			if(badpix){
				continue;
			}
			//for(int i = 0; i < all_px.size();
			
			std::ifstream robot_file(fname_robot);
			while(robot_file.getline(line, 255)){
				std::string line_s = std::string(line);
				if(std::count(line_s.begin(), line_s.end(), ',') != 2){
					printf("\t\e[39mBad robot file format. Line %lu (%s).\e[31m\n", all_px.size(), line);
					break;
				}
				Eigen::Vector3d point;
				point.z() = 0.0;
				int index;
				sscanf(line, "%i, %lf, %lf", &index, &point.x(), &point.y());
				if(index >= all_px.size()){
					printf("\t\e39mPixel index %d is out of range!\n", index);
					break;
				}
				
				pixel_locations.push_back(all_px[index]);
				target_locations.push_back(point);
				
				if(all_px[index].x() > max_u){
					max_u = all_px[index].x();
				}
				if(all_px[index].y() > max_v){
					max_v = all_px[index].y();
				}	
			}
		}
	}
	
	int n = target_locations.size();
	printf("\nIdentified %d points.\n", n);
	
	//Convert the target locations from rbip frame to forearm frame
	/*Eigen::Affine3d lidar_to_forearm;
	lidar_to_forearm.matrix() <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
	;
	for(int i = 0; i < target_locations.size(); i++){
		target_locations[i] = lidar_to_forearm * target_locations[i];
	}*/
	//They do not need to be in the forearm frame.
	
	
	//Build the optimization problem
	ceres::Problem problem;
	ceres::Solver::Options options;
	
	//Camera calibration is most useful externally as SYSREF_to_CAM,
	//but for the math to work here we want CAM_to_SYSREF. So we will convert.
	double CAM_to_RBIP_t [3];// = {c_x, c_y, c_z};
	double CAM_to_RBIP_r [3];// = {cc_utils::rtod(c_r), cc_utils::rtod(c_p), cc_utils::rtod(c_w)};
	
	cc_utils::invert_eul(
		c_x, c_y, c_z, 
		c_r, c_p, c_w, 
		
		CAM_to_RBIP_t[0], CAM_to_RBIP_t[1], CAM_to_RBIP_t[2],
		CAM_to_RBIP_r[0], CAM_to_RBIP_r[1], CAM_to_RBIP_r[2]
	);
	CAM_to_RBIP_r[0] = cc_utils::rtod(CAM_to_RBIP_r[0]);
	CAM_to_RBIP_r[1] = cc_utils::rtod(CAM_to_RBIP_r[1]);
	CAM_to_RBIP_r[2] = cc_utils::rtod(CAM_to_RBIP_r[2]);
	
	//Select only PART of the data, for debug
	//pixel_locations.resize(11);
	//target_locations.resize(11);
	
	
	//Initialize visualization.
	cc_utils::init_visualization(std::ceil(max_u) + 100, std::ceil(max_v) + 100, pixel_locations, options);

	
	//Pack the problem.
	for(int i = 0; i < target_locations.size(); i++){
		double pixels_as_array [2] = { pixel_locations[i].x(), pixel_locations[i].y() };
		
		double target_as_array [3] = { target_locations[i].x(), target_locations[i].y(), target_locations[i].z() };
		
		ceres::CostFunction *cost_function = CalibrationEntry::Create(
			pixels_as_array,
			target_as_array,
			
			fx, fy, cx, cy
		);
		
		
		problem.AddResidualBlock(cost_function, NULL,
			CAM_to_RBIP_t, CAM_to_RBIP_r
		);
	}
	
	
	//Bound the rotations.
	cc_utils::bound_rotation(problem, CAM_to_RBIP_r);
	//TODO What is this for?
	//problem.SetParameterLowerBound(CAM_to_SYSREF_t, 1, -0.1);
	//problem.SetParameterUpperBound(CAM_to_SYSREF_t, 1, 0.1);
	
	
	//Run the solver!
	options.minimizer_progress_to_stdout = true;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.max_num_iterations = 1000;
	ceres::Solver::Summary summary;
    	ceres::Solve(options, &problem, &summary);
    	
    	
    	//Convert the camera-to-forearm transform back into forearm-to-camera for easy comparison
   	double c_x_out, c_y_out, c_z_out, c_r_out, c_p_out, c_w_out;
   	
    	Eigen::Affine3d b = cc_utils::invert_eul(
		CAM_to_RBIP_t[0], CAM_to_RBIP_t[1], CAM_to_RBIP_t[2], 
		cc_utils::dtor(CAM_to_RBIP_r[0]), cc_utils::dtor(CAM_to_RBIP_r[1]), cc_utils::dtor(CAM_to_RBIP_r[2]), 
		
		c_x_out, c_y_out, c_z_out,
		c_r_out, c_p_out, c_w_out
	);
   	
   	//Output goodness-of-fit data
    	printf("\nCalibration complete.\n\n");
    	
    	printf("\e[36mRMS value is \e[35m%f px\e[36m.\n", cc_utils::rms());
    	
   	//Output calibration data.
   	std::printf("\nRBIP to CAMERA:\n");
	std::printf("\tx = \e[35m%f\e[36m\ty = \e[35m%f\e[36m\tz = \e[35m%f\e[36m\n", c_x_out, c_y_out, c_z_out);
	std::printf("\tr = \e[35m%f\e[36m\tp = \e[35m%f\e[36m\tw = \e[35m%f\n\n", c_r_out, c_p_out, c_w_out);
	std::printf("\t%f\t%f\t%f\t%f\n", b.matrix()(0, 0), b.matrix()(0, 1), b.matrix()(0, 2), b.matrix()(0, 3));
	std::printf("\t%f\t%f\t%f\t%f\n", b.matrix()(1, 0), b.matrix()(1, 1), b.matrix()(1, 2), b.matrix()(1, 3));
	std::printf("\t%f\t%f\t%f\t%f\n", b.matrix()(2, 0), b.matrix()(2, 1), b.matrix()(2, 2), b.matrix()(2, 3));
	std::printf("\t%f\t%f\t%f\t%f\e[36m\n\n", b.matrix()(3, 0), b.matrix()(3, 1), b.matrix()(3, 2), b.matrix()(3, 3));
	
	if(output != NULL){

		
		/*Eigen::Affine3d FOREARM_to_SYSREF;
		Eigen::Matrix4d m;
		m <<//SYSREF_to_FOREARM
			 0.9997041,  0.0040474,  0.0239884,	 0,
			-0.0039514,  0.9999840, -0.0040474,	 0.245988,
			-0.0240044,  0.0039514,  0.9997041,	 0,
			 0,		0,		 0,		 1
		;
		FOREARM_to_SYSREF.matrix() = m.inverse();
		
		Eigen::Affine3d FOREARM_to_CAMERA = FOREARM_to_SYSREF * b;*/
   	
		double x_out = b.translation().x();
		double y_out = b.translation().y();
		double z_out = b.translation().z();
		
		Eigen::Quaterniond q = (Eigen::Quaterniond)b.rotation();
		
		std::string lf_string = "<launch>\n\t<node\n\t\tpkg=\"tf\"\n\t\t";
		lf_string += "type=\"static_transform_publisher\"\n\t\t";
		lf_string += "name=\"camera_optical_frame_pub\"\n\t\targs=\"";
		lf_string += std::to_string(x_out) + " " + std::to_string(y_out) + " " + std::to_string(z_out) + " ";
		lf_string += std::to_string(q.x()) + " " + std::to_string(q.y()) + " " + std::to_string(q.z()) + " " + std::to_string(q.w()) + " ";
		lf_string += "RBIP_frame camera_optical_frame 100\" \n\t/>\n</launch>";
		
		printf("Writing launch file \n\n%s\n\n", lf_string.c_str());
		
		std::ofstream master_file(std::string(output) + ".launch");
		if(!master_file){
			printf("Could not write to %s\n", (std::string(output) + ".launch").c_str());
			master_file.close();
		} else{
			master_file << lf_string;
			master_file.close();
			printf("Wrote to %s\n", (std::string(output) + ".launch").c_str());
		}
		
		time_t tsec = time(0);
		tm* tword = localtime(&tsec);
		
		std::string t_string = std::string(output) + "_"
			+std::to_string(tword->tm_mday)+"-"+std::to_string(tword->tm_mon+1)+"-"+std::to_string(tword->tm_year+1900)+"_"
			+std::to_string(tword->tm_hour)+":"+std::to_string(tword->tm_min)+":"+std::to_string(tword->tm_sec)+".launch"
		;
		std::ofstream timed_file(t_string);
		if(!timed_file){
			printf("Could not write to %s\n", t_string.c_str());
			timed_file.close();
		} else{
			timed_file << lf_string;
			timed_file.close();
			printf("Wrote to %s\n", t_string.c_str());
		}
	}
   	
	return 0;
}
