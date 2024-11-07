#include <fstream>
#include <iostream>
#include <math.h>
#include <unistd.h>

#include <boost/lexical_cast.hpp>

#include <Eigen/Eigen>

#include "yaml-cpp/yaml.h"

#include <cc_utils/cc_utils.h>

//TODO There has GOT to be a simpler and less brittle way to do this.
std::string mat_to_linear(const Eigen::MatrixXd & m){
	std::string s = "";
	try{//LexicalCast because we want to keep full precision.
		 s = s + boost::lexical_cast<std::string>(m(0, 0));
		 for(int i = 0; i < 4; i++){
			  for(int j = 0; j < 4; j++){
				   if(i != 0 || j != 0){
					    s = s + ", " + boost::lexical_cast<std::string>(m(i, j));
				   }
			  }
		 }
	} catch (boost::bad_lexical_cast const& e){
		 return "";//Not sure if this can ever happen.
	}
	return s;
}


int cnt;
	
std::vector<double> ul_calc;
std::vector<double> vl_calc;
std::vector<double> ul_real;
std::vector<double> vl_real;
cv::Mat debug_mat_l;
cv::Mat the_ground_truth_l;

std::vector<double> ur_calc;
std::vector<double> vr_calc;
std::vector<double> ur_real;
std::vector<double> vr_real;
cv::Mat debug_mat_r;
cv::Mat the_ground_truth_r;

class VisualCallback : public ceres::IterationCallback {
	public:
		//TODO Are either of these necessary??
		explicit VisualCallback(){}
		~VisualCallback() {}
	
		ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
			cv::namedWindow("Iteration Projection (Left)", cv::WINDOW_GUI_EXPANDED | cv::WINDOW_NORMAL);
			cv::imshow("Iteration Projection (Left)", debug_mat_l);
			
			cv::namedWindow("Iteration Projection (Right)", cv::WINDOW_GUI_EXPANDED | cv::WINDOW_NORMAL);
			cv::imshow("Iteration Projection (Right)", debug_mat_r);
			cv::waitKey(500);
			//cv::imwrite("/home/tes77/dbg_img_" + std::to_string(cnt) + ".png", debug_mat);
			cnt++;
			//cv::waitKey();
			
			ul_calc.clear();
			ul_real.clear();
			vl_calc.clear();
			vl_real.clear();
			
			ur_calc.clear();
			ur_real.clear();
			vr_calc.clear();
			vr_real.clear();
		
			the_ground_truth_l.copyTo(debug_mat_l);
			the_ground_truth_r.copyTo(debug_mat_r);
			return ceres::SOLVER_CONTINUE;
		}
};


class CalibrationEntry{
public:
	//Giant stupid constructor chain.
	//The fact that it takes two functions to pass the exact same arguments into the exact same variables is Ceres' fault; not mine.
	CalibrationEntry(
		const double l_pixels_in[2], const double r_pixels_in[2],
		const double MILL_to_SLED_translation_in[3],
		const double TARGET_to_POINT_translation_in[3],
		
		double fx_l_in, double fy_l_in, double cx_l_in, double cy_l_in,
		double dist_l_in [5],
		double fx_r_in, double fy_r_in, double cx_r_in, double cy_r_in,
		double dist_r_in [5],
		
		const int r_x_in, const int r_y_in
	){
		l_pixels[0] = l_pixels_in[0];
		l_pixels[1] = l_pixels_in[1];
		r_pixels[0] = r_pixels_in[0];
		r_pixels[1] = r_pixels_in[1];
		
		MILL_to_SLED_translation[0] = MILL_to_SLED_translation_in[0];
		MILL_to_SLED_translation[1] = MILL_to_SLED_translation_in[1];
		MILL_to_SLED_translation[2] = MILL_to_SLED_translation_in[2];
		
		TARGET_to_POINT_translation[0] = TARGET_to_POINT_translation_in[0];
		TARGET_to_POINT_translation[1] = TARGET_to_POINT_translation_in[1];
		TARGET_to_POINT_translation[2] = TARGET_to_POINT_translation_in[2];
		
		fx_l = fx_l_in;
		fy_l = fy_l_in;
		cx_l = cx_l_in;
		cy_l = cy_l_in;
		
		fx_r = fx_r_in;
		fy_r = fy_r_in;
		cx_r = cx_r_in;
		cy_r = cy_r_in;
		
		for(int n = 0; n < 5; n++){
			dist_l[n] = dist_l_in[n];
			dist_r[n] = dist_r_in[n];
		}
		
		r_x = r_x_in;
		r_y = r_y_in;
	}
	static ceres::CostFunction* Create(
		const double l_pixels_in[2], const double r_pixels_in[2],
		const double MILL_to_SLED_translation_in[3],
		const double TARGET_to_POINT_translation_in[3],
		
		double fx_l_in, double fy_l_in, double cx_l_in, double cy_l_in,
		double dist_l_in [5],
		double fx_r_in, double fy_r_in, double cx_r_in, double cy_r_in,
		double dist_r_in [5],
		
		const int r_x_in, const int r_y_in
		
	){
		return new ceres::AutoDiffCostFunction<CalibrationEntry, 4,//Residual output comes first.
		//	target rotation	base translation	base rotation	IO translation	IO rotation
			3,			3,			3,		3,		3
		>(new CalibrationEntry (//Just pass all the arguments in in the same order.
			l_pixels_in, r_pixels_in, MILL_to_SLED_translation_in, TARGET_to_POINT_translation_in,
			
			fx_l_in, fy_l_in, cx_l_in, cy_l_in,
			dist_l_in,
			fx_r_in, fy_r_in, cx_r_in, cy_r_in,
			dist_r_in,
			
			r_x_in, r_y_in
		));
	}
	
	
	//Constant member vars
	//Perpoints
	double l_pixels[2];
	double r_pixels[2];
	double MILL_to_SLED_translation[3];
	double TARGET_to_POINT_translation[3];
	//Globals
	int r_x, r_y;
	
	double fx_l, fy_l, cx_l, cy_l;
	double dist_l [5];
	
	double fx_r, fy_r, cx_r, cy_r;
	double dist_r [5];
	
	template<typename T> bool operator()(//TODO Why are all these const / should all these be const?
		const T* SLED_to_TARGET_rotation,
		const T* CAM_to_MILL_translation, const T* CAM_to_MILL_rotation,
		const T* L_to_R_translation, const T* L_to_R_rotation,
		
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
		
		T R_to_POINT [3];
		cc_utils::transformPoint_euler(L_to_R_translation, L_to_R_rotation, CAM_to_POINT, R_to_POINT);
		
		T u_l, v_l, u_r, v_r;
		cc_utils::project(
			CAM_to_POINT[0], CAM_to_POINT[1], CAM_to_POINT[2],
			
			T(fx_l), T(fy_l), T(cx_l), T(cy_l),
			T(dist_l[0]), T(dist_l[1]), T(dist_l[2]), T(dist_l[3]), T(dist_l[4]),
		
			T(r_x), T(r_y),
		
			u_l, v_l
		);
		cc_utils::project(
			R_to_POINT[0], R_to_POINT[1], R_to_POINT[2],
			
			T(fx_r), T(fy_r), T(cx_r), T(cy_r),
			T(dist_r[0]), T(dist_r[1]), T(dist_r[2]), T(dist_r[3]), T(dist_r[4]),
		
			T(r_x), T(r_y),
		
			u_r, v_r
		);
		
		//printf("Real u, v: (%f, %f)\n", image_pixels[0], image_pixels[1]);
		ul_calc.push_back(cc_utils::val(u_l));
		vl_calc.push_back(cc_utils::val(v_l));
		ul_real.push_back(l_pixels[0]);
		vl_real.push_back(l_pixels[1]);
		cv::drawMarker(debug_mat_l, cv::Point(cc_utils::val(u_l), cc_utils::val(v_l)), cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 5);
		
		ur_calc.push_back(cc_utils::val(u_r));
		vr_calc.push_back(cc_utils::val(v_r));
		ur_real.push_back(r_pixels[0]);
		vr_real.push_back(r_pixels[1]);
		cv::drawMarker(debug_mat_r, cv::Point(cc_utils::val(u_r), cc_utils::val(v_r)), cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 5);
		
		//std::getchar();
		
		residual[0] = u_l - T(l_pixels[0]);
		residual[1] = v_l - T(l_pixels[1]);
		//TODO Unduplicate this
		residual[2] = u_r - T(r_pixels[0]);
		residual[3] = v_r - T(r_pixels[1]);
		
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
	double fx_l, fy_l, cx_l, cy_l;
	double k1_l, k2_l, k3_l, p1_l, p2_l;
	try{
		fx_l = intrinsic_file["fx"].as<double>();
		fy_l = intrinsic_file["fy"].as<double>();
		cx_l = intrinsic_file["cx"].as<double>();//These two values are not used (replaced by the calculation of c_fixed), but I am keeping them around for consistency with the base files.
		cy_l = intrinsic_file["cy"].as<double>();
		
		k1_l = intrinsic_file["k1"].as<double>();
		k2_l = intrinsic_file["k2"].as<double>();
		k3_l = intrinsic_file["k3"].as<double>();
		p1_l = intrinsic_file["p1"].as<double>();
		p2_l = intrinsic_file["p2"].as<double>();
	} catch(YAML::Exception e){
		printf("\e[39mIntrinsic parse exception \"%s\".\e[31m\n", e.what());
		return 0;
	}
	printf("Successfully initialized left intrinsics from %s.\n", argv[3]);
	
	try{
		intrinsic_file = YAML::LoadFile(argv[6]);
	} catch(YAML::BadFile e){//If file is not extant and well-formed...
		printf("\e[39mInitial intrinsic file \"%s\" does not exist or contains syntax errors.\e[31m\n", argv[6]);
		return 0;
	}
	double fx_r, fy_r, cx_r, cy_r;
	double k1_r, k2_r, k3_r, p1_r, p2_r;
	try{
		fx_r = intrinsic_file["fx"].as<double>();
		fy_r = intrinsic_file["fy"].as<double>();
		cx_r = intrinsic_file["cx"].as<double>();//These two values are not used (replaced by the calculation of c_fixed), but I am keeping them around for consistency with the base files.
		cy_r = intrinsic_file["cy"].as<double>();
		
		k1_r = intrinsic_file["k1"].as<double>();
		k2_r = intrinsic_file["k2"].as<double>();
		k3_r = intrinsic_file["k3"].as<double>();
		p1_r = intrinsic_file["p1"].as<double>();
		p2_r = intrinsic_file["p2"].as<double>();
	} catch(YAML::Exception e){
		printf("\e[39mIntrinsic parse exception \"%s\".\e[31m\n", e.what());
		return 0;
	}
	printf("Successfully initialized left intrinsics from %s.\n", argv[6]);
	
	//Read in data
	
	//Left
	std::ifstream data_file;
	data_file.open(argv[1]);
	if(!data_file){
		printf("\e[39mCould not find data file \"%s\".\e[31m\n", argv[1]);
		return 0;
	}
	
	std::vector<Eigen::Vector2d> pixels_l;
	std::vector<Eigen::Vector2d> target_coordinates_l;
	std::vector<Eigen::Vector3d> mill_coordinates_l;
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
		
		pixels_l.push_back(px_entry);
		target_coordinates_l.push_back(target_entry);
		mill_coordinates_l.push_back(mill_entry);
	}
	printf("Read in \e[1m%d\e[0m entries from %s.\n\n", n, argv[1]);
	data_file.close();
	
	//Right
	data_file.open(argv[4]);
	if(!data_file){
		printf("\e[39mCould not find data file \"%s\".\e[31m\n", argv[4]);
		return 0;
	}
	std::vector<Eigen::Vector2d> pixels_r;
	std::vector<Eigen::Vector2d> target_coordinates_r;
	std::vector<Eigen::Vector3d> mill_coordinates_r;
	n = 0;
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
		
		pixels_r.push_back(px_entry);
		target_coordinates_r.push_back(target_entry);
		mill_coordinates_r.push_back(mill_entry);
	}
	printf("Read in \e[1m%d\e[0m entries from %s.\n\n", n, argv[4]);
	
	//Sort
	std::vector<Eigen::Vector2d> pixels_l_sorted;
	std::vector<Eigen::Vector2d> pixels_r_sorted;
	std::vector<Eigen::Vector2d> target_coordinates_sorted;
	std::vector<Eigen::Vector3d> mill_coordinates_sorted;
	for(int l = 0; l < pixels_l.size(); l++){
		for(int r = 0; r < pixels_r.size(); r++){
			if(
				target_coordinates_r[r][0] == target_coordinates_l[l][0] &&
				target_coordinates_r[r][1] == target_coordinates_l[l][1] &&
				mill_coordinates_r[r][0] == mill_coordinates_l[l][0] &&
				mill_coordinates_r[r][1] == mill_coordinates_l[l][1] &&
				mill_coordinates_r[r][2] == mill_coordinates_l[l][2]
			){
				target_coordinates_sorted.push_back(target_coordinates_l[l]);
				mill_coordinates_sorted.push_back(mill_coordinates_l[l]);
				pixels_l_sorted.push_back(pixels_l[l]);
				pixels_r_sorted.push_back(pixels_r[r]);
				break;
			}
		}
	}
	
	printf("Produced \e[1m%lu\e[0m sorted pairs\n\n", pixels_l_sorted.size());
	
	//Build the optimization problem
	ceres::Problem problem;
	ceres::Solver::Options options;
	
	//Set up visualization
	debug_mat_l = cv::Mat(480, 640, CV_8UC3);
	the_ground_truth_l = cv::Mat(480, 640, CV_8UC3);
	debug_mat_r = cv::Mat(480, 640, CV_8UC3);
	the_ground_truth_r = cv::Mat(480, 640, CV_8UC3);
	cnt = 0;
	
	for(int i = 0; i < pixels_l_sorted.size(); i++){
		cv::drawMarker(
			the_ground_truth_l,
			cv::Point(pixels_l_sorted[i][0], pixels_l_sorted[i][1]),
			cv::Scalar(255, 0, 0),//Marker color
			cv::MARKER_TILTED_CROSS, 5//Type and size.
		);
	}
	for(int i = 0; i < pixels_r_sorted.size(); i++){
		cv::drawMarker(
			the_ground_truth_r,
			cv::Point(pixels_r_sorted[i][0], pixels_r_sorted[i][1]),
			cv::Scalar(255, 0, 0),//Marker color
			cv::MARKER_TILTED_CROSS, 5//Type and size.
		);
	}
	the_ground_truth_l.copyTo(debug_mat_l);
	the_ground_truth_r.copyTo(debug_mat_r);
		
	options.callbacks.push_back(new VisualCallback());
	options.update_state_every_iteration = true;
	
	//Initialize the unknown values from their defaults.
	double SLED_to_TARGET_r [3] = {cc_utils::rtod(trg_init_r), cc_utils::rtod(trg_init_p), cc_utils::rtod(trg_init_w)};
	double CAM_to_MILL_t [3] = {MtC_init_x, MtC_init_y, MtC_init_z};
	double CAM_to_MILL_r [3] = {cc_utils::rtod(MtC_init_r), cc_utils::rtod(MtC_init_p), cc_utils::rtod(MtC_init_w)};
	double L_to_R_t [3] = {0.0, 0.0, -0.005};
	double L_to_R_r [3] = {0.0, 0.0, 0.0};
	
	double distortion_l[5] = {k1_l, k2_l, k3_l, p1_l, p2_l};
	double distortion_r[5] = {k1_r, k2_r, k3_r, p1_r, p2_r};
	
	for(int i = 0; i < pixels_l_sorted.size(); i++){
		//Add the perpoint constants.
		double pixels_array_l[2] = {pixels_l_sorted[i].x(), pixels_l_sorted[i].y()};
		double pixels_array_r[2] = {pixels_r_sorted[i].x(), pixels_r_sorted[i].y()};
		double mill_array[3] = {mill_coordinates_sorted[i].x(), mill_coordinates_sorted[i].y(), mill_coordinates_sorted[i].z()};
		double target_array[3] = {target_coordinates_sorted[i].x(), target_coordinates_sorted[i].y(), 0};
		
		ceres::CostFunction *cost_function = CalibrationEntry::Create(
			pixels_array_l, pixels_array_r,
			mill_array,
			target_array,
			
			fx_l, fy_l, cx_l, cy_l,
			distortion_l,
			fx_r, fy_r, cx_r, cy_r,
			distortion_r,
			
			//And the universal constants.
			resolution_y, resolution_x
		);
		
		//And then the global parameters to optimize
		problem.AddResidualBlock(cost_function, NULL,
			SLED_to_TARGET_r,
			CAM_to_MILL_t, CAM_to_MILL_r,
			L_to_R_t, L_to_R_r
		);
	}
	
	//Bound the two rotations.
	cc_utils::bound_rotation(problem, SLED_to_TARGET_r);
	cc_utils::bound_rotation(problem, CAM_to_MILL_r);
	cc_utils::bound_rotation(problem, L_to_R_r);
	
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
    	//Find another way to calculate the RMS
    	/*double rms = cc_utils::rms();
    	printf("\e[36mRMS value is \e[35m%f px\e[36m.\n", rms);
    	
    	if(std::isnan(rms)){
    		printf("Solver did not converge. Won't save results.\n");
    		return 0;
    	}*/
    	
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
	
	printf("In long form that is %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n\n\n",
		b.matrix()(0, 0), b.matrix()(0, 1), b.matrix()(0, 2), b.matrix()(0, 3),
		b.matrix()(1, 0), b.matrix()(1, 1), b.matrix()(1, 2), b.matrix()(1, 3),
		b.matrix()(2, 0), b.matrix()(2, 1), b.matrix()(2, 2), b.matrix()(2, 3),
		b.matrix()(3, 0), b.matrix()(3, 1), b.matrix()(3, 2), b.matrix()(3, 3)
	);
	
	std::printf("LEFT to RIGHT:\n");
	double LtR_x = L_to_R_t[0];
	double LtR_y = L_to_R_t[1];
	double LtR_z = L_to_R_t[2];
	double LtR_r = cc_utils::dtor(L_to_R_r[0]);
	double LtR_p = cc_utils::dtor(L_to_R_r[1]);
	double LtR_w = cc_utils::dtor(L_to_R_r[2]);
	
	Eigen::Affine3d c;
	c =
		Eigen::AngleAxisd(LtR_w, Eigen::Vector3d::UnitZ()) *
   		Eigen::AngleAxisd(LtR_p, Eigen::Vector3d::UnitY()) *
   		Eigen::AngleAxisd(LtR_r, Eigen::Vector3d::UnitX());
   	c.translation() = Eigen::Vector3d(LtR_x, LtR_y, LtR_z);
   	
   	std::printf("\tx = \e[35m%f\e[36m\ty = \e[35m%f\e[36m\tz = \e[35m%f\e[36m\n", LtR_x, LtR_y, LtR_z);
	std::printf("\tr = \e[35m%f\e[36m\tp = \e[35m%f\e[36m\tw = \e[35m%f\n\n", LtR_r, LtR_p, LtR_w);
	std::printf("\t%f\t%f\t%f\t%f\n", c.matrix()(0, 0), c.matrix()(0, 1), c.matrix()(0, 2), c.matrix()(0, 3));
	std::printf("\t%f\t%f\t%f\t%f\n", c.matrix()(1, 0), c.matrix()(1, 1), c.matrix()(1, 2), c.matrix()(1, 3));
	std::printf("\t%f\t%f\t%f\t%f\n", c.matrix()(2, 0), c.matrix()(2, 1), c.matrix()(2, 2), c.matrix()(2, 3));
	std::printf("\t%f\t%f\t%f\t%f\e[36m\n", c.matrix()(3, 0), c.matrix()(3, 1), c.matrix()(3, 2), c.matrix()(3, 3));
	
	printf("In long form that is %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
		c.matrix()(0, 0), c.matrix()(0, 1), c.matrix()(0, 2), c.matrix()(0, 3),
		c.matrix()(1, 0), c.matrix()(1, 1), c.matrix()(1, 2), c.matrix()(1, 3),
		c.matrix()(2, 0), c.matrix()(2, 1), c.matrix()(2, 2), c.matrix()(2, 3),
		c.matrix()(3, 0), c.matrix()(3, 1), c.matrix()(3, 2), c.matrix()(3, 3)
	);
	
	std::ofstream fout = std::ofstream(argv[7]);
	fout << "matrix: [" << mat_to_linear(c.matrix()) << "]\n";
	fout << "ros_x: " << c.translation().x() << "\n";
	fout << "ros_y: " << c.translation().y() << "\n";
	fout << "ros_z: " << c.translation().z() << "\n";
	fout << "ros_r: " << LtR_r << "\n";
	fout << "ros_p: " << LtR_p << "\n";
	fout << "ros_w: " << LtR_w << "\n";
	fout.close();

	printf("Successfully wrote interocular data to %s\n", argv[7]);
	
	/*
	
	if(position != NULL){
		std::ofstream fout = std::ofstream(position);
		fout << "translation: [" << MtC_x << ", " << MtC_y << ", " << MtC_z << "]\n";
		fout << "rpy: [" << MtC_r << ", " << MtC_p << ", " << MtC_w << "]\n";
		fout << "matrix: [" <<
			b.matrix()(0, 0) << ", " << b.matrix()(0, 1) << ", " << b.matrix()(0, 2) << ", " << b.matrix()(0, 3) << ", " <<
			b.matrix()(1, 0) << ", " << b.matrix()(1, 1) << ", " << b.matrix()(1, 2) << ", " << b.matrix()(1, 3) << ", " <<
			b.matrix()(2, 0) << ", " << b.matrix()(2, 1) << ", " << b.matrix()(2, 2) << ", " << b.matrix()(2, 3) << ", " <<
			b.matrix()(3, 0) << ", " << b.matrix()(3, 1) << ", " << b.matrix()(3, 2) << ", " << b.matrix()(3, 3) << "]\n"
		;
		
		//Same format as came in:
		fout << "mill_to_camera_x: " << MtC_x << "\n";
		fout << "mill_to_camera_y: " << MtC_y << "\n";
		fout << "mill_to_camera_z: " << MtC_z << "\n";
		
		fout << "mill_to_camera_r: " << MtC_r << "\n";
		fout << "mill_to_camera_p: " << MtC_p << "\n";
		fout << "mill_to_camera_w: " << MtC_w << "\n";
		
		
		
		fout << "sled_to_target_r: " << cc_utils::dtor(SLED_to_TARGET_r[0]) << "\n";
		fout << "sled_to_target_p: " << cc_utils::dtor(SLED_to_TARGET_r[1]) << "\n";
		fout << "sled_to_target_w: " << cc_utils::dtor(SLED_to_TARGET_r[2]) << "\n";
		
		fout << "resolution_u: " << resolution_x << "\n";
		fout << "resolution_v: " << resolution_y << "\n";
		fout.close();
	}*/

	return 0;
}
