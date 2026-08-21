#include <iostream>
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Eigen>
#include <opencv2/calib3d.hpp>

#define LCONST 5
#define RCONST 26

std::string mat_to_linear(const cv::Mat & m){
	std::string s = "";
	try{//LexicalCast because we want to keep full precision.
		s = s + boost::lexical_cast<std::string>(m.at<double>(0, 0));
		for(int i = 0; i < m.rows; i++){
			for(int j = 0; j < m.cols; j++){
				if(i != 0 || j != 0){
					s = s + ", " + boost::lexical_cast<std::string>(m.at<double>(i, j));
				}
			}
		}
	} catch (boost::bad_lexical_cast const& e){
		return "";//Not sure if this can ever happen.
	}
	return s;
}

std::string mat_to_linear(const Eigen::Matrix4d & m){
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

void mat_to_rpw(Eigen::Vector3d & rpw, const Eigen::Affine3d & mat){//TODO Apparently this conversion exists NOWHERE ELSE IN OUR CODEBASE...
	Eigen::Matrix3d r = mat.linear();
	if(r(2,0) != 1.0 && r(2,0) != -1.0){
		rpw.y() = -asin(r(2,0));
		rpw.x() = atan2(r(2,1) / cos(rpw.y()), r(2,2) / cos(rpw.y()));
		rpw.z() = atan2(r(1,0) / cos(rpw.y()), r(0,0) / cos(rpw.y()));
	} else{
		rpw.z() = 0.0;
		if(r(2,0) == -1.0){
			rpw.y() = M_PI / 2.0;
			rpw.x() = atan2(r(0,1), r(0,2));
		} else{
			rpw.y() = -M_PI / 2.0;
			rpw.x() = atan2(-r(0,1), -r(0,2));
		}
	}
}

int main(int argc, char** argv){
	YAML::Node l_camera;
	YAML::Node r_camera;
	printf("Loading right intrinsic file %s\n", argv[3]);
	r_camera = YAML::LoadFile(argv[3]);
	printf("Loading left intrinsic file %s\n", argv[1]);
	l_camera = YAML::LoadFile(argv[1]);
	
	
	double fx_l, fy_l, cx_l, cy_l;
	double k1_l, k2_l, k3_l, p1_l, p2_l;
	double fx_r, fy_r, cx_r, cy_r;
	double k1_r, k2_r, k3_r, p1_r, p2_r;
	try{
		fx_l = l_camera["camera_matrix"]["data"][0].as<double>();
		fy_l = l_camera["camera_matrix"]["data"][4].as<double>();
		cx_l = l_camera["camera_matrix"]["data"][2].as<double>();
		cy_l = l_camera["camera_matrix"]["data"][5].as<double>();
		
		k1_l = l_camera["distortion_coefficients"]["data"][0].as<double>();
		k2_l = l_camera["distortion_coefficients"]["data"][1].as<double>();
		k3_l = l_camera["distortion_coefficients"]["data"][4].as<double>();
		p1_l = l_camera["distortion_coefficients"]["data"][2].as<double>();
		p2_l = l_camera["distortion_coefficients"]["data"][3].as<double>();
		
	} catch(YAML::RepresentationException e){
		printf("\e[39mLeft Intrinsic parse exception \"%s\".\e[31m\n", e.what());
		return 0;
	}
	printf("Successfully initialized left intrinsics from %s.\n", argv[1]);
	
	try{
		fx_r = r_camera["camera_matrix"]["data"][0].as<double>();
		fy_r = r_camera["camera_matrix"]["data"][4].as<double>();
		cx_r = r_camera["camera_matrix"]["data"][2].as<double>();
		cy_r = r_camera["camera_matrix"]["data"][5].as<double>();
		
		k1_r = r_camera["distortion_coefficients"]["data"][0].as<double>();
		k2_r = r_camera["distortion_coefficients"]["data"][1].as<double>();
		k3_r = r_camera["distortion_coefficients"]["data"][4].as<double>();
		p1_r = r_camera["distortion_coefficients"]["data"][2].as<double>();
		p2_r = r_camera["distortion_coefficients"]["data"][3].as<double>();
		
	} catch(YAML::RepresentationException e){
		printf("\e[39mRight Intrinsic parse exception \"%s\".\e[31m\n", e.what());
		return 0;
	}
	printf("Successfully initialized right intrinsics from %s.\n", argv[1]);
	
	cv::Mat INTRENSIC_L = (cv::Mat_<double>(3, 3) <<
		fx_l, 0.0, cx_l,
		0.0, fy_l, cy_l, 
		0.0,0.0, 1.0
	);
	std::cout << "Left intrinsics: \n" << INTRENSIC_L << "\n";
	
	cv::Mat INTRENSIC_R = (cv::Mat_<double>(3, 3) <<
		fx_r, 0.0, cx_r,
		0.0, fy_r, cy_r, 
		0.0,0.0, 1.0
	);
	std::cout << "Right intrinsics: \n" << INTRENSIC_R << "\n";
	
	cv::Mat DISTORTION_L = (cv::Mat_<double>(1, 5) <<
		k1_l, k2_l, p1_l, p2_l, k3_l
	);
	std::cout << "Left distortion: \n" << DISTORTION_L << "\n";
	
	cv::Mat DISTORTION_R = (cv::Mat_<double>(1, 5) <<
		k1_r, k2_r, p1_r, p2_r, k3_r
	);
	std::cout << "Right distortion: \n" << DISTORTION_R << "\n";
	
	YAML::Node l_position;
	l_position = YAML::LoadFile(argv[2]);
	Eigen::Affine3d L_to_SLED;
	L_to_SLED.matrix() <<
		l_position["matrix"][ 0].as<double>(), l_position["matrix"][ 1].as<double>(), l_position["matrix"][ 2].as<double>(), l_position["matrix"][ 3].as<double>(),
		l_position["matrix"][ 4].as<double>(), l_position["matrix"][ 5].as<double>(), l_position["matrix"][ 6].as<double>(), l_position["matrix"][ 7].as<double>(),
		l_position["matrix"][ 8].as<double>(), l_position["matrix"][ 9].as<double>(), l_position["matrix"][10].as<double>(), l_position["matrix"][11].as<double>(),
		l_position["matrix"][12].as<double>(), l_position["matrix"][13].as<double>(), l_position["matrix"][14].as<double>(), l_position["matrix"][15].as<double>()
	;
	
	printf("\n\nLeft camera to mill:\n");
	std::cout << L_to_SLED.matrix();
	
	
	YAML::Node r_position;
	r_position = YAML::LoadFile(argv[4]);
	Eigen::Affine3d R_to_SLED;
	R_to_SLED.matrix() <<
		r_position["matrix"][ 0].as<double>(), r_position["matrix"][ 1].as<double>(), r_position["matrix"][ 2].as<double>(), r_position["matrix"][ 3].as<double>(),
		r_position["matrix"][ 4].as<double>(), r_position["matrix"][ 5].as<double>(), r_position["matrix"][ 6].as<double>(), r_position["matrix"][ 7].as<double>(),
		r_position["matrix"][ 8].as<double>(), r_position["matrix"][ 9].as<double>(), r_position["matrix"][10].as<double>(), r_position["matrix"][11].as<double>(),
		r_position["matrix"][12].as<double>(), r_position["matrix"][13].as<double>(), r_position["matrix"][14].as<double>(), r_position["matrix"][15].as<double>()
	;
	
	printf("\nRight camera to mill:\n");
	std::cout << R_to_SLED.matrix();
	std::cout << "\n";
	
	Eigen::Affine3d L_to_R = L_to_SLED.inverse() * R_to_SLED;
	printf("\n\n\nLEFT TO RIGHT INTEROCULAR TRANSFORM IS \n\n");
	std::cout << L_to_R.matrix();
	printf("\n\n");
	
	Eigen::Vector3d rpw;
	mat_to_rpw(rpw, L_to_R);
	printf("In ROS format that is xyz='%f %f %f' rpy='%f %f %f'\n",
		L_to_R.translation().x(), L_to_R.translation().y(), L_to_R.translation().z(),
		rpw.x(), rpw.y(), rpw.z()
	);
	
	//TODO: Add rectification levels.
	cv::Mat RECTIFICATION_L;
	cv::Mat RECTIFICATION_R;
	cv::Mat PROJECTION_L;
	cv::Mat PROJECTION_R;
	
	RECTIFICATION_L = (cv::Mat_<double>(3, 3) << 1., 0., 0., 0., 1., 0., 0., 0., 1.);
	RECTIFICATION_R = (cv::Mat_<double>(3, 3) << 1., 0., 0., 0., 1., 0., 0., 0., 1.);
	
	PROJECTION_L = (cv::Mat_<double>(3, 4) << INTRENSIC_L.at<double>(0), 0., INTRENSIC_L.at<double>(2), 0., 0., INTRENSIC_L.at<double>(4), INTRENSIC_L.at<double>(5), 0., 0., 0., 1., 0.);
	PROJECTION_R = (cv::Mat_<double>(3, 4) << INTRENSIC_R.at<double>(0), 0., INTRENSIC_R.at<double>(2), 0., 0., INTRENSIC_R.at<double>(4), INTRENSIC_R.at<double>(5), 0., 0., 0., 1., 0.);
	
	
	/*cv::Mat Q;//Not used.
	cv::Size_<int> roi = cv::Size_<int>(640, 480);
	cv::Rect roi_1;
	cv::Rect roi_2;
	cv::Size nis = cv::Size_<int>(640, 480);
	
	cv::stereoRectify(
		INTRENSIC_L,
		DISTORTION_L,
		INTRENSIC_R,
		DISTORTION_R,
		roi,
		R,
		T,
		RECTIFICATION_L,
		RECTIFICATION_R,
		PROJECTION_L,
		PROJECTION_R,
		Q,
		//0,
		cv::CALIB_ZERO_DISPARITY,
		1.0,
		nis,
		&roi_1,
		&roi_2
	);
	
	//if(resize){
		/*double scale_factor_LX = 640.0 / roi_1.height;
		double scale_factor_LY = 480.0 / roi_1.height;
		double scale_factor_RX = 640.0 / roi_2.height;
		double scale_factor_RY = 480.0 / roi_2.height;
	
		double scale_factor_max = std::max(
			scale_factor_LX,
			std::max(
				scale_factor_LY,
				std::max(
					scale_factor_RX, scale_factor_RY
				)
			)
		);
	
		nis = cv::Size_<int>(640 * scale_factor_max, 480 * scale_factor_max);
	
		cv::stereoRectify(
			INTRENSIC_L,
			DISTORTION_L,
			INTRENSIC_R,
			DISTORTION_R,
			roi,
			R,
			T,
			RECTIFICATION_L,
			RECTIFICATION_R,
			PROJECTION_L,
			PROJECTION_R,
			Q,
			//0,
			cv::CALIB_ZERO_DISPARITY,
			1.0,
			nis
		);*/
	//}*/
	
	std::ofstream fout;
	fout = std::ofstream(argv[5]);
	fout << "image_width: 640\n";//TODO Parametrize this
	fout << "image_height: 480\n";//TODO Parametrize this
	fout << "camera_name: endo_cam_l\n";//TODO Add parametrizable names.
	fout << "camera_matrix:\n";
	fout << "  rows: 3\n";
	fout << "  cols: 3\n";
	fout << "  data: [" << mat_to_linear(INTRENSIC_L) << "]\n";
	fout << "distortion_model: plumb_bob\n";
	fout << "distortion_coefficients:\n";
	fout << "  rows: 1\n";
	fout << "  cols: 5\n";
	fout << "  data: [" << mat_to_linear(DISTORTION_L) << "]\n";
	fout << "rectification_matrix:\n";
	fout << "  rows: 3\n";
	fout << "  cols: 3\n";
	fout << "  data: [" << mat_to_linear(RECTIFICATION_L) << "]\n";
	fout << "projection_matrix:\n";
	fout << "  rows: 3\n";
	fout << "  cols: 4\n";
	fout << "  data: [" << mat_to_linear(PROJECTION_L) << "]\n";
	fout.close();
	
	fout = std::ofstream(argv[6]);
	fout << "image_width: 640\n";//TODO Parametrize this
	fout << "image_height: 480\n";//TODO Parametrize this
	fout << "camera_name: endo_cam_l\n";//TODO Add parametrizable names.
	fout << "camera_matrix:\n";
	fout << "  rows: 3\n";
	fout << "  cols: 3\n";
	fout << "  data: [" << mat_to_linear(INTRENSIC_R) << "]\n";
	fout << "distortion_model: plumb_bob\n";
	fout << "distortion_coefficients:\n";
	fout << "  rows: 1\n";
	fout << "  cols: 5\n";
	fout << "  data: [" << mat_to_linear(DISTORTION_R) << "]\n";
	fout << "rectification_matrix:\n";
	fout << "  rows: 3\n";
	fout << "  cols: 3\n";
	fout << "  data: [" << mat_to_linear(RECTIFICATION_R) << "]\n";
	fout << "projection_matrix:\n";
	fout << "  rows: 3\n";
	fout << "  cols: 4\n";
	fout << "  data: [" << mat_to_linear(PROJECTION_R) << "]\n";
	fout.close();
	
	fout = std::ofstream(argv[5]);
	fout << "matrix: [" << mat_to_linear(L_to_R.matrix()) << "]\n";
	fout << "ros_x: " << L_to_R.translation().x() << "\n";
	fout << "ros_y: " << L_to_R.translation().y() << "\n";
	fout << "ros_z: " << L_to_R.translation().z() << "\n";
	fout << "ros_r: " << rpw.x() << "\n";
	fout << "ros_p: " << rpw.y() << "\n";
	fout << "ros_w: " << rpw.z() << "\n";
	fout.close();

	//TODO Dafuq is all this?

	/*for(int i = 1; i < 17; i++){
		printf("%s\n", argv[LCONST + i]);
	}
	
	Eigen::Affine3d L_to_SLED;
	L_to_SLED.matrix() <<
		std::stod(argv[LCONST + 1 ]), std::stod(argv[LCONST + 2 ]), std::stod(argv[LCONST + 3 ]), std::stod(argv[LCONST + 4 ]), 
		std::stod(argv[LCONST + 5 ]), std::stod(argv[LCONST + 6 ]), std::stod(argv[LCONST + 7 ]), std::stod(argv[LCONST + 8 ]), 
		std::stod(argv[LCONST + 9 ]), std::stod(argv[LCONST + 10]), std::stod(argv[LCONST + 11]), std::stod(argv[LCONST + 12]), 
		std::stod(argv[LCONST + 13]), std::stod(argv[LCONST + 14]), std::stod(argv[LCONST + 15]), std::stod(argv[LCONST + 16])
	;
	
	printf("\n\nLeft camera to mill:\n");
	std::cout << L_to_SLED.matrix();
	std::cout << "\n";
	
	for(int i = 1; i < 17; i++){
		printf("%s\n", argv[RCONST + i]);
	}
	
	Eigen::Affine3d R_to_SLED;
	R_to_SLED.matrix() <<
		std::stod(argv[RCONST + 1 ]), std::stod(argv[RCONST + 2 ]), std::stod(argv[RCONST + 3 ]), std::stod(argv[RCONST + 4 ]), 
		std::stod(argv[RCONST + 5 ]), std::stod(argv[RCONST + 6 ]), std::stod(argv[RCONST + 7 ]), std::stod(argv[RCONST + 8 ]), 
		std::stod(argv[RCONST + 9 ]), std::stod(argv[RCONST + 10]), std::stod(argv[RCONST + 11]), std::stod(argv[RCONST + 12]), 
		std::stod(argv[RCONST + 13]), std::stod(argv[RCONST + 14]), std::stod(argv[RCONST + 15]), std::stod(argv[RCONST + 16])
	;
	
	printf("\nRight camera to mill:\n");
	std::cout << R_to_SLED.matrix();
	
	Eigen::Affine3d L_to_R = L_to_SLED.inverse() * R_to_SLED;
	printf("\n\n\nLEFT TO RIGHT INTEROCULAR TRANSFORM IS \n\n");
	std::cout << L_to_R.matrix();
	printf("\n\n");
	
	Eigen::Matrix3d r = L_to_R.rotation();
	Eigen::Vector3d t = L_to_R.translation();
	
	cv::Mat R = (cv::Mat_<double>(3, 3) <<
		r(0, 0),
		r(0, 1),
		r(0, 2),
		r(1, 0),
		r(1, 1),
		r(1, 2),
		r(2, 0),
		r(2, 1),
		r(2, 2)
	);
	
	cv::Mat T = (cv::Mat_<double>(3, 1) <<
		t(0),
		t(1),
		t(2)
	);
	
	printf("Reading left file %s...\n", argv[44]);
	printf("Reading right file %s...\n", argv[43]);
	
	YAML::Node l_camera;
	YAML::Node r_camera;
	r_camera = YAML::LoadFile(argv[44]);
	l_camera = YAML::LoadFile(argv[43]);
	
	YAML::Node tmp;
	tmp = l_camera["camera_matrix"]["data"];
	cv::Mat INTRENSIC_L = (cv::Mat_<double>(3, 3) <<
		tmp[0].as<double>(), tmp[1].as<double>(), tmp[2].as<double>(),
		tmp[3].as<double>(), tmp[4].as<double>(), tmp[5].as<double>(), 
		tmp[6].as<double>(), tmp[7].as<double>(), tmp[8].as<double>()
	);
	std::cout << "Left intrinsics: \n" << tmp << "\n";
	
	tmp = r_camera["camera_matrix"]["data"];
	cv::Mat INTRENSIC_R = (cv::Mat_<double>(3, 3) <<
		tmp[0].as<double>(), tmp[1].as<double>(), tmp[2].as<double>(),
		tmp[3].as<double>(), tmp[4].as<double>(), tmp[5].as<double>(), 
		tmp[6].as<double>(), tmp[7].as<double>(), tmp[8].as<double>()
	);
	std::cout << "Right intrinsics: \n" << tmp << "\n";
	
	tmp = l_camera["distortion_coefficients"]["data"];
	cv::Mat DISTORTION_L = (cv::Mat_<double>(1, 5) <<
		tmp[0].as<double>(),
		tmp[1].as<double>(),
		tmp[2].as<double>(),
		tmp[3].as<double>(),
		tmp[4].as<double>()
	);
	std::cout << "Left distortion: \n" << tmp << "\n";
	
	tmp = r_camera["distortion_coefficients"]["data"];
	cv::Mat DISTORTION_R = (cv::Mat_<double>(1, 5) <<
		tmp[0].as<double>(),
		tmp[1].as<double>(),
		tmp[2].as<double>(),
		tmp[3].as<double>(),
		tmp[4].as<double>()
	);
	std::cout << "Right distortion: \n" << tmp << "\n";
	
	
	
	cv::Mat RECTIFICATION_L;
	cv::Mat RECTIFICATION_R;
	cv::Mat PROJECTION_L;
	cv::Mat PROJECTION_R;
	
	RECTIFICATION_L = (cv::Mat_<double>(3, 3) << 1., 0., 0., 0., 1., 0., 0., 0., 1.);
	RECTIFICATION_R = (cv::Mat_<double>(3, 3) << 1., 0., 0., 0., 1., 0., 0., 0., 1.);
	
	PROJECTION_L = (cv::Mat_<double>(3, 4) << INTRENSIC_L.at<double>(0), 0., INTRENSIC_L.at<double>(2), 0., 0., INTRENSIC_L.at<double>(4), INTRENSIC_L.at<double>(5), 0., 0., 0., 1., 0.);
	PROJECTION_R = (cv::Mat_<double>(3, 4) << INTRENSIC_R.at<double>(0), 0., INTRENSIC_R.at<double>(2), 0., 0., INTRENSIC_R.at<double>(4), INTRENSIC_R.at<double>(5), 0., 0., 0., 1., 0.);
	
	
	cv::Mat Q;//Not used.
	cv::Size_<int> roi = cv::Size_<int>(640, 480);
	cv::Rect roi_1;
	cv::Rect roi_2;
	cv::Size nis = cv::Size_<int>(640, 480);
	
	cv::stereoRectify(
		INTRENSIC_L,
		DISTORTION_L,
		INTRENSIC_R,
		DISTORTION_R,
		roi,
		R,
		T,
		RECTIFICATION_L,
		RECTIFICATION_R,
		PROJECTION_L,
		PROJECTION_R,
		Q,
		//0,
		cv::CALIB_ZERO_DISPARITY,
		1.0,
		nis,
		&roi_1,
		&roi_2
	);
	
	//if(resize){
		/*double scale_factor_LX = 640.0 / roi_1.height;
		double scale_factor_LY = 480.0 / roi_1.height;
		double scale_factor_RX = 640.0 / roi_2.height;
		double scale_factor_RY = 480.0 / roi_2.height;
	
		double scale_factor_max = std::max(
			scale_factor_LX,
			std::max(
				scale_factor_LY,
				std::max(
					scale_factor_RX, scale_factor_RY
				)
			)
		);
	
		nis = cv::Size_<int>(640 * scale_factor_max, 480 * scale_factor_max);
	
		cv::stereoRectify(
			INTRENSIC_L,
			DISTORTION_L,
			INTRENSIC_R,
			DISTORTION_R,
			roi,
			R,
			T,
			RECTIFICATION_L,
			RECTIFICATION_R,
			PROJECTION_L,
			PROJECTION_R,
			Q,
			//0,
			cv::CALIB_ZERO_DISPARITY,
			1.0,
			nis
		);*/
	//}
	
	/*std::ofstream fout;
	
	fout = std::ofstream(argv[43]);
	fout << "image_width: " << nis.width << "\n";
	fout << "image_height: " << nis.height << "\n";
	fout << "camera_name: endo_cam_l\n";
	fout << "camera_matrix:\n";
	fout << "  rows: 3\n";
	fout << "  cols: 3\n";
	fout << "  data: [" << mat_to_linear(INTRENSIC_L) << "]\n";
	fout << "distortion_model: plumb_bob\n";
	fout << "distortion_coefficients:\n";
	fout << "  rows: 1\n";
	fout << "  cols: 5\n";
	fout << "  data: [" << mat_to_linear(DISTORTION_L) << "]\n";
	fout << "rectification_matrix:\n";
	fout << "  rows: 3\n";
	fout << "  cols: 3\n";
	fout << "  data: [" << mat_to_linear(RECTIFICATION_L) << "]\n";
	fout << "projection_matrix:\n";
	fout << "  rows: 3\n";
	fout << "  cols: 4\n";
	fout << "  data: [" << mat_to_linear(PROJECTION_L) << "]\n";
	fout.close();
	
	fout = std::ofstream(argv[44]);
	fout << "image_width: " << nis.width << "\n";
	fout << "image_height: " << nis.height << "\n";
	fout << "camera_name: endo_cam_r\n";
	fout << "camera_matrix:\n";
	fout << "  rows: 3\n";
	fout << "  cols: 3\n";
	fout << "  data: [" << mat_to_linear(INTRENSIC_R) << "]\n";
	fout << "distortion_model: plumb_bob\n";
	fout << "distortion_coefficients:\n";
	fout << "  rows: 1\n";
	fout << "  cols: 5\n";
	fout << "  data: [" << mat_to_linear(DISTORTION_R) << "]\n";
	fout << "rectification_matrix:\n";
	fout << "  rows: 3\n";
	fout << "  cols: 3\n";
	fout << "  data: [" << mat_to_linear(RECTIFICATION_R) << "]\n";
	fout << "projection_matrix:\n";
	fout << "  rows: 3\n";
	fout << "  cols: 4\n";
	fout << "  data: [" << mat_to_linear(PROJECTION_R) << "]\n";
	fout.close();*/

	return 0;
}
