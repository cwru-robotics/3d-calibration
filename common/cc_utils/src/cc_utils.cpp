#include <cc_utils/cc_utils.h>

namespace cc_utils{

	cv::Mat debug_mat;
	cv::Mat the_ground_truth;
	int cnt;
	
	std::vector<double> u_calc;
	std::vector<double> v_calc;
	std::vector<double> u_real;
	std::vector<double> v_real;
	
	class VisualCallback : public ceres::IterationCallback {
	public:
		//TODO Are either of these necessary??
		explicit VisualCallback(){}
		~VisualCallback() {}
	
		ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
			cv::namedWindow("Iteration Projection", cv::WINDOW_GUI_EXPANDED | cv::WINDOW_NORMAL);
			cv::imshow("Iteration Projection", debug_mat);
			cv::waitKey(500);
			//cv::imwrite("/home/tes77/dbg_img_" + std::to_string(cnt) + ".png", debug_mat);
			cnt++;
			//cv::waitKey();
			
			cc_utils::u_calc.clear();
			cc_utils::u_real.clear();
			cc_utils::v_calc.clear();
			cc_utils::v_real.clear();
		
			the_ground_truth.copyTo(debug_mat);
			return ceres::SOLVER_CONTINUE;
		}
	};
	
	void init_visualization(int res_x, int res_y, const std::vector<Eigen::Vector2d> gt_pixels, ceres::Solver::Options & o){
		debug_mat = cv::Mat(res_y, res_x, CV_8UC3);
		the_ground_truth = cv::Mat(res_y, res_x, CV_8UC3);
		cnt = 0;
	
		for(int i = 0; i < gt_pixels.size(); i++){
			cv::drawMarker(
				the_ground_truth,
				cv::Point(gt_pixels[i][0], gt_pixels[i][1]),
				cv::Scalar(255, 0, 0),//Marker color
				cv::MARKER_TILTED_CROSS, 5//Type and size.
			);
		}
		
		the_ground_truth.copyTo(debug_mat);
		
		o.callbacks.push_back(new VisualCallback());
		o.update_state_every_iteration = true;
	}
	
	void add_to_visualization(double u, double v, double u_base, double v_base){
		u_calc.push_back(u);
		v_calc.push_back(v);
		u_real.push_back(u_base);
		v_real.push_back(v_base);
		
		cv::drawMarker(debug_mat, cv::Point(u, v), cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 5);
	}
	
	double rms(){
		double sum = 0;
		for(int i = 0; i < u_calc.size(); i++){
			sum += sqrt(
				pow(u_calc[i] - u_real[i], 2)
				+
				pow(v_calc[i] - v_real[i], 2)
			);
		}
		return sum / (double)u_calc.size();
	}
	
	void bound_rotation(ceres::Problem & p, double * variable){
		p.SetParameterLowerBound(variable, 0, -180.0);
		p.SetParameterUpperBound(variable, 0,  180.0);
		p.SetParameterLowerBound(variable, 1, -180.0);
		p.SetParameterUpperBound(variable, 1,  180.0);
		p.SetParameterLowerBound(variable, 2, -180.0);
		p.SetParameterUpperBound(variable, 2,  180.0);
	}
	
	
	Eigen::Affine3d invert_eul(
		const double & x_in, const double & y_in, const double & z_in, 
		const double & r_in, const double & p_in, const double & w_in, 
		
		double & x_out, double & y_out, double & z_out, 
		double & r_out, double & p_out, double & w_out
	){
		Eigen::Affine3d a;
		a =
			Eigen::AngleAxisd(w_in, Eigen::Vector3d::UnitZ()) *
   			Eigen::AngleAxisd(p_in, Eigen::Vector3d::UnitY()) *
   			Eigen::AngleAxisd(r_in, Eigen::Vector3d::UnitX());
   		a.translation() = Eigen::Vector3d(x_in, y_in, z_in);
   	
   		Eigen::Affine3d b = a.inverse();
   		Eigen::Vector3d ea = b.rotation().eulerAngles(2, 1, 0);
   	
		x_out = b.translation().x();
		y_out = b.translation().y();
		z_out = b.translation().z();
		r_out = ea.z();
		p_out = ea.y();
		w_out = ea.x();
		
		return b;
	}
};
