#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace cc_utils{
	//Visualization utils.
	void init_visualization(int res_x, int res_y, const std::vector<Eigen::Vector2d> gt_pixels, ceres::Solver::Options & o);
	
	//Officially the stupidest thing, this profoundly hacky process is required to
	//get the core value out of a ceres::jet since it cannot be accessed
	//directly even in a read-only manner. WHY is this functionalty missing from a
	//flagship piece of software released by a major corporation for prestigious
	//institutions of learning to use?????????
	template<typename T> inline double val(const T& j){
		std::ostringstream oss;
		oss << j;
	
		//Sometimes a Jet string has a big long blob of derivative data attached to it
		//that we need to cut off, and a bracket at the beginning...
		if(oss.str().find_first_of("[") != std::string::npos){
			return std::stod(oss.str().substr(1, oss.str().find_first_of(";") - 1));
		}
	
		//And SOMETIMES it's just a number.
		else{
			return std::stod(oss.str());
		}
	
		//I have NO idea why.
	}
	
	void add_to_visualization(double u, double v, double u_base, double v_base);
	
	double rms();

	//Math utils
	template <typename T> inline T dtor(T d){
		return (d * T(M_PI)) / T(180.0);
	}
	template <typename T> inline T rtod(T r){
		return (r * T(180.0)) / T(M_PI);
	}
	
	template <typename T> inline void transformPoint_rm(
		const T translation[3],
		const T rotation[9],
		const T point_in[3],
		T point_out[3]
	){
		/*std::cout << "\tTRANSLATION\n";
		std::cout << "\t" << translation[0] << "\n";
		std::cout << "\t" << translation[1] << "\n";
		std::cout << "\t" << translation[2] << "\n";
	
		std::cout << "\tROTATION\n";
		std::cout << "\t" << rotation[0] << "  " << rotation[1] << "  " << rotation[2] << "\n";
		std::cout << "\t" << rotation[3] << "  " << rotation[4] << "  " << rotation[5] << "\n";
		std::cout << "\t" << rotation[6] << "  " << rotation[7] << "  " << rotation[8] << "\n";*/

		point_out[0] = (point_in[0] * rotation[0] + point_in[1] * rotation[1] + point_in[2] * rotation[2]) + translation[0];
		point_out[1] = (point_in[0] * rotation[3] + point_in[1] * rotation[4] + point_in[2] * rotation[5]) + translation[1];
		point_out[2] = (point_in[0] * rotation[6] + point_in[1] * rotation[7] + point_in[2] * rotation[8]) + translation[2];
	}
	template <typename T> inline void transformPoint_euler(
		const T translation[3],
		const T rotation[3],
		const T point_in[3],
		T point_out[3]
	){
		T R [9];
		ceres::EulerAnglesToRotationMatrix(rotation, 3, R);
	
		cc_utils::transformPoint_rm(translation, R, point_in, point_out);
	}
	
	template <typename T> inline void project(
		const T& point_x, const T& point_y, const T& point_z,
		const T& fx, const T& fy, const T& cx, const T& cy,
		const T& k1, const T& k2, const T& k3, const T& p1, const T& p2,
		
		const T& r_x, const T& r_y,
		
		T& u, T& v
	){
		//Projection
		T xp1 = point_x;
		T yp1 = point_y;
		T zp1 = point_z;
		T xp_plane;
		T yp_plane;
	
		std::cout << point_x << "\n";
		std::cout << point_y << "\n";
		std::cout << point_z << "\n\n";
		
		if(zp1 != T(0)){//If the distance to the image plane is zero something is VERY wrong.
			xp_plane = xp1 / zp1;
			yp_plane = yp1 / zp1;
		}
		
		std::cout<< "x planarized " << xp_plane << "\n";
		std::cout<< "y planarized " << yp_plane << "\n\n";
		
		std::cout<< "fx " << fx << "\n";
		std::cout<< "fy " << fy << "\n";
		std::cout<< "cx " << cx << "\n";
		std::cout<< "cy " << cy << "\n";
		std::cout<< "k1 " << k1 << "\n";
		std::cout<< "k2 " << k2 << "\n";
		std::cout<< "k3 " << k3 << "\n";
		std::cout<< "p1 " << p1 << "\n";
		std::cout<< "p2 " << p2 << "\n\n";
		
		//Distortion.
		T r2 = pow(xp_plane, T(2.0)) + pow(yp_plane, T(2.0));
		T r4 = r2 * r2;
		T r6 = r2 * r2 * r2;
		
		T X_radial = xp_plane * k1 * r2 + xp_plane * k2 * r4 + xp_plane * k3 * r6;
		T Y_radial = yp_plane * k1 * r2 + yp_plane * k2 * r4 + yp_plane * k3 * r6;
		
		T X_tangential = T(2.0) * p1 * xp_plane * yp_plane + p2 * (r2 + T(2.0) * xp_plane * xp_plane);
		T Y_tangential = p1 * (r2 + T(2.0) * yp_plane * yp_plane) + T(2.0) * p2 * xp_plane * yp_plane;
		
		T xd = xp_plane + X_radial + X_tangential;
		T yd = yp_plane + Y_radial + Y_tangential;
		
		//T xd = xp_plane;
		//T yd = yp_plane;
		
		//std::cout<< "x distorted " << xd << "\n";
		//std::cout<< "y distorted " << yd << "\n\n";
		
		//Project
		u = cx + fx * xd;
		v = cy + fy * yd;
	
		std::cout<< "u projected " << u << "\n";
		std::cout<< "v projected " << v << "\n\n";
		//std::getchar();
	}
	
	void bound_rotation(ceres::Problem & p, double * variable);
}
