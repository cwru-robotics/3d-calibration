#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <single_corner/single_corner.h>
#include <ros/ros.h>

using namespace std;

namespace sc {
    cv::Point2d detect_single_corner(const cv::Mat& img)
    {
        // define center point
        cv::Point2d pt_C(floor(img.cols/2.0 + 1), floor(img.rows/2.0 + 1));
        double acceptable_radius = 150;

        //Blur the image to eliminate thin lines; the Hough really likes these
        //but they are not the same thing as edges of checkerboards.
        // cv::Mat soft;
        // cv::blur(img, soft, cv::Size(5, 5));

        //cv::imshow("Blurred", soft);
        
        // turn into binary to perform closing operation
        cv::Mat bin_img;
        cv::threshold(img, bin_img, 75, 175, cv::THRESH_BINARY);
        // cv::imshow("bin_img", bin_img);
        
        // remove dirt and noise
        cv::Mat closed_img;
        int size = 1;
        cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * size + 1, 2 * size + 1), cv::Point(size, size));
        cv::dilate(bin_img, closed_img, element, cv::Point(-1, -1), 2, 1, 1);
        cv::erode(closed_img, closed_img, element, cv::Point(-1, -1), 2, 1, 1);
        // cv::imshow("closed_img", closed_img);

        //Perform edge detection
        cv::Mat canny_img;
        Canny(closed_img, canny_img, 50, 200, 3);
        // cv::imshow("canny_img", canny_img);

        //Hough transform
        std::vector<cv::Vec2f> lines;
        HoughLines(canny_img, lines, 1, CV_PI / 180, 50, 0, 0);

        // Eliminate all lines too close to the edges of the image.
        // int w = img.cols;
        // int h = img.rows;
        // std::vector<cv::Vec2f> lines_tmp;
        // for (int i = 0; i < lines.size(); i++) {
        //     float rho = lines[i][0];
        //     if (rho < w / 4 || rho > 3 * (w / 4)) {
        //         continue;
        //     }
        //     if (rho < h / 4 || rho > 3 * (h / 4)) {
        //         continue;
        //     }
        //     lines_tmp.push_back(lines[i]);
        // }
        // lines = std::vector<cv::Vec2f>(lines_tmp);


        // Draw all the lines
        std::vector<cv::Vec2f> close_to_center_lines;
        cout    << left << setw(6) << "rho "
                << left << setw(16) << "| theta"
                << left << "| distance to center" << endl;
        cout << "--------------------------------------------" << endl;
        cv::Mat all = img.clone();
        for (int i = 0; i < lines.size(); i++) {
            float rho = lines[i][0];
            float theta = lines[i][1];
            cv::Point pt_A, pt_B;
            double a = cos(theta), b = sin(theta);
            double x0 = a * rho, y0 = b * rho;
            pt_A.x = cvRound(x0 + 1000 * (-b));
            pt_A.y = cvRound(y0 + 1000 * (a));
            pt_B.x = cvRound(x0 - 1000 * (-b));
            pt_B.y = cvRound(y0 - 1000 * (a));

            // check whether the line is close enough to center of image
            cv::Vec2d vec_BA(pt_B.x - pt_A.x, pt_B.y - pt_A.y);
            cv::Vec2d vec_CA(pt_C.x - pt_A.x, pt_C.y - pt_A.y);
            double dist_to_C = abs((vec_BA[0]*vec_CA[1] - vec_BA[1]*vec_CA[0]))/sqrt(vec_BA[0]*vec_BA[0] + vec_BA[1]*vec_BA[1]);
            
            //! print all lines
            // cout    << left << setw(6) << rho <<  "| "
            //         << left << setw(14) << theta << "| "
            //         << left << dist_to_C << endl;
            
            int str = (i * 255) / lines.size();
            line(all, pt_A, pt_B, cv::Scalar(255 - str, 0, str), 3, cv::LINE_AA);

            if (dist_to_C < acceptable_radius)   
            {
                //! print selected lines
                cout    << left << setw(6) << rho <<  "| "
                        << left << setw(14) << theta << "| "
                        << left << dist_to_C << endl;
                close_to_center_lines.push_back(lines[i]);
            }
        }
        cv::imshow("All Lines", all);

        // if couldn't find center, return top left corner
        if (close_to_center_lines.empty()) 
        {
            ROS_WARN("No detected line that is close enough to center of img");
            return cv::Point2d(0,0);
        }

        // Find the "leading" line.
        double strongest_rho = close_to_center_lines[0][0];
        double strongest_theta = close_to_center_lines[0][1];
        //printf("Strongest theta is %f.\n", strongest_theta);

        // Find the strongest line which is reasonably perpendicular to the leading line.
        int opposition_index = 0;
        while (abs(strongest_theta - close_to_center_lines[opposition_index][1]) < M_PI/4.0
                && opposition_index < close_to_center_lines.size()) 
        {
            opposition_index++;
        }

        // if no opposition, return top left corner
        if (opposition_index == 0 || opposition_index >= close_to_center_lines.size())
        {
            ROS_WARN("Found 1 line, but couldn't find its perpendicular one");
            return cv::Point2d(0,0);
        }

        double opposition_rho = lines[opposition_index][0];
        double opposition_theta = lines[opposition_index][1];
        //printf("Opposition theta is %f.\n", lines[opposition_index][1]);

        //Calculate the intersection of the lines.
        double a_s = cos(strongest_theta);
        double b_s = sin(strongest_theta);
        double a_o = cos(opposition_theta);
        double b_o = sin(opposition_theta);

        double x0_s = a_s * strongest_rho;
        double y0_s = b_s * strongest_rho;
        double x0_o = a_o * opposition_rho;
        double y0_o = b_o * opposition_rho;

        cv::Point p1s, p2s, p1o, p2o;
        p1s.x = cvRound(x0_s + 1000 * -b_s);
        p1s.y = cvRound(y0_s + 1000 * a_s);
        p2s.x = cvRound(x0_s - 1000 * -b_s);
        p2s.y = cvRound(y0_s - 1000 * a_s);
        p1o.x = cvRound(x0_o + 1000 * -b_o);
        p1o.y = cvRound(y0_o + 1000 * a_o);
        p2o.x = cvRound(x0_o - 1000 * -b_o);
        p2o.y = cvRound(y0_o - 1000 * a_o);

        cv::Mat both = img.clone();
        cv::line(both, p1s, p2s, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        cv::line(both, p1o, p2o, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
        cv::circle(both, pt_C, acceptable_radius, cv::Scalar(0,255,0));
        cv::drawMarker( both, pt_C,	cv::Scalar(0, 255, 0), cv::MARKER_TILTED_CROSS, 9);
        cv::imshow("Intersection Lines", both);

        string ws_path = getenv("ROS_WORKSPACE");
        cv::imwrite(ws_path + "/debug_img/" + "corner_img.jpg", both);

        double x1 = p1s.x;
        double y1 = p1s.y;
        double x2 = p2s.x;
        double y2 = p2s.y;
        double x3 = p1o.x;
        double y3 = p1o.y;
        double x4 = p2o.x;
        double y4 = p2o.y;

        double d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        cv::Point output;
        output.x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / d;
        output.y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / d;

        if(cv::waitKey()) cv::destroyAllWindows();

        return output;
    }
}
