/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Copyright (C) 2014, Southwest Research Institute, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

/********************************************************************************************
 **   Slight Modification of OpenCV function to use ellipse fitting rather than center of mass of contour ****
 **  to provide the location of the circle ****
 ********************************************************************************************/

#include "opencv2/features2d.hpp"
#include "opencv2/imgproc.hpp"

#include "opencv2/core/utility.hpp"
//#include "opencv2/core/private.hpp"
#include "opencv2/core/ocl.hpp"
#include "opencv2/core/hal/hal.hpp"

#include <algorithm>

#ifdef HAVE_TEGRA_OPTIMIZATION
#include "opencv2/features2d/features2d_tegra.hpp"
#endif

#include "opencv2/opencv.hpp"

#include "../include/circle_detector.hpp"
#include <iterator>

//#define DEBUG_CIRCLE_DETECTOR

#ifdef DEBUG_CIRCLE_DETECTOR
#include "opencv2/opencv_modules.hpp"
#ifdef HAVE_OPENCV_HIGHGUI
#include "opencv2/highgui/highgui.hpp"
#else
#undef DEBUG_CIRCLE_DETECTOR
#endif
#endif

const double CLUSTER_ACCEPTANCE_RADIUS = 10; //10 pixels??

int num_findCircles_calls=0;

using namespace std;
int g_ans;
namespace cv {
/*
    class CV_EXPORTS_W CircleDetectorImpl : public CircleDetector {
    protected:

        struct CV_EXPORTS Center {
            Point2d location;
            double radius;
            double confidence;
        };

        virtual void detect(InputArray image, std::vector<KeyPoint>& keypoints, InputArray mask = noArray());
        virtual void findCircles(InputArray image, InputArray binaryImage, std::vector<Center>& centers) const;

        Params params;

    public:
        explicit CircleDetectorImpl(const CircleDetector::Params& parameters = CircleDetector::Params());

        virtual void read(const FileNode& fn);
        virtual void write(FileStorage& fs) const;
        Point2d center_to_point(Center center);

    };
*/
    /*
     *  CircleDetector
     */
    CircleDetector::Params::Params() {
        thresholdStep = 10;
        minThreshold = 50;
        maxThreshold = 220;
        minRepeatability = 2;
        minDistBetweenCircles = 10;
        minRadiusDiff = 200; //50; //10;

        filterByColor = false;
        circleColor = 0;

        filterByArea = true;
        minArea = 25; //25;
        maxArea = 10000; //10000; //5000;

        filterByCircularity = true; //false;
        minCircularity = 0.8f;
        maxCircularity = std::numeric_limits<float>::max();

        filterByInertia = false;
        minInertiaRatio = 0.1f; //0.1f;
        maxInertiaRatio = std::numeric_limits<float>::max();

        filterByConvexity = true;
        minConvexity = 0.5f;
        maxConvexity = std::numeric_limits<float>::max();
    }

    void CircleDetector::Params::read(const cv::FileNode& fn) {
        thresholdStep = fn["thresholdStep"];
        minThreshold = fn["minThreshold"];
        maxThreshold = fn["maxThreshold"];

        minRepeatability = (size_t) (int) fn["minRepeatability"];
        minDistBetweenCircles = fn["minDistBetweenCircles"];

        filterByColor = (int) fn["filterByColor"] != 0 ? true : false;
        circleColor = (uchar) (int) fn["circleColor"];

        filterByArea = (int) fn["filterByArea"] != 0 ? true : false;
        minArea = fn["minArea"];
        maxArea = fn["maxArea"];

        filterByCircularity = (int) fn["filterByCircularity"] != 0 ? true : false;
        minCircularity = fn["minCircularity"];
        maxCircularity = fn["maxCircularity"];

        filterByInertia = (int) fn["filterByInertia"] != 0 ? true : false;
        minInertiaRatio = fn["minInertiaRatio"];
        maxInertiaRatio = fn["maxInertiaRatio"];

        filterByConvexity = (int) fn["filterByConvexity"] != 0 ? true : false;
        minConvexity = fn["minConvexity"];
        maxConvexity = fn["maxConvexity"];
    }

    void CircleDetector::Params::write(cv::FileStorage& fs) const {
        fs << "thresholdStep" << thresholdStep;
        fs << "minThreshold" << minThreshold;
        fs << "maxThreshold" << maxThreshold;

        fs << "minRepeatability" << (int) minRepeatability;
        fs << "minDistBetweenCircles" << minDistBetweenCircles;

        fs << "filterByColor" << (int) filterByColor;
        fs << "circleColor" << (int) circleColor;

        fs << "filterByArea" << (int) filterByArea;
        fs << "minArea" << minArea;
        fs << "maxArea" << maxArea;

        fs << "filterByCircularity" << (int) filterByCircularity;
        fs << "minCircularity" << minCircularity;
        fs << "maxCircularity" << maxCircularity;

        fs << "filterByInertia" << (int) filterByInertia;
        fs << "minInertiaRatio" << minInertiaRatio;
        fs << "maxInertiaRatio" << maxInertiaRatio;

        fs << "filterByConvexity" << (int) filterByConvexity;
        fs << "minConvexity" << minConvexity;
        fs << "maxConvexity" << maxConvexity;
    }

    CircleDetectorImpl::CircleDetectorImpl(const CircleDetector::Params& parameters) : params(parameters) {
    }

    void CircleDetectorImpl::read(const cv::FileNode& fn) {
        params.read(fn);
    }

    void CircleDetectorImpl::write(cv::FileStorage& fs) const {
        writeFormat(fs);
        params.write(fs);
    }

    Point2d CircleDetectorImpl::center_to_point(Center center) {
        Point2d point = center.location;
        return point;
    }

    void make_new_cluster(Point2d test_pt, vector<Point2d> &cluster_centers,
            vector<double> &n_members) {
        cluster_centers.push_back(test_pt);
        n_members.push_back(1.0);
    }

    void add_to_cluster(Point2d test_pt, int i_cluster,
            vector<Point2d> &cluster_centers, vector<double> &n_members) {
        double init_num_cluster_pts = n_members[i_cluster];
        Point2d init_cluster_ctr = cluster_centers[i_cluster];
        Point2d new_cluster_ctr = (init_num_cluster_pts * init_cluster_ctr + test_pt) / (init_num_cluster_pts + 1.0);
        cluster_centers[i_cluster] = new_cluster_ctr;
        n_members[i_cluster] = init_num_cluster_pts + 1.0;
    }

    bool fits_in_cluster(Point2d cluster_center, Point2d test_pt) {
        double dist = norm(cluster_center - test_pt);
        if (dist < CLUSTER_ACCEPTANCE_RADIUS) {
            //printf("test pt is %f from cluster center", dist);
            return true;
        } else
            return false;
    }

    void CircleDetectorImpl::findCircles(InputArray _image, InputArray _binaryImage, std::vector<Center>& centers) const {
        //  CV_INSTRUMENT_REGION()
        //printf("************* findCircles ******************\n");
        num_findCircles_calls++;
        //printf("call number %d",num_findCircles_calls);
        Mat image = _image.getMat(); // Oh so much  cleaner this way :(
        Mat binaryImage = _binaryImage.getMat();
        Mat keypointsImage = image;
        (void) image;
        centers.clear();
        double circularityRatio, inertiaRatio, convexityRatio, circleArea;
        int n_points, circleColor;
        vector<vector<Point> > contours;
        Mat tmpBinaryImage = binaryImage.clone();
        findContours(tmpBinaryImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
        int n_contours = contours.size();
        // vector<vector<Point> > contours0;
        //     for( size_t k = 0; k < contours0.size(); k++ ) 
        //      approxPolyDP(Mat(contours0[k]), contours[k], eps, true);

        //printf("found %d contours\n", n_contours);
        // loop on all contours
        int n_survivors = 0;
        for (size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++) {
            // each if statement may eliminate a contour through the continue function
            // some if statements may also set the confidence whose default is 1.0
            Center center;
            center.confidence = 1;
            Moments moms = moments(Mat(contours[contourIdx]));
            //printf("contour %d\n", (int) contourIdx);
            if (params.filterByArea) {
                double area = moms.m00;
                circleArea = area;
                //if (area < params.minArea || area >= params.maxArea) {
                //    printf("rejected for area = %f\n",circleArea);
                //}
                //else {printf("area= %f\n",circleArea); }
                if (circleArea > 2000) {
                    //cout << "large circle, contour:  " << contourIdx << endl;
                }
                if (area < params.minArea || area >= params.maxArea) continue;

            }

            double area = moms.m00;
            double perimeter = arcLength(Mat(contours[contourIdx]), true);
            double ratio = 4 * CV_PI * area / (perimeter * perimeter);
            circularityRatio = ratio;
            /*try moving this lower
          if (params.filterByCircularity)
          {

              if (circularityRatio < params.minCircularity || circularityRatio >= params.maxCircularity) {
                printf("rejected for circularity at ratio = %f\n",circularityRatio);
                         if (circleArea>2000) { 
                cout<<"large circle being rejected; enter 1: ";
                cin>>g_ans;
              }
            }
      
            if (circularityRatio < params.minCircularity || circularityRatio >= params.maxCircularity) continue;
          }
             */
            if (params.filterByInertia) {
                double denominator = sqrt(pow(2 * moms.mu11, 2) + pow(moms.mu20 - moms.mu02, 2));
                const double eps = 1e-2;
                double ratio;
                if (denominator > eps) {
                    double cosmin = (moms.mu20 - moms.mu02) / denominator;
                    double sinmin = 2 * moms.mu11 / denominator;
                    double cosmax = -cosmin;
                    double sinmax = -sinmin;

                    double imin = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmin - moms.mu11 * sinmin;
                    double imax = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmax - moms.mu11 * sinmax;
                    ratio = imin / imax;
                    inertiaRatio = ratio;
                } else {
                    ratio = 1;
                    inertiaRatio = ratio;
                }
                if (ratio < params.minInertiaRatio || ratio >= params.maxInertiaRatio) {
                    //printf("rejected for inertia ratio %f\n", inertiaRatio);
                    if (circleArea > 2000) {
                        //cout << "large circle being rejected; enter 1: ";
                        //cin>>g_ans;
                    }
                }

                if (ratio < params.minInertiaRatio || ratio >= params.maxInertiaRatio) continue;

                center.confidence = ratio * ratio;
            }


            vector<Point> hull;
            convexHull(Mat(contours[contourIdx]), hull);
            double contour_area = contourArea(Mat(contours[contourIdx]));
            double hullArea = contourArea(Mat(hull));
            double convexityRatio = contour_area / hullArea;
            //convexityRatio = ratio;

            /* try moving this down
          if (params.filterByConvexity)
          {
 
            if (convexityRatio < params.minConvexity || convexityRatio >= params.maxConvexity) {
                printf("rejected for convexity, ratio = %f \n",convexityRatio);
               if (circleArea>2000) { 
                cout<<"large circle being rejected; enter 1: ";
                cin>>g_ans;
            }          
            }

            if (convexityRatio < params.minConvexity || convexityRatio >= params.maxConvexity) continue;
          }
             */
            Mat pointsf;
            Mat(contours[contourIdx]).convertTo(pointsf, CV_32F);
            n_points = pointsf.rows;
            if (pointsf.rows < 5) {
                //printf("rejected for too few points; pointsf.rows = %d\n", n_points);
                if (circleArea > 2000) {
                    //cout << "large circle being rejected; enter 1: ";
                    //cin>>g_ans;
                }
            }

            if (pointsf.rows < 5) continue;

            RotatedRect box = fitEllipse(pointsf);

            // find center
            // center.location = Point2d(moms.m10 / moms.m00, moms.m01 / moms.m00);
            center.location = box.center;

            if (circleArea > 2000) {
                //cout << "large circle, contour:  " << contourIdx << endl;
                //printf("center: %f, %f\n", center.location.x, center.location.y);
            }

            if (params.filterByCircularity) {

                if (circularityRatio < params.minCircularity || circularityRatio >= params.maxCircularity) {
                    //printf("rejected for circularity at ratio = %f\n", circularityRatio);
                    if (circleArea > 2000) {
                        //cout << "large circle being rejected; enter 1: ";
                        //cin>>g_ans;
                    }
                }

                if (circularityRatio < params.minCircularity || circularityRatio >= params.maxCircularity) continue;
            }

            if (params.filterByConvexity) {

                if (convexityRatio < params.minConvexity || convexityRatio >= params.maxConvexity) {
                    //printf("rejected for convexity, ratio = %f \n", convexityRatio);
                    if (circleArea > 2000) {
                        //cout << "large circle being rejected; enter 1: ";
                        //cin>>g_ans;
                    }
                }

                if (convexityRatio < params.minConvexity || convexityRatio >= params.maxConvexity) continue;
            }
            // one more filter by color of central pixel
            if (params.filterByColor) {
                circleColor = binaryImage.at<uchar>(cvRound(center.location.y), cvRound(center.location.x));
                if (binaryImage.at<uchar>(cvRound(center.location.y), cvRound(center.location.x)) != params.circleColor) {
                    //printf("rejected for color %d \n", circleColor);
                    if (circleArea > 2000) {
                        //cout << "large circle being rejected; enter 1: ";
                        //cin>>g_ans;
                    }
                }
                if (binaryImage.at<uchar>(cvRound(center.location.y), cvRound(center.location.x)) != params.circleColor) continue;
            }

            // compute circle radius
            //	{
            //	vector<double> dists;
            //	for (size_t pointIdx = 0; pointIdx < contours[contourIdx].size(); pointIdx++)
            //	{
            //		Point2d pt = contours[contourIdx][pointIdx];
            //		dists.push_back(norm(center.location - pt));
            //	}
            //	std::sort(dists.begin(), dists.end());
            //	center.radius = (dists[(dists.size() - 1) / 2] + dists[dists.size() / 2]) / 2.;
            //}
            center.radius = (box.size.height + box.size.width) / 4.0;
            //printf("radius = %f\n", center.radius);
            centers.push_back(center);
            n_survivors++;
            //#ifdef DEBUG_CIRCLE_DETECTOR
            //OOPS!!  drawing circles seems to affect the grid finder!
            //maybe keypointsImage points to the image being analyzed??
            //circle(keypointsImage, center.location, 5, Scalar(255, 255, 255), 5);
            /*cout << "number of surviving contours = " << n_survivors << endl;
            printf("imshow and waitKey...\n");
            printf("**** ACCEPTED CONTOUR****  %d centers\n",(int) centers.size());
            printf("center: %f, %f\n", center.location.x, center.location.y);
            printf(" area = %f\n", circleArea);
            printf("circularity ratio = %f\n", circularityRatio);
            printf("inertia ratio %f\n", inertiaRatio);
            printf("convexity ratio = %f \n", convexityRatio);
            printf("num points = %d\n", n_points);
            printf(" color %d \n", circleColor);
            printf("radius = %f\n", center.radius);*/
            if (circleArea > 2000) {
                //cout << "large circle accepted; enter 1: ";
                //cin>>g_ans;
            }

            //imshow("bk", keypointsImage);
            //waitKey(100);
            //cout<<"enter 1: ";
            //cin>>g_ans;
            //#endif
        }
        //#ifdef DEBUG_CIRCLE_DETECTOR
        //printf("imshow and waitKey...\n");
       // imshow("bk", keypointsImage);
        //waitKey(10);
        //wsn: consolidate the surviving centers here...
        //this did not work, since this function seems to get called oddly by pattern finder
        /*
        int n_centers = centers.size();
        cout<<"starting clustering of "<<n_centers<<" centers"<<endl;
        vector<Point2d> cluster_centers;
        vector<double> n_members;
        Center test_center;
        if (n_centers>0) {
        test_center = centers[0];
        Point2d test_pt = test_center.location; //center_to_point(test_center);
        make_new_cluster(test_pt, cluster_centers, n_members);
        int n_clusters = 1;
        
        for (int i_center = 1; i_center < n_centers; i_center++) {
            int i_cluster = 0;
            bool found_fit = false;
            test_center = centers[i_center];
            test_pt = test_center.location; //center_to_point(centers[i_center]); //try this contour; see if it fits a cluster
            while ((i_cluster < n_clusters) && (!found_fit)) { //search through clusters
                Point2d cluster_ctr = cluster_centers[i_cluster];
                found_fit = fits_in_cluster(cluster_ctr, test_pt);
                if (found_fit) {
                    add_to_cluster(test_pt, i_cluster, cluster_centers, n_members); //add_to_cluster(Point2d test_pt, int i_cluster,                                                            //vector<Point2d> &cluster_centers, vector<double> &n_members)
                    found_fit = true;
                }
                i_cluster++;
            }

            if (!found_fit) {
                make_new_cluster(test_pt, cluster_centers, n_members);
                printf("making new cluster: cluster number %d \n", (int) cluster_centers.size());
            }
        }
        n_clusters = (int) cluster_centers.size();
        printf("done clustering points; number of clusters = %d \n", n_clusters);
        for (int i = 0; i < n_clusters; i++) {
            Point2d cluster_ctr = cluster_centers[i];
            printf("cluster %d has %d points at %f,%f \n", i, (int) n_members[i], cluster_ctr.x, cluster_ctr.y);
        }
        
        }
        else {
            cout<<"PROBLEM: ZERO CENTERS \n"<<endl;
        }
        cout<<"end of circle test; enter 1:";
        cin>>g_ans;
        */
        //#endif
    }

    void CircleDetectorImpl::detect(InputArray _image, std::vector<KeyPoint>& keypoints, InputArray mask) {
        Mat image = _image.getMat();

        //  CV_INSTRUMENT_REGION()

        // TODO: support mask
        keypoints.clear();
        Mat grayscaleImage;
        if (image.channels() == 3)
            cvtColor(image, grayscaleImage, CV_BGR2GRAY);
        else
            grayscaleImage = image;

        vector<vector<Center> > centers;
        for (double thresh = params.minThreshold; thresh < params.maxThreshold; thresh += params.thresholdStep) {
            Mat binarizedImage;
            threshold(grayscaleImage, binarizedImage, thresh, 255, THRESH_BINARY);

#ifdef DEBUG_CIRCLE_DETECTOR
            //    Mat keypointsImage;
            //    cvtColor( binarizedImage, keypointsImage, CV_GRAY2RGB );
#endif

            vector<Center> curCenters;
            findCircles(grayscaleImage, binarizedImage, curCenters);
            vector<vector<Center> > newCenters;
            for (size_t i = 0; i < curCenters.size(); i++) {
#ifdef DEBUG_CIRCLE_DETECTOR
                //      circle(keypointsImage, curCenters[i].location, curCenters[i].radius, Scalar(0,0,255),-1);
#endif

                bool isNew = true;
                for (size_t j = 0; j < centers.size(); j++) {
                    double dist = norm(centers[j][centers[j].size() / 2].location - curCenters[i].location);
                    //				isNew = dist >= params.minDistBetweenCircles && dist >= centers[j][ centers[j].size() / 2 ].radius &&
                    // dist >= curCenters[i].radius;
                    double rad_diff = fabs(centers[j][centers[j].size() / 2].radius - curCenters[i].radius);
                    isNew = dist >= params.minDistBetweenCircles || rad_diff >= params.minRadiusDiff;
                    if (!isNew) {
                        centers[j].push_back(curCenters[i]);

                        size_t k = centers[j].size() - 1;
                        while (k > 0 && centers[j][k].radius < centers[j][k - 1].radius) {
                            centers[j][k] = centers[j][k - 1];
                            k--;
                        }
                        centers[j][k] = curCenters[i];

                        break;
                    }
                }
                if (isNew) {
                    newCenters.push_back(vector<Center>(1, curCenters[i]));
                    // centers.push_back(vector<Center> (1, curCenters[i]));
                }
            }
            std::copy(newCenters.begin(), newCenters.end(), std::back_inserter(centers));

#ifdef DEBUG_CIRCLE_DETECTOR
            //    imshow("binarized", keypointsImage );
            // waitKey();
#endif
        }

        for (size_t i = 0; i < centers.size(); i++) {
            if (centers[i].size() < params.minRepeatability) continue;
            Point2d sumPoint(0, 0);
            double normalizer = 0;
            for (size_t j = 0; j < centers[i].size(); j++) {
                sumPoint += centers[i][j].confidence * centers[i][j].location;
                normalizer += centers[i][j].confidence;
            }
            sumPoint *= (1. / normalizer);
            KeyPoint kpt(sumPoint, (float) (centers[i][centers[i].size() / 2].radius * 2.0));
            keypoints.push_back(kpt);
        }

#ifdef DEBUG_CIRCLE_DETECTOR
        //namedWindow("keypoints", CV_WINDOW_NORMAL);
        Mat outImg = image.clone();
        for (size_t i = 0; i < keypoints.size(); i++) {
            circle(outImg, keypoints[i].pt, keypoints[i].size, Scalar(255, 0, 255), -1);
        }
        // drawKeypoints(image, keypoints, outImg);
        //imshow("keypoints", outImg);
        //waitKey();
#endif
    }

    Ptr<CircleDetector> CircleDetector::create(const CircleDetector::Params& params) {
        return makePtr<CircleDetectorImpl>(params);
    }

} // end  namespace cv
