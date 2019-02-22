//============================================================================
// Name        : main.cpp
// Author      : Marcelo E. Kaihara
// Version     :
// Copyright   :
// Description : Transforms the equirectangular image by applying rotations
//============================================================================

#include <iostream>

#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>

#include "cartesian2Spherical.hpp"
#include <cmath>

void computeEpipole (const cv::Matx33d &R, const cv::Matx13d &t,
					  cv::Matx13d &e1, cv::Matx13d &e2) {
// Computes the epipoles on a pair of images
// Inputs:
// 		R 	:the rotation matrix
// 		t 	:the translation vector
// Outputs:
//		e1 	: the epipole position in sphere 1
//		e2	: the epipole position in sphere 2

	// compute epipole position in sphere 1 - in the frame of sphere 1
	e1 = (-R.t() * (t * (1 / cv::norm(t))).t()).t();

	// compute epipole position in sphere 2 - in the frame of sphere 2
	e2 = t * (1 / cv::norm(t));

}

void spherical2polar (const cv::Matx13d &v, double &theta, double &phi){
// Converts from spherical coordinates to polar coordinates
// Input:
//	v	: spherical coordinates
// Outputs:
//	theta	: the angle that corresponds to the x-direction on an equirectangular image, it goes from 0 (left) to 2pi (right)
// 	rho		: the angle that corresponds to the y-direction on an equirectangular image, it goes from pi/2 (top) to -pi/2 (bottom)
//
// Remarks:
//
//		    y
//         pi/2
//	    	|
//		    |
//   pi ----------- 0 x
//   		|
// 	    	|
//     	  3/2*pi
//
//			z
//		   pi/2
// 			|
//          |
//  -----------------
//			|
//			|
//		  -pi/2


	// normalize the vector
	cv::Matx13d v1 {v * (1 / cv::norm(v)) };

	double y = v1(1);
	double x = v1(0);
	double z = v1(2);

	double h = sqrt(x*x + y*y);

	if (y >= 0) {
		theta = acos (x / h);
	} else {
		theta = 2 * M_PI - acos (x / h);
	}

	phi =asin (z);

}

void polar2cartesianEqui (const double &theta, const double &phi, const int &width, const int &height,
						  cv::Point2d &p) {
// converts the polar coordinates into cartesian coordinates of the equirectangular image
// Inputs:
//		theta: 	goes from 0 to 2*pi, direction of x of the equirectangular image
//		phi	 :  goes from -pi/2 to pi/2, direction of y of the equirectangular image
//		width:  the width of the equirectangular image
//		height: the height of the equirectangular image
// Outputs:
//		x	 : x-coordinate of the equirectangular image [0, width -1]
// 		y	 : y-coordinage of the equirectangular image [0, height -1], 0 corresponds to top
//
// Remarks:
//      phi = pi/2 corresponds to y = 0
//      phi = -pi/2 corresponds to height
//		theta = 0 corresponds to x = 0
//		theta = 2*pi corresponds to x = width

	double x { theta / (2 * M_PI) * width };
	double y { (-phi / M_PI + 0.5) * height };

	p.x = x;
	p.y = y;

}

void alignImages (const cv::Mat &image_orig, cv::Mat &image_dest,
				  const cv::Point2d &p1, const cv::Point2d &p2) {
// Aligns the image by means of shift so that point p2 is aligned to p1
// Inputs:
//		image_orig	: the original image to transform
//		p1			: cartesian coordinate of reference point
//		p2			: cartesian coordinate of the mapped p1 point on destination image
// Output:
//		image_dest: the transformed image

	int width = image_orig.cols;
	int height = image_orig.rows;

	cv::Point2d shift { };
	shift.x = p2.x - p1.x;
	shift.y = p2.y - p1.y;

	cv::Mat posx (image_orig.size(), CV_32FC1);
	cv::Mat posy (image_orig.size(), CV_32FC1);

	// Goes through each point of the destination image and find the corresponding x and y coordinate
	for (int y { 0 }; y < height; ++y) {
		for (int x { 0 }; x < width; ++x) {
			double x_dest { x - shift.x };
			double y_dest { y - shift.y };
			if (x_dest < 0) {
				x_dest = width - x_dest;
			} else if (x_dest >= width) {
				x_dest = x_dest - width;
			}
			if (y_dest < 0) {
				y_dest = height - y_dest;
			} else if (y_dest >= height) {
				y_dest = y_dest - height;
			}
			posx.at<float>(y,x) = x_dest;
			posy.at<float>(y,x) = y_dest;

		}
	}

	cv::remap(image_orig, image_dest, posx, posy, cv::INTER_CUBIC, cv::BORDER_CONSTANT, cv::Scalar(0,0,0));

}

int main() {

	cv::Mat image1{};
	image1 = cv::imread("./data_in/equi1.bmp");
	cv::Mat image2{};
	image2 = cv::imread("./data_in/equi2.bmp");

	if (!image1.data) {
		std::cout << " Could not open or find the image" << std::endl;
		return -1;
	}
	if (!image2.data) {
		std::cout << " Could not open or find the image" << std::endl;
		return -1;
	}




	cv::Matx33d R = cv::Matx33d::eye();
	cv::Matx13d t {0, 50, 0};

	cv::Matx13d e1 { };
	cv::Matx13d e2 { };

	computeEpipole (R, t, e1, e2);

	double theta1 {};
	double phi1 {};

	int width { 6016 };
	int height { 3008 };
	cv::Point2d p1{};

	spherical2polar(e1, theta1, phi1);
	polar2cartesianEqui(theta1, phi1, width, height, p1);

	cv::circle(image1, p1, 10, cv::Scalar(0,0,255), -1);

	double theta2 {};
	double phi2 {};
	cv::Point2d p2{};

	spherical2polar(e2, theta2, phi2);
	polar2cartesianEqui(theta2, phi2, width, height, p2);

	cv::circle(image2, p2, 10, cv::Scalar(0,0,255), -1);

	cv::namedWindow("Display window1", cv::WINDOW_KEEPRATIO);
	cv::imshow ("Display window1", image1);
	cv::namedWindow("Display window2", cv::WINDOW_KEEPRATIO);
	cv::imshow ("Display window2", image2);


	cv::Mat image3 {};
	image3.create(image2.size(), image2.type());

	alignImages (image2, image3, p1, p2);

	cv::namedWindow("Display window3", cv::WINDOW_KEEPRATIO);
	cv::imshow ("Display window3", image3);

	cv::waitKey(0);

	return 0;

}
