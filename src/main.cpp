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

const bool PRINT {false};

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

inline void polar2spherical (const double &theta, const double &phi, cv::Matx13d &p) {
// Converts polar coordinates to spherical coordinates
// Inputs:
//		theta: scans on the x-y direction and is in the range [0,2pi]
//		phi: scans on the z direction and is in the range [-pi/2, pi/2]
// Output:
//		p: the point in spherical coordinates

	// coordinate conversion
	p(0) = { cos(theta) * cos(phi) };
	p(1) = { sin(theta) };
	p(2) = { cos(theta) * sin(phi) };


}

inline void pixelCoord2Polar (const double &x, const double &y, const double &width, const double &height, double &theta, double &phi){
// Transforms pixel coodinates into polar coordinates
// Inputs:
//		x : the x position on the equirectangular image
//		y : the y position on the equirectangular image
//		width: width of the image
//		height: height of the image
// Outputs:
//		theta: scans on the x-y direction on the spherical coordinates and is in the range of [0,2pi]
//		phi: scans on the z direction on the spherical coordinates and is in the range [-pi/2,pi/2]

	// transforms the pixel coordinate into polar
	theta = (x / width) * 2 * M_PI;
	phi = ((y / (height)) - 0.5) * M_PI;
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

void calculateOpposite (const double &theta, const double &phi, double &theta_op, double &phi_op){
// Calculates the opposite point of the sphere
// Inputs:
//		theta:
//		phi:
// Outputs:
//		theta_op:
//		phi_op:

	phi_op = -phi;

	theta_op = theta + M_PI;

	if (theta_op >= 2 * M_PI) {
		theta_op -= 2 * M_PI;
	}

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
				x_dest = width + x_dest;
			} else if (x_dest >= width) {
				x_dest = x_dest - width;
			}
			if (y_dest < 0) {
				y_dest = height + y_dest;
			} else if (y_dest >= height) {
				y_dest = y_dest - height;
			}
			posx.at<float>(y,x) = x_dest;
			posy.at<float>(y,x) = y_dest;

		}
	}

	cv::remap(image_orig, image_dest, posx, posy, cv::INTER_CUBIC, cv::BORDER_CONSTANT, cv::Scalar(0,0,0));

}

inline cv::Matx13d crossProduct (const cv::Matx13d &a, const cv::Matx13d &b) {
// Returns the cross-product between vectors a and b

	const double & a1 = a(0);
	const double & a2 = a(1);
	const double & a3 = a(2);
	const double & b1 = b(0);
	const double & b2 = b(1);
	const double & b3 = b(2);
	cv::Matx13d ab {a2*b3 - a3*b2, a3*b1 -a1*b3, a1*b2 - a2*b1};

	return ab;
}

inline double computeAngleBetween (const cv::Matx13d &a, const cv::Matx13d &b) {
// Returns the angle between vectors a and b
// Inputs:
//		a 	: unit vector
//		b 	: unit vector
// Output:
//		angle between a and b in the range [0, pi]

	// dot product between u and v
	double cosTheta { a(0)*b(0) + a(1)*b(1) + a(2)*b(2) };

	// returns the angle in rad
	return std::acos (cosTheta);

}

void computeRotationMatrix (const cv::Matx13d &e1, const cv::Matx13d &northPole, cv::Matx33d &R1NP){

	// calculate the norm of e1
	double n_e1 = sqrt(e1(0) * e1(0) + e1(1) * e1(1) + e1(2) * e1(2));
	// normalize e1
	cv::Matx13d e1_ { e1(0) / n_e1, e1(1) / n_e1, e1(2) / n_e1 };

	// calculate the norm of northPole
	double n_northPole = sqrt(northPole(0) * northPole(0) + northPole(1) * northPole(1) + northPole(2) * northPole(2));
	// normalize northPole
	cv::Matx13d northPole_ { northPole(0) / n_northPole, northPole(1) / n_northPole, northPole(2) / n_northPole };

	// The cross product between e1_ and northPole_
	cv::Matx13d p = crossProduct(e1_, northPole_);
	// Calculate the norm of p
	double n_p = sqrt(p(0) * p(0) + p(1) * p(1) + p(2) * p(2));

	// The angle between the vectors
	double theta = computeAngleBetween(e1_, northPole_);

	// Normalize p
	double ux { p(0) / n_p };
	double uy { p(1) / n_p };
	double uz { p(2) / n_p };

	double cosT = cos(theta);
	double sinT = sin(theta);

	// Calculate the rotation matrix
	R1NP(0, 0) = cosT + ux * ux * (1 - cosT);
	R1NP(0, 1) = ux * uy * (1 - cosT) - uz * sinT;
	R1NP(0, 2) = ux * uz * (1 - cosT) + uy * sinT;
	R1NP(1, 0) = uy * ux*(1 - cosT) + uz * sinT;
	R1NP(1, 1) = cosT + uy * uy * (1 - cosT);
	R1NP(1, 2) = uy * uz * (1 - cosT) - ux * sinT;
	R1NP(2, 0) = uz * ux * (1 - cosT) - uy * sinT;
	R1NP(2, 1) = uz * uy * (1 - cosT) + ux * sinT;
	R1NP(2, 2) = cosT + uz * uz * (1 - cosT);

}

void rotateImages (const cv::Mat &image_orig, cv::Mat &image_dest,
				   const cv::Matx33d &R) {
// Rotate image based on the rotation matrix R
// Inputs:
//		image_orig	: the original image to transform
//		R			: rotation matrix
// Output:
//		image_dest: the transformed image

	int width = image_orig.cols;
	int height = image_orig.rows;

	cv::Mat posx (image_orig.size(), CV_32FC1);
	cv::Mat posy (image_orig.size(), CV_32FC1);

	// Goes through each point of the destination image and find the corresponding x and y coordinate
	for (int y { 0 }; y < height; ++y) {
		for (int x { 0 }; x < width; ++x) {

//	int y { static_cast<int> (.5 * height)};
//	int x { static_cast<int> (3. / 4. * width) };

//	int y { static_cast<int>(0.5 * height) };
//	int x { static_cast<int>(0) };

//	int y { static_cast<int>(0.5 * height) };
//	int x { static_cast<int>(0.25 * width) };


//	int y { static_cast<int>(0.5 * height) };
//	int x { static_cast<int>(0.5 * width) };

//	int y { static_cast<int>(0) };
//	int x { static_cast<int>(0.0 * width) };

//	int y { static_cast<int>(height-1) };
//	int x { static_cast<int>(0.5 * width) };

//	std::cout << "height: " << height << std::endl;
//	std::cout << "width: " << width << std::endl;
//	std::cout << "(" << x << "," << y << ")" << std::endl;

//	{
//		{

			// transforms the pixel coordinate into polar
			double theta { };
			double phi { };

			// Converts pixel coordinates to polar coordinates
			pixelCoord2Polar (x, y, width, height, theta, phi);

			if (PRINT) {
				std::cout << "theta = " << theta << std::endl;
				std::cout << "phi = " << phi << std::endl;
			}

			cv::Matx13d p_orig { };

			// Converts from polar to spherical coordinates
			polar2spherical (theta, phi, p_orig);

			if (PRINT) {
				std::cout << "x = " << p_orig(0) << std::endl;
				std::cout << "y = " << p_orig(1) << std::endl;
				std::cout << "z = " << p_orig(2) << std::endl;
			}

			// Applies rotation
			cv::Matx13d p_dest {(R.inv() * p_orig.t()).t()};

			if (PRINT) {
				std::cout << R.inv();
				std::cout << std::endl;
			}

			if (PRINT) {
				std::cout << "x_dest = " << p_dest(0) << std::endl;
				std::cout << "y_dest = " << p_dest(1) << std::endl;
				std::cout << "z_dest = " << p_dest(2) << std::endl;
			}

			double theta_dest{};
			double phi_dest{};
			// Converts from spherical to polar coordinates
			spherical2polar (p_dest, theta_dest, phi_dest);

			if (PRINT) {
				std::cout << "theta_dest = " << theta_dest << std::endl;
				std::cout << "phi_dest = " << phi_dest << std::endl;
			}

			cv::Point2d p_xy_dest {};

			polar2cartesianEqui (theta_dest, phi_dest, width, height, p_xy_dest);

			if (PRINT) {
				std::cout << "x_equi_dest = " << p_xy_dest.x << std::endl;
				std::cout << "y_equi_dest = " << p_xy_dest.y << std::endl;
			}

			posx.at<float>(y,x) = p_xy_dest.x;
			posy.at<float>(y,x) = p_xy_dest.y;

		}
	}


	cv::remap(image_orig, image_dest, posx, posy, cv::INTER_CUBIC, cv::BORDER_CONSTANT, cv::Scalar(0,0,0));

}


const int width { 6016 };
const int height { 3008 };

int main() {

	cv::Mat image1{};
	image1 = cv::imread("./data_in/equi1.bmp");
	//image1 = cv::imread("./data_in/20181218-160912-843294.bmp");

	cv::Mat image2{};
	image2 = cv::imread("./data_in/equi2.bmp");
	//image2 = cv::imread("./data_in/20181218-160924-593294.bmp");

	if (!image1.data) {
		std::cout << " Could not open or find the image" << std::endl;
		return -1;
	}
	if (!image2.data) {
		std::cout << " Could not open or find the image" << std::endl;
		return -1;
	}





	// Rotation and translation matrices between the images
	cv::Matx33d R = cv::Matx33d::eye();
	cv::Matx13d t {0, 50, 0};

//	cv::Matx33d R = {0.998933282130651, 0.0451650170386429, 0.0096134846744263,
//					-0.0449238715149459, 0.998702703733909, -0.023974055199083,
//					-0.0106838017482054, 0.0235166066956765, 0.999666357136083};
//	cv::Matx13d t {0.0448056518887687, 0.647316427094443, -0.0919317133950784};


	// Epipoles of the images
	cv::Matx13d e1 { };
	cv::Matx13d e2 { };

	// Compute the epipoles of the two images
	computeEpipole (R, t, e1, e2);

	//==============================================================
	// Calculate the polar angles that corresponds to epipole e1
	double theta1 {};
	double phi1 {};

	// Point in cartesian coordinates of the epipole e1 on the equirectangular image 1
	cv::Point2d p1{};

	spherical2polar(e1, theta1, phi1);
	polar2cartesianEqui(theta1, phi1, width, height, p1);

	std::cout << p1.x << std::endl;
	std::cout << p1.y << std::endl;

	cv::circle(image1, p1, 50, cv::Scalar(0,0,255), -1);

	// Calculate the polar angles that corresponds to epipole e2
	double theta2 {};
	double phi2 {};

	// Point in cartesian coordinates of the epipole e2 on the equirectangular image 2
	cv::Point2d p2{};

	spherical2polar(e2, theta2, phi2);
	polar2cartesianEqui(theta2, phi2, width, height, p2);

	cv::circle(image2, p2, 50, cv::Scalar(0,0,255), -1);

	cv::namedWindow("Display window1", cv::WINDOW_KEEPRATIO);
	cv::imshow ("Display window1", image1);
	cv::namedWindow("Display window2", cv::WINDOW_KEEPRATIO);
	cv::imshow ("Display window2", image2);
	//==============================================================

	// North pole
	const cv::Matx13d northPole { 0, 0, 1 };

	// The rotation matrix to move e1 to the north pole
	cv::Matx33d R1NP { };

	// Compute the rotation matrix
	computeRotationMatrix(e1, northPole, R1NP);

	// Create the destination image
	cv::Mat image1_n {};
	image1_n.create(image1.size(), image1.type());

	// Rotates the images using the calculated rotation matrix
	rotateImages (image1, image1_n, R1NP);

	cv::namedWindow("Display window1_n", cv::WINDOW_KEEPRATIO);
	cv::imshow ("Display window1_n", image1_n);


	// Calculates the opposit of the epipole p2
	double theta2_b { };
	double phi2_b { };
	calculateOpposite(theta2, phi2, theta2_b, phi2_b);

	cv::Matx13d e2_b {};
	// Converts from polar to spherical coordinates
	polar2spherical (theta2_b, phi2_b, e2_b);

	// The rotation matrix to move e1 to the north pole
	cv::Matx33d R2NP { };

	// Compute the rotation matrix
	computeRotationMatrix(e2_b, northPole, R2NP);

	// Create the destination image
	cv::Mat image2_n { };
	image2_n.create(image2.size(), image2.type());

	// Rotates the images using the calculated rotation matrix
	rotateImages(image2, image2_n, R2NP);

	cv::namedWindow("Display window2_n", cv::WINDOW_KEEPRATIO);
	cv::imshow("Display window2_n", image2_n);

	cv::waitKey(0);


	cv::imwrite("image1.jpg",image1);
	cv::imwrite("image2.jpg",image2);
	cv::imwrite("image1_t.jpg",image1_n);
	cv::imwrite("image2_t.jpg",image2_n);


	return 0;

}
