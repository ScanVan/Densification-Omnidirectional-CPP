#ifndef SRC_TRANSFORMATION_HPP_
#define SRC_TRANSFORMATION_HPP_

#include <iostream>

#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>

#include "cartesian2Spherical.hpp"
#include <cmath>
#include <memory>

template <typename T>
class EquiPair {
public:
	std::shared_ptr<cv::Mat> equi1 { };
	std::shared_ptr<cv::Mat> equi2 { };
	cv::Matx<T,3,3> R { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
	cv::Matx<T,1,3> t { 0, 0, 0 };
	cv::Matx<T,1,3> e1 { 0, 0, 0 };
	cv::Matx<T,1,3> e2 { 0, 0, 0 };
	T e1_theta { 0 };
	T e1_phi { 0 };
	T e2_theta { 0 };
	T e2_phi { 0 };
	T e1_x_equi { 0 };
	T e1_y_equi { 0 };
	T e2_x_equi { 0 };
	T e2_y_equi { 0 };
	double width { 0 };
	double height { 0 };
	const cv::Matx<T,1,3> northPole { 0, 0, 1 };
	cv::Matx<T,3,3> R1NP { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	cv::Matx<T,3,3> R2NP { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	cv::Matx<T,3,3> R1NPinv { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	cv::Matx<T,3,3> R2NPinv { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	cv::Mat equi1_rot { };
	cv::Mat equi2_rot { };

	EquiPair(std::shared_ptr<cv::Mat> equi1, std::shared_ptr<cv::Mat> equi2, cv::Matx<T,3,3> R, cv::Matx<T,1,3> t);

	void computeEpipole (const cv::Matx<T,3,3> &R, const cv::Matx<T,1,3> &t, cv::Matx<T,1,3> &e1, cv::Matx<T,1,3> &e2);

	void Spherical2Polar (const cv::Matx<T,1,3> &v, T &theta, T &phi);
	void Polar2Spherical (const T &theta, const T &phi, cv::Matx<T,1,3> &p);

	void Polar2CartesianEqui (const T &theta, const T &phi, T &x, T &y);
	void CartesianEqui2Polar (const T &x, const T &y, T &theta, T &phi);

	void computeRotationMatrix (const cv::Matx<T,1,3> &epipole, const cv::Matx<T,1,3> &northPole, cv::Matx<T,3,3> &RotM);
	inline cv::Matx<T,1,3> crossProduct (const cv::Matx<T,1,3> &a, const cv::Matx<T,1,3> &b);
	inline T computeAngleBetween (const cv::Matx<T,1,3> &a, const cv::Matx<T,1,3> &b);
	void rotateImages (const cv::Mat &image_orig, cv::Mat &image_dest, const cv::Matx<T,3,3> & Rinv);

	void polar2cartesianEqui (const T &theta, const T &phi, cv::Point_<T> &p);
	void calculateOpposite (const T &theta, const T &phi, T &theta_op, T &phi_op);

};

template<typename T>
EquiPair<T>::EquiPair(std::shared_ptr<cv::Mat> equi1, std::shared_ptr<cv::Mat> equi2, cv::Matx<T,3,3> R, cv::Matx<T,1,3> t) :
		equi1(equi1), equi2(equi2), R(R), t(t) {

	width = equi1->cols;
	height = equi1->rows;

	computeEpipole(this->R, this->t, e1, e2);
	Spherical2Polar(e1, e1_theta, e1_phi);
	Spherical2Polar(e2, e2_theta, e2_phi);
	Polar2CartesianEqui(e1_theta, e1_phi, e1_x_equi, e1_y_equi);
	Polar2CartesianEqui(e2_theta, e2_phi, e2_x_equi, e2_y_equi);
	cv::circle(*(this->equi1), cv::Point_<T>(e1_x_equi, e1_y_equi), 50, cv::Scalar(0,0,255), -1);
	cv::circle(*(this->equi2), cv::Point_<T>(e2_x_equi, e2_y_equi), 50, cv::Scalar(255,0,0), -1);

	calculateOpposite(e2_theta, e2_phi, e2_theta, e2_phi);
	Polar2Spherical(e2_theta, e2_phi, e2);
	Polar2CartesianEqui(e2_theta, e2_phi, e2_x_equi, e2_y_equi);
	cv::circle(*(this->equi2), cv::Point_<T>(e2_x_equi, e2_y_equi), 50, cv::Scalar(0,0,255), -1);

	computeRotationMatrix(e1, northPole, R1NP);
	computeRotationMatrix(e2, northPole, R2NP);

	R1NPinv = R1NP.inv();
	R2NPinv = R2NP.inv();
	equi1_rot.create(this->equi1->size(), this->equi1->type());
	rotateImages (*(this->equi1), equi1_rot, R1NPinv);
	equi2_rot.create(this->equi2->size(), this->equi2->type());
	rotateImages (*(this->equi2), equi2_rot, R2NPinv);

}

template<typename T>
inline void EquiPair<T>::calculateOpposite (const T &theta, const T &phi, T &theta_op, T &phi_op){
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

template<typename T>
void EquiPair<T>::computeEpipole (const cv::Matx<T,3,3> &R, const cv::Matx<T,1,3> &t, cv::Matx<T,1,3> &e1, cv::Matx<T,1,3> &e2) {
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

template<typename T>
inline void EquiPair<T>::Polar2Spherical (const T &theta, const T &phi, cv::Matx<T,1,3> &p) {
// Converts polar coordinates to spherical coordinates
// Inputs:
//		theta: scans on the x-y direction and is in the range [0,2pi]
//		phi: scans on the z direction and is in the range [-pi/2, pi/2]
// Output:
//		p: the point in spherical coordinates

	// coordinate conversion
	p(0) = { static_cast<T>(cos(theta) * cos(phi)) };
	p(1) = { static_cast<T>(sin(theta) * cos(phi)) };
	p(2) = { static_cast<T>( sin(phi)) };

}

template<typename T>
inline void EquiPair<T>::Spherical2Polar (const cv::Matx<T,1,3> &v, T &theta, T &phi){
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

	T x = v(0);
	T y = v(1);
	T z = v(2);

	phi = asin (z);


//	T h = acos(phi);
	T h = sqrt(x*x + y*y);

	if (y >= 0) {
		theta = acos (x / h);
	} else {
		theta = 2 * M_PI - acos (x / h);
	}

}

template <typename T>
void EquiPair<T>::Polar2CartesianEqui (const T &theta, const T &phi, T &x, T &y) {

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

	x = theta / (2 * M_PI) * width;
	y = (-phi / M_PI + 0.5) * (height -1);

}

template<typename T>
inline void EquiPair<T>::CartesianEqui2Polar (const T &x, const T &y, T &theta, T &phi){
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
	phi = -((y / (height-1)) - 0.5) * M_PI;
}


template<typename T>
void EquiPair<T>::computeRotationMatrix (const cv::Matx<T,1,3> &epipole, const cv::Matx<T,1,3> &northPole, cv::Matx<T,3,3> &RotM){

	// calculate the norm of e1
	T norm_epi = sqrt(epipole(0) * epipole(0) + epipole(1) * epipole(1) + epipole(2) * epipole(2));
	// normalize e1
	cv::Matx<T,1,3> epi { epipole(0) / norm_epi, epipole(1) / norm_epi, epipole(2) / norm_epi };

	// calculate the norm of northPole
	T n_northPole = sqrt(northPole(0) * northPole(0) + northPole(1) * northPole(1) + northPole(2) * northPole(2));
	// normalize northPole
	cv::Matx<T,1,3> northPole_ { northPole(0) / n_northPole, northPole(1) / n_northPole, northPole(2) / n_northPole };

	// The cross product between e1_ and northPole_
	cv::Matx<T,1,3> p = crossProduct(epi, northPole_);
	// Calculate the norm of p
	T n_p = sqrt(p(0) * p(0) + p(1) * p(1) + p(2) * p(2));

	// The angle between the vectors
	T theta = computeAngleBetween(epi, northPole_);

	// Normalize p
	T ux { p(0) / n_p };
	T uy { p(1) / n_p };
	T uz { p(2) / n_p };

	T cosT = cos(theta);
	T sinT = sin(theta);

	// Calculate the rotation matrix
	RotM(0, 0) = cosT + ux * ux * (1 - cosT);
	RotM(0, 1) = ux * uy * (1 - cosT) - uz * sinT;
	RotM(0, 2) = ux * uz * (1 - cosT) + uy * sinT;
	RotM(1, 0) = uy * ux*(1 - cosT) + uz * sinT;
	RotM(1, 1) = cosT + uy * uy * (1 - cosT);
	RotM(1, 2) = uy * uz * (1 - cosT) - ux * sinT;
	RotM(2, 0) = uz * ux * (1 - cosT) - uy * sinT;
	RotM(2, 1) = uz * uy * (1 - cosT) + ux * sinT;
	RotM(2, 2) = cosT + uz * uz * (1 - cosT);

}

template<typename T>
inline cv::Matx<T,1,3> EquiPair<T>::crossProduct (const cv::Matx<T,1,3> &a, const cv::Matx<T,1,3> &b) {
// Returns the cross-product between vectors a and b

	const T & a1 = a(0);
	const T & a2 = a(1);
	const T & a3 = a(2);
	const T & b1 = b(0);
	const T & b2 = b(1);
	const T & b3 = b(2);
	cv::Matx<T,1,3> ab {a2*b3 - a3*b2, a3*b1 -a1*b3, a1*b2 - a2*b1};

	return ab;
}

template<typename T>
inline T EquiPair<T>::computeAngleBetween (const cv::Matx<T,1,3> &a, const cv::Matx<T,1,3> &b) {
// Returns the angle between vectors a and b
// Inputs:
//		a 	: unit vector
//		b 	: unit vector
// Output:
//		angle between a and b in the range [0, pi]

	// dot product between u and v
	T cosTheta { a(0)*b(0) + a(1)*b(1) + a(2)*b(2) };

	// returns the angle in rad
	return std::acos (cosTheta);

}

template<typename T>
void EquiPair<T>::rotateImages (const cv::Mat &image_orig, cv::Mat &image_dest, const cv::Matx<T,3,3> & Rinv){
// Rotate image based on the rotation matrix R
// Inputs:
//		image_orig	: the original image to transform
//		Rinv		: rotation matrix
// Output:
//		image_dest: the transformed image

	cv::Mat posx (image_orig.size(), CV_32FC1);
	cv::Mat posy (image_orig.size(), CV_32FC1);

	T theta { };
	T phi { };

	T theta_dest{};
	T phi_dest{};

	cv::Matx<T,1,3> p_orig { };
	cv::Matx<T,1,3> p_dest { };

	T x_dest { };
	T y_dest { };
	cv::Point_<T> p_xy_dest { };

	// Goes through each point of the destination image and find the corresponding x and y coordinate
	for (int y { 0 }; y < height; ++y) {
		for (int x { 0 }; x < width; ++x) {


			// Converts pixel coordinates to polar coordinates
			CartesianEqui2Polar (x, y, theta, phi);

			// Converts from polar to spherical coordinates
			Polar2Spherical (theta, phi, p_orig);

			// Applies rotation
			p_dest = (Rinv * p_orig.t()).t();

			// Converts from spherical to polar coordinates
			Spherical2Polar (p_dest, theta_dest, phi_dest);

			// Converts from polar to pixel coordinates
			Polar2CartesianEqui (theta_dest, phi_dest, x_dest, y_dest);


			posx.at<float>(y,x) = x_dest;
			posy.at<float>(y,x) = y_dest;

		}
	}

	cv::remap(image_orig, image_dest, posx, posy, cv::INTER_CUBIC, cv::BORDER_CONSTANT, cv::Scalar(0,0,0));

}



#endif /* SRC_TRANSFORMATION_HPP_ */
