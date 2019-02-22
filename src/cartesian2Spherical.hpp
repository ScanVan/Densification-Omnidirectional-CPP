#ifndef SRC_CARTESIAN2SPHERICAL_HPP_
#define SRC_CARTESIAN2SPHERICAL_HPP_

#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>

cv::Matx13d convertCartesian2Spherical (double x, double y, int width, int height) {
// Inputs:
// x 				: cartesian x-coordinate
// y				: cartesian y-coordinate
// width			: width of the equirectangular image
// height			: height of the equirectangular image
// Outputs:
// Points<double>	: converted spherical coordinates in type Points<double>

	// coordinate re-normalization
	double tm1 { ((x - 1) / width) * 2 * M_PI };
	double tm2 { ((y / height) - 0.5) * M_PI };


	// coordinate conversion
	double p1 { cos(tm2) * cos(tm1) };
	double p2 { cos(tm2) * sin(tm1) };
	double p3 { sin(tm2) };

	cv::Matx13d  p { p1, p2, p3 };

	return p;

}


#endif /* SRC_CARTESIAN2SPHERICAL_HPP_ */
