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


	cv::namedWindow("Display window1", cv::WINDOW_KEEPRATIO);
	cv::imshow ("Display window1", image1);
	cv::namedWindow("Display window2", cv::WINDOW_KEEPRATIO);
	cv::imshow ("Display window2", image2);
	cv::waitKey(0);



	return 0;

}
