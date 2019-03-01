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
#include <opencv4/opencv2/calib3d.hpp>

#include <cmath>
#include "transformation.hpp"
#include <memory>


int main() {

	cv::Mat image1{};
	cv::Mat image2{};

	// Read images from file
	image1 = cv::imread("./data_in/equi1.bmp");
	image2 = cv::imread("./data_in/equi2.bmp");
	// Rotation and translation matrices between the images
	cv::Matx33d R = cv::Matx33d::eye();
	cv::Matx13d t {0, 50, 0};

//	// Read images from file
//	image1 = cv::imread("./data_in/20181218-160912-843294.bmp");
//	image2 = cv::imread("./data_in/20181218-160924-593294.bmp");
//	// Rotation and translation matrices between the images
//	cv::Matx33d R = {0.998927507290289, 0.0452783759684353, 0.00968007481742678,
//					-0.0450349994790951, 0.998696271463141, -0.0240334389453134,
//					-0.0107556497120284, 0.0235717210928949, 0.999664288630934};
//	cv::Matx13d t {0.0443775171597948, 0.647351744454243, -0.0919138857448478};


	if (!image1.data) {
		std::cout << " Could not open or find the image" << std::endl;
		return -1;
	}
	if (!image2.data) {
		std::cout << " Could not open or find the image" << std::endl;
		return -1;
	}

	std::shared_ptr<cv::Mat> pImg1 = std::make_shared<cv::Mat>(image1);
	std::shared_ptr<cv::Mat> pImg2 = std::make_shared<cv::Mat>(image2);

	EquiPair<double> eqp {pImg1, pImg2, R, t};

	eqp.Convert2Gray();

	cv::namedWindow("Display window1", cv::WINDOW_KEEPRATIO);
	cv::imshow("Display window1", *(eqp.equi1));
	cv::namedWindow("Display window2", cv::WINDOW_KEEPRATIO);
	cv::imshow("Display window2", *(eqp.equi2));

	cv::namedWindow("Display window1_n", cv::WINDOW_KEEPRATIO);
	cv::imshow("Display window1_n", eqp.equi1_rot_g);
	cv::namedWindow("Display window2_n", cv::WINDOW_KEEPRATIO);
	cv::imshow("Display window2_n", eqp.equi2_rot_g);

	cv::waitKey(0);




	cv::imwrite("image1.jpg",*(eqp.equi1));
	cv::imwrite("image2.jpg",*(eqp.equi2));
	cv::imwrite("image1_t.jpg",eqp.equi1_rot);
	cv::imwrite("image2_t.jpg",eqp.equi2_rot);


	return 0;

}
