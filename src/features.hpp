#ifndef SRC_FEATURES_HPP_
#define SRC_FEATURES_HPP_

#include <iostream>

#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include "gms_matcher.h"

//#define USE_AKAZE_FEATURE
#define USE_ORB_FEATURE

class EquiFeatures {
public:
	std::shared_ptr<cv::Mat> equi1_t { };
	std::shared_ptr<cv::Mat> equi2_t { };
	std::shared_ptr<cv::Mat> mask {};

	cv::Mat sec1 {};
	cv::Mat sec2 {};

	std::vector<cv::KeyPoint> kpts1;
	std::vector<cv::KeyPoint> kpts2;
	cv::Mat desc1 { };
	cv::Mat desc2 { };
	std::vector<cv::DMatch> matches;

	int numSections { 10 };
	double toleranceX { 20 };
	double toleranceFactorY { 1.2 };

	EquiFeatures(std::shared_ptr<cv::Mat> equi1_t, std::shared_ptr<cv::Mat> equi2_t);
	void setMask(std::shared_ptr<cv::Mat> mask) {this->mask = mask;}
	void extractSection();
	void extractFeatures();
	void matchFeatures();
	void drawMatches();

};

EquiFeatures::EquiFeatures(std::shared_ptr<cv::Mat> equi1_t, std::shared_ptr<cv::Mat> equi2_t) :
		equi1_t(equi1_t), equi2_t(equi2_t) {
}

void EquiFeatures::extractSection() {

	const int width {300};

	sec1.create(cv::Size(width, (*equi1_t).rows), (*equi1_t).type());
	sec2.create(cv::Size(width, (*equi2_t).rows), (*equi2_t).type());

	cv::Rect Region1(cv::Point(width,0), cv::Size(width,(*equi1_t).rows));
	cv::Rect Region2(cv::Point(0,0), cv::Size(width,(*equi1_t).rows));
	(*equi1_t)(Region1).copyTo(sec1(Region2));
	(*equi2_t)(Region1).copyTo(sec2(Region2));

}

void EquiFeatures::extractFeatures(){


#ifdef USE_ORB_FEATURE
	cv::Ptr<cv::ORB> orb = cv::ORB::create(10000);
	orb->setFastThreshold(0);
	orb->detectAndCompute(sec1, cv::noArray(), kpts1, desc1);
	orb->detectAndCompute(sec2, cv::noArray(), kpts2, desc2);
#endif

#ifdef USE_AKAZE_FEATURE
	//cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.0005f, 4, 4, cv::KAZE::DIFF_PM_G2);

	// The following parameters worked for the sequence 1-34 of the Sion Outdoor dataset
	cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.00025f, 4, 4, cv::KAZE::DIFF_PM_G2);

	akaze->detectAndCompute(sec1, cv::noArray(), kpts1, desc1);
	akaze->detectAndCompute(sec2, cv::noArray(), kpts2, desc2);
#endif

}

void EquiFeatures::matchFeatures() {

	std::vector<cv::DMatch> matches_all {};
	std::vector<cv::DMatch> matches_inliner {};
	cv::BFMatcher matcher(NORM_HAMMING, true);
	// Second param is boolean variable, crossCheck which is false by default. If it is true, Matcher returns only those matches with value (i,j) such that i-th descriptor in set A has j-th descriptor in set B as the best match and vice-versa. That is, the two features in both sets should match each other. It provides consistent result, and is a good alternative to ratio test proposed by D.Lowe in SIFT paper.
	matcher.match(desc1, desc2, matches_all);


	// GMS filter
	std::vector<bool> vbInliers;
	gms_matcher gms(kpts1, sec1.size(), kpts2, sec2.size(), matches_all);
	//int num_inliers = gms.GetInlierMask(vbInliers, false, false);
	gms.GetInlierMask(vbInliers, false, false);

	// collect matches
	for (size_t i = 0; i < vbInliers.size(); ++i) {
		// if inliner, then push into valid matches
		if (vbInliers[i] == true) {
			matches.push_back(matches_all[i]);
		}
	}

/*	// Calculate the absolute distances in the Y-direction of the matched features
	std::vector<double> distancesY {};
	for (const auto &m:matches_inliner) {
		auto keypoint1 = kpts1[m.queryIdx].pt;
		auto keypoint2 = kpts2[m.trainIdx].pt;
		distancesY.push_back(keypoint1.y-keypoint2.y);
	}

	// Calculate the mean value of the distances
	double mean { std::accumulate(distancesY.begin(), distancesY.end(), 0.0) / distancesY.size() };

	// Calculate variance of the distances
	double var { 0.0 };
	for (const auto & val : distancesY) {
		var += pow(val - mean, 2);
	}
	var /=(distancesY.size() - 1);

	// Calculate the std of the distances
	double std { sqrt(var) };*/

/*


	// check consistency of matches
	for (const auto &m:matches_inliner) {
		auto keypoint1 = kpts1[m.queryIdx].pt;
		auto keypoint2 = kpts2[m.trainIdx].pt;
		//if ((abs(keypoint1.x-keypoint2.x) < toleranceX)) {
		//if ((abs(keypoint1.x-keypoint2.x) < toleranceX)&&((keypoint1.y-keypoint2.y) - mean < std * toleranceFactorY)) {
		{
			matches.push_back(m);
		}

	}
*/

	std::cout << "Get total " << matches.size() << " matches." << std::endl;

}

void EquiFeatures::drawMatches() {

	cv::Mat outImg{};

	cv::drawMatches(sec1, kpts1, sec2, kpts2, matches, outImg);

	cv::imwrite("matches.jpg",outImg);
	namedWindow("Matches", cv::WINDOW_NORMAL);
	imshow ("Matches", outImg);
	waitKey(0);
}


#endif /* SRC_FEATURES_HPP_ */
