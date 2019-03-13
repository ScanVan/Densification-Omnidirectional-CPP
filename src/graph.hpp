#ifndef SRC_GRAPH_HPP_
#define SRC_GRAPH_HPP_

#include <iostream>
#include <fstream>
#include <algorithm>

#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

#include "mincut.hpp"

template <typename T>
class EquiGraph {
public:
	std::shared_ptr<cv::Mat> equi1_t { };
	std::shared_ptr<cv::Mat> equi2_t { };

	std::vector<std::vector<T>> equi1_t_g {};
	std::vector<std::vector<T>> equi2_t_g {};

	std::vector<std::vector<T>> disp_map {};

	std::vector<std::vector<std::vector<T>>> disparity_labels{};

	double width {};
	double height {};

	EquiGraph(std::shared_ptr<cv::Mat> equi1_t, std::shared_ptr<cv::Mat> equi2_t);
	void Convert2Gray (std::shared_ptr<cv::Mat> &img_p, std::vector<std::vector<T>> &img_gray);
	void Convert2Gray1();
	void Convert2Gray2();
	void Compute();
	T Dissimilarity(T IL_m1, T IL, T IL_p1, T IR_m1, T IR, T IR_p1);

};

template<typename T>
EquiGraph<T>::EquiGraph(std::shared_ptr<cv::Mat> equi1_t, std::shared_ptr<cv::Mat> equi2_t) :
		equi1_t(equi1_t), equi2_t(equi2_t) {

	width = equi1_t->cols;
	height = equi1_t->rows;

}

template<typename T>
void EquiGraph<T>::Convert2Gray (std::shared_ptr<cv::Mat> &img_p, std::vector<std::vector<T>> &img_gray) {

	img_gray.clear();

	for (int x { 0 }; x < width; ++x) {

		std::vector<T> col { };

		for (int y { 0 }; y < height; ++y) {

			cv::Vec3b intensity = img_p->at<cv::Vec3b>(y, x);

			int B = intensity.val[0];
			int G = intensity.val[1];
			int R = intensity.val[2];

			T gray_val = 0.2989 * R + 0.5870 * G + 0.1140 * B;

			col.push_back(gray_val);

		}

		img_gray.push_back(col);
	}
}


template<typename T>
void EquiGraph<T>::Convert2Gray1(){
	EquiGraph<T>::Convert2Gray(equi1_t, equi1_t_g);
}

template<typename T>
void EquiGraph<T>::Convert2Gray2(){
	EquiGraph<T>::Convert2Gray(equi2_t, equi2_t_g);
}

template<typename T>
T EquiGraph<T>::Dissimilarity(T IL_m1, T IL, T IL_p1, T IR_m1, T IR, T IR_p1) {
// Formulae from "A Pixel Dissimilarity Measure That Is Insensitive to Image Sampling"
// S. Birchfield and C. Tomasi
// Inputs:
// 		IL_m1 	: Pixel intensity of the left image at position x_L - 1
//		IL		: Pixel intensity of the left image at position x_L
//		IL_p1	: Pixel intensity of the left image at position x_L + 1
//		IR_m1	: Pixel intensity of the right image at position x_R - 1
//		IR 		: Pixel intensity of the right image at position x_R
//		IR_p1	: Pixel intensity of the right image at position x_R + 1
// Output:
//		Dissimilarity between the pixels from the left and right images at position x_L and x_R respectively

	T IRm { 0.5 * (IR + IR_m1) };
	T IRp { 0.5 * (IR + IR_p1) };
	T IminR = std::min<T>( { IRm, IRp, IR });
	T ImaxR = std::max<T>( { IRm, IRp, IR });

	T d_LR = std::max<T>( { 0, (IL - ImaxR), (IminR - IL) });

	T ILm { 0.5 * (IL + IL_m1) };
	T ILp { 0.5 * (IL + IL_p1) };
	T IminL = std::min<T>( { ILm, ILp, IL });
	T ImaxL = std::max<T>( { ILm, ILp, IL });

	T d_RL = std::max<T>( { 0, (IR - ImaxL), (IminL - IR) });

	return std::min<T>( { d_LR, d_RL });

}

template<typename T>
void EquiGraph<T>::Compute(){

	Convert2Gray1();
	Convert2Gray2();

	std::vector<std::vector<T>> disparityMap{};

	int w { static_cast<int>(width) };

	for (int col { 0 }; col < w; ++col) {

		std::cout << col << std::endl;

		// Alias of the columns to consider
		std::vector<T> &col_L = equi1_t_g[col];
		std::vector<T> &col_R = equi2_t_g[col];

		// Vector of vector of disparities
		std::vector<std::vector<T>> dvecvec { };

		// Copy the columns to new vectors to deal with boundaries
		std::vector<T> colL { };
		std::vector<T> colR { };

		colL.reserve(col_L.size() + 2);
		colR.reserve(col_R.size() + 2);

		colL.push_back(*(col_L.end() - 1));
		std::copy(col_L.begin(), col_L.end(), std::back_inserter(colL));
		colL.push_back(col_L[0]);

		colR.push_back(*(col_R.end() - 1));
		std::copy(col_R.begin(), col_R.end(), std::back_inserter(colR));
		colR.push_back(col_R[0]);

//		std::cout << "x = ";
//		for (auto x : colL) {
//			std::cout << x << " ";
//		}
//		std::cout << std::endl;
//
//		std::cout << "y = ";
//		for (auto x : colR) {
//			std::cout << x << " ";
//		}
//		std::cout << std::endl;

		for (int iL { 1 }; iL < colL.size() - 1; ++iL) {

			std::vector<T> dvec = { };

			for (int iR { iL }; iR < colR.size() - 2 + iL; ++iR) {

				int iR_ { iR };
				if (iR > colR.size() - 2) {
					iR_ -= colR.size() - 2;
				}

//				std::cout << "(" << iL << ", " << iR_ << ") ";

				dvec.push_back(Dissimilarity(colL[iL - 1], colL[iL], colL[iL + 1], colR[iR_ - 1], colR[iR_], colR[iR_ + 1]));

			}
//			std::cout << std::endl;
			dvecvec.push_back(dvec);
		}

		if (col == 0) {
			for (auto &x : dvecvec) {
				for (auto &y : x) {
					std::cout << y << "\t ";
				}
				std::cout << std::endl;
			}
		}

		std::vector<T> disparityMapCol{};

		for (auto &x : dvecvec) {

			auto it = std::min_element(x.begin(), x.end());
			disparityMapCol.push_back(std::distance(x.begin(), it));

		}

		disparityMap.push_back(disparityMapCol);

	}

	std::ofstream f ("d_map.txt",std::ios::trunc);

	if (!f.good()) {
		throw (std::runtime_error("Error opening the disparity file."));
	}

	f << "d=[";

	for (int y { 0 }; y < height; ++y) {
		for (int x { 0 }; x < width -1; ++x) {
			f << disparityMap[x][y] << ", ";
		}
		f << disparityMap[width -1][y] << ";" << std::endl;

	}

	f << "]" << std::endl;

	f.close();


	// Let us create a graph shown in the above example
	int graph[V][V] = { {0, 16, 13, 0, 0, 0},
						{0, 0, 10, 12, 0, 0},
						{0, 4, 0, 0, 14, 0},
						{0, 0, 9, 0, 0, 20},
						{0, 0, 0, 7, 0, 4},
						{0, 0, 0, 0, 0, 0}
  					};

	minCut(graph, 0, 5);


}



#endif /* SRC_GRAPH_HPP_ */
