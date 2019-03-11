#ifndef SRC_GRAPH_HPP_
#define SRC_GRAPH_HPP_

#include <iostream>

#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

template <typename T>
class EquiGraph {
public:
	std::shared_ptr<cv::Mat> equi1_t { };
	std::shared_ptr<cv::Mat> equi2_t { };

	std::vector<std::vector<T>> equi1_t_g {};
	std::vector<std::vector<T>> equi2_t_g {};

	std::vector<std::vector<T>> disparity_labels{};

	double width {};
	double height {};

	EquiGraph(std::shared_ptr<cv::Mat> equi1_t, std::shared_ptr<cv::Mat> equi2_t);
	void Convert2Gray (std::shared_ptr<cv::Mat> &img_p, std::vector<std::vector<T>> &img_gray);
	void Convert2Gray1();
	void Convert2Gray2();
	void Compute();

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

	for (int y { 0 }; y < height; ++y) {

		std::vector<T> row{};

		for (int x { 0 }; x < width; ++x) {

			cv::Vec3b intensity = img_p->at<cv::Vec3b>(y, x);

			int B = intensity.val[0];
			int G = intensity.val[1];
			int R = intensity.val[2];

			T gray_val = 0.2989 * R + 0.5870 * G + 0.1140 * B;

			row.push_back(gray_val);

		}

		img_gray.push_back(row);
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
void EquiGraph<T>::Compute(){

	Convert2Gray1();
	Convert2Gray2();




}



#endif /* SRC_GRAPH_HPP_ */
