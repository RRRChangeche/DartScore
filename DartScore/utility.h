#pragma once
#include "yolo_v2_class.hpp"    // imported functions from DLL

void draw_scoreArea(cv::Mat& mat_img, std::vector<bbox_t> result_vec, 
	std::vector<std::string> obj_names, std::map<std::string, std::pair<int,int>> calPoints_stdmap, cv::Mat& M, float& scale);

void draw_darts(cv::Mat mat_img, std::vector<bbox_t> result_vec,
	std::vector<std::string> obj_names, cv::Mat M, float& scale);

void crop_dartBoard_by_calibratedPoints(cv::Mat& mat_img, std::map<std::string, std::pair<int, int>> calPoints_stdmap);
