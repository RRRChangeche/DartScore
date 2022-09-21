#pragma once
#include "yolo_v2_class.hpp"    // imported functions from DLL

class DartBoard {
public:
	DartBoard();
	~DartBoard();

	void draw_scoreArea(cv::Mat& mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names);

	void draw_darts(cv::Mat mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names);

	void crop_dartBoard_by_calibratedPoints(cv::Mat& mat_img);

private:
	std::map<std::string, std::pair<int, int>> calibratePoints_std;	// standard calibrate points map
	cv::Point2f dstPoints[4];	// standard calibrate points 
	cv::Point2f srcPoints[4];	// perspective calibrate points from yolo
	//vector<cv::Point2f> dartPoints;	// dart points
	cv::Mat M;		// perspective transformation matrix 
	float scale;	// scale from source img size to standard img size
	float Rscale[6] = { 12.7, 32, 182, 214, 308, 340 };	// score area radius
	float R;		// outer radius of score area
	cv::Point center;	// center of dartboard
	std::unordered_map<int, int> scoreMap;	// score map to location on the dartboard
};
