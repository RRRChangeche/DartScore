//#include <map>
//#include <string>
#include <iostream>
#include "utility.h";

using namespace std;

#ifdef OPENCV

void draw_scoreArea(cv::Mat mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names, std::map<std::string, std::pair<int, int>> calPoints_stdmap, cv::Mat& M)
{
	// get calibrate points
	std::map<std::string, bbox_t> calPoints_map;
	//vector<bbox_t> calPoints;
	cv::Point2f dstPoints[4];
	cv::Point2f srcPoints[4];
	for (auto& i : result_vec) {
		if (obj_names.size() > i.obj_id) {
			string obj_name = obj_names[i.obj_id];
			//if (calPoints_stdmap.count(obj_name)) calPoints_map[obj_name] = i;
			cv::Point2f p = cv::Point2f(i.x + i.w / 2, i.y + i.h / 2);
			if (obj_name == "topP") srcPoints[0] = p;
			else if (obj_name == "bottomP") srcPoints[1] = p;
			else if (obj_name == "leftP") srcPoints[2] = p;
			else if (obj_name == "rightP") srcPoints[3] = p;
		}
	}
	bool isAllCalPointsCatched = true;
	for (auto& i : calPoints_stdmap) {
		/*if (calPoints_map.count(i.first)) {
			calPoints.push_back(calPoints_map[i.first]);
		}*/
		cv::Point2f p = cv::Point2f(i.second.first, i.second.second);
		if (i.first == "topP") {
			dstPoints[0] = p;
			if (srcPoints[0] == cv::Point2f(0, 0)) isAllCalPointsCatched = false;
		}
		else if (i.first == "bottomP") {
			dstPoints[1] = p;
			if (srcPoints[1] == cv::Point2f(0, 0)) isAllCalPointsCatched = false;
		}
		else if (i.first == "leftP") {
			dstPoints[2] = p;
			if (srcPoints[2] == cv::Point2f(0, 0)) isAllCalPointsCatched = false;
		}
		else if (i.first == "rightP") {
			dstPoints[3] = p;
			if (srcPoints[3] == cv::Point2f(0, 0)) isAllCalPointsCatched = false;
		}
	}

	// using getPerspectiveTransform() if catch 4 calibrated points
	// otherwise using SIFT method with standard dartboard image to get perspective tansformation
	// M is transformation matrix
	//if (calPoints.size() == 4) {
	if (isAllCalPointsCatched){
		M = cv::getPerspectiveTransform(srcPoints, dstPoints);
		cv::warpPerspective(mat_img, mat_img, M, mat_img.size());
		cout << "Calibrated points catched! M = \n" << M << endl;
	}
	else {
		cout << "Calibrated points not catched!" << endl;
	}
}

void draw_darts(cv::Mat mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names, cv::Mat M)
{
	vector<cv::Point2f> scrPoints;
	vector<cv::Point2f> dstPoints;
	for (auto& i : result_vec) {
		if (obj_names.size() > i.obj_id and obj_names[i.obj_id] == "dartP") {
			scrPoints.push_back(cv::Point2f(i.x + i.w / 2, i.y + i.h / 2));
		}
	}
	cv::perspectiveTransform(scrPoints, dstPoints, M);

	// draw darts position
	for (auto& p : dstPoints) {
		cv::circle(mat_img, p, 2, { 0,225,0 }, 2);
	}
}

#endif    // OPENCV
