//#include <map>
//#include <string>
#include <iostream>
#include "utility.h";

using namespace std;

#ifdef OPENCV

void resize_to_std(cv::Mat& mat_img, cv::Point2f* srcPoints, cv::Point2f* dstPoints, float& scale) {
	scale = abs(dstPoints[1].y - dstPoints[0].y) / abs(srcPoints[1].y - srcPoints[0].y);
	cv::resize(mat_img, mat_img, cv::Size(), scale, scale);
	for (int i = 0; i < 4; i++) srcPoints[i] *= scale;
}

void draw_scoreArea(cv::Mat& mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names, 
	std::map<std::string, std::pair<int, int>> calPoints_stdmap, cv::Mat& M, float& scale)
{
	// get calibrate points
	// Points = {topP, bottomP, leftP, rightP}
	cv::Point2f dstPoints[4];	// standard front-view points
	cv::Point2f srcPoints[4];	// perspective view points from result
	for (auto& i : result_vec) {
		if (obj_names.size() > i.obj_id) {
			string obj_name = obj_names[i.obj_id];
			cv::Point2f p = cv::Point2f(i.x + i.w / 2, i.y + i.h / 2);
			if (obj_name == "topP") srcPoints[0] = p;
			else if (obj_name == "bottomP") srcPoints[1] = p;
			else if (obj_name == "leftP") srcPoints[2] = p;
			else if (obj_name == "rightP") srcPoints[3] = p;
		}
	}
	bool isAllCalPointsCatched = true;
	for (auto& i : calPoints_stdmap) {
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

	// Calibrate score area to front-view 
	// using getPerspectiveTransform() if catch 4 calibrated points
	// otherwise using SIFT method with standard dartboard image to get perspective tansformation
	// M is transformation matrix
	// then expand img matrix to cover entire transformed img and crop back to origin size
	cv::Mat mat_img_out;
	if (isAllCalPointsCatched){
		resize_to_std(mat_img, srcPoints, dstPoints, scale);	// scale srcPoints from mat_img to dstPoints's size 
		cv::imshow("Origin", mat_img);
		M = cv::getPerspectiveTransform(srcPoints, dstPoints);
		cv::warpPerspective(mat_img, mat_img, M, mat_img.size());

		cout << "Calibrated points catched!\n M = \n" << M << endl;
	}
	else {
		cout << "Calibrated points not catched!" << endl;
		return;
	}

	// Draw score area 
}

void draw_darts(cv::Mat mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names, cv::Mat M, float& scale)
{
	vector<cv::Point2f> srcPoints;
	vector<cv::Point2f> dstPoints;
	for (auto& i : result_vec) {
		if (obj_names.size() > i.obj_id and obj_names[i.obj_id] == "dartP") {
			srcPoints.push_back(cv::Point2f(i.x + i.w / 2, i.y + i.h / 2) * scale);	// scale srcPoints to map dstPoints's position
		}
	}
	cv::perspectiveTransform(srcPoints, dstPoints, M);

	// draw darts position
	for (auto& p : dstPoints) {
		cv::circle(mat_img, p, 2, { 0,225,0 }, 2);
	}
}

void crop_dartBoard_by_calibratedPoints(cv::Mat& mat_img, std::map<std::string, std::pair<int, int>> calPoints_stdmap)
{
	cv::Point2f calPoints[4];	// standard front-view points
	for (auto& i : calPoints_stdmap) {
		cv::Point2f p = cv::Point2f(i.second.first, i.second.second);
		if (i.first == "topP") calPoints[0] = p;
		else if (i.first == "bottomP") calPoints[1] = p;
		else if (i.first == "leftP") calPoints[2] = p;
		else if (i.first == "rightP") calPoints[3] = p;
	}

	int w_pad = abs(calPoints[3].x - calPoints[2].x) / 5;
	int h_pad = abs(calPoints[1].y - calPoints[0].y) / 5;
	int row_start = (calPoints[0].y - w_pad) < 0 ? 0 : calPoints[0].y - w_pad;
	int row_end = (calPoints[1].y + w_pad) > mat_img.rows ? mat_img.rows : calPoints[1].y + w_pad;
	int col_start = (calPoints[2].x - w_pad) < 0 ? 0 : calPoints[2].x - w_pad;
	int col_end = (calPoints[3].x + w_pad) > mat_img.cols ? mat_img.cols : calPoints[3].x + w_pad;
	mat_img = mat_img(cv::Range(row_start, row_end), cv::Range(col_start, col_end));
	//cv::rectangle(mat_img, cv::Rect(cv::Point2i(col_start, row_start), cv::Point2i(col_end, row_end)), { 0,255,0 }, 2);
}

#endif    // OPENCV
