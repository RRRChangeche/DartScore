//#include <map>
//#include <string>
#include <iostream>
#include <math.h>
#include <map>
#include "utility.h";
#define PI 3.14159265

using namespace std;

#ifdef OPENCV

void resize_to_std(cv::Mat& mat_img, cv::Point2f* srcPoints, cv::Point2f* dstPoints, float& scale) {
	scale = abs(dstPoints[1].y - dstPoints[0].y) / abs(srcPoints[1].y - srcPoints[0].y);
	cv::resize(mat_img, mat_img, cv::Size(), scale, scale);
	for (int i = 0; i < 4; i++) srcPoints[i] *= scale;
}

float getDegreeFrom2Vector(const cv::Point2f& v1, const cv::Point2f& v2) {
	cv::Point2f p1(v1), p2(v2);
	double d1 = cv::norm(p1), d2 = cv::norm(p2);
	double dot = p1.dot(p2);
	double cross = p1.cross(p2);
	// if (cross < 0) means vector located at 3 or 4 Quadrant
	// arccos = 0 ~ pi
	// make sure output = 0 ~ 2pi
	float theta = 0.0;
	if (cross < 0) theta = (std::acos(dot / (d1 * d2)) + PI) / PI * 180;
	else theta = std::acos(dot / (d1 * d2)) / PI * 180;
	return theta;
}

cv::Point2f polar2xy(const cv::Point& center, float R, float theta) {
	// theta in degrees from 0.0 to 360.0
	//return (round(center[0] + r * np.cos(theta * np.pi / 180.0 - 0.5 * np.pi)), round(center[1] + r * np.sin(theta * np.pi / 180.0 - 0.5 * np.pi)))
	theta = theta * PI / 180.0;	// degree to radian
	return cv::Point2f(center) + R * cv::Point2f(sin(theta), cos(theta));
}

// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
bool findIntersection(cv::Point2f a1, cv::Point2f a2, cv::Point2f b1, cv::Point2f b2, cv::Point& c){
	cv::Point2f x = b1 - a1;
	cv::Point2f d1 = a2 - a1;
	cv::Point2f d2 = b2 - b1;

	float cross = d1.cross(d2);
	//float cross = d1.x * d2.y - d1.y * d2.x;
	if (abs(cross) < /*EPS*/1e-8) return false;

	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	c = a1 + d1 * t1;
	return true;
}


DartBoard::DartBoard() {
	// calibration points from standard image
	calibratePoints_std = { {"topP", {262,49}}, {"bottomP", {204,411}}, {"leftP", {53,196}}, {"rightP", {417, 255}} };
	// calibrate points
	for (int i = 0; i < 4; i++) {
		dstPoints[i] = cv::Point2f(0, 0);
		srcPoints[i] = cv::Point2f(0, 0);
	}
	// transformation matrix (default=unit matrix) for perspective transformation
	M = cv::Mat::eye(3, 3, CV_64F); 
	// scale factor for resize image to standard 
	scale = 0.0;
	// ratio of R grid of score area 
	//Rscale = new float[6] { 12.7, 32, 182, 214, 308, 340 };
	// score map
	scoreMap = { {0,1}, {1,18}, {2,4}, {3,13}, {4,6}, {5,10}, {6,15}, {7,2}, {8,17}, {9,3}, {10,19},
				{11,7}, {12,16}, {13,8}, {14,11}, {15,14}, {16,9}, {17,12}, {18,5}, {19,20} };
}

DartBoard::~DartBoard()
{

}

void DartBoard::draw_scoreArea(cv::Mat& mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names)
{
	// get calibrate points
	// Points = {topP, bottomP, leftP, rightP}
	//cv::Point2f dstPoints[4];	// standard front-view points
	//cv::Point2f srcPoints[4];	// perspective view points from result
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
	for (auto& i : calibratePoints_std) {
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
	/*
	* Steps:
	* using getPerspectiveTransform() if catch 4 calibrated points, otherwise
	* using SIFT method with standard dartboard image to get perspective tra-
	* nsformation, then expand img matrix to cover entire transformed img and
	* crop back to origin size.
	* 
	* M = transformation matrix
	*/
	cv::Mat mat_img_out;
	if (isAllCalPointsCatched){
		resize_to_std(mat_img, srcPoints, dstPoints, scale);	// scale srcPoints from mat_img to dstPoints's size 
		cv::imshow("Origin", mat_img);
		M = cv::getPerspectiveTransform(srcPoints, dstPoints);
		cv::warpPerspective(mat_img, mat_img, M, mat_img.size());
		// draw calibrate points
		for (const auto& p : dstPoints) cv::circle(mat_img, p, 2, { 0,225,0 }, 2);	
		//cout << "Calibrated points catched!\n M = \n" << M << endl;
		cout << "Calibrated points catched!" << endl;
	}
	else {
		cout << "Calibrated points not catched!" << endl;
		return;
	}

	// Draw score area grid
	// find center of dartboard
	cv::Point tp = dstPoints[0], bp = dstPoints[1], lp = dstPoints[2], rp = dstPoints[3];
	cv::Point center1;
	cv::Point center2((tp.x + bp.x + lp.x + rp.x) / 4 + 2, (tp.y + bp.y + lp.y + rp.y) / 4 + 3);
	if (findIntersection(tp, bp, lp, rp, center1)) center = (center1 + center2) / 2;
	else center = center2;
	cv::circle(mat_img, center, 2, { 0,255,0 }, 2);
	// Calibrateinital angle start from direction {0, 1}
	float iangle = (
		getDegreeFrom2Vector(cv::Point2f({ 0,1 }), cv::Point2f(tp - center)) - 9 +
		getDegreeFrom2Vector(cv::Point2f({ 0,1 }), cv::Point2f(rp - center)) - 99 +
		getDegreeFrom2Vector(cv::Point2f({ 0,1 }), cv::Point2f(bp - center)) - 189 +
		getDegreeFrom2Vector(cv::Point2f({ 0,1 }), cv::Point2f(lp - center)) - 279) / 4.0;

	float R = (cv::norm(center - tp) + cv::norm(center - bp) + cv::norm(center - lp) + cv::norm(center - rp)) / 4.0;
	//float Rscale[6] = { 12.7, 32, 182, 214, 308, 340 }; 
	// draw circle grid 
	for (auto& scale : Rscale) {
		scale /= 340.0;
		cv::circle(mat_img, center, R * scale, { 0, 255, 0 }, 1);
	}
	// draw line grid
	for (float i = 9+iangle; i < 369+iangle; i += 18) {
		cv::line(mat_img, cv::Point(polar2xy(center, R*32/340, i)), cv::Point(polar2xy(center, R, i)), { 0, 255, 0 }, 1);
	}
}

void DartBoard::draw_darts(cv::Mat mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names)
{
	vector<cv::Point2f> srcDartPoints;
	vector<cv::Point2f> dstDartPoints;
	for (auto& i : result_vec) {
		if (obj_names.size() > i.obj_id and obj_names[i.obj_id] == "dartP") {
			srcDartPoints.push_back(cv::Point2f(i.x + i.w / 2, i.y + i.h / 2) * scale);	// scale srcPoints to map dstPoints's position
		}
	}
	cv::perspectiveTransform(srcDartPoints, dstDartPoints, M);

	// draw darts position and check score 
	if (dstDartPoints.size() == 0) {
		cout << "No darts detecred!" << endl;
		return;
	}

	cv::Point tp = dstDartPoints[0];	// topP
	cv::Point v0(tp - center);	// vector from center to topP
	for (const auto& pos : dstDartPoints) {
		// calculate score
		cv::Point vdart(cv::Point(pos)-center); // vector from center to dart
		double ddart = cv::norm(vdart);

		// calculate degree
		float theta = getDegreeFrom2Vector(v0, vdart);
		int score = scoreMap[int(theta / 18)];
		string scoreText = to_string(score);

		// calculate Rscale
		float rscale = ddart / R;
		string scaleText = "";
		if (rscale <= Rscale[0]) { // DBull
			scaleText = "DBULL";
			score = 50;
		}
		else if (Rscale[0] < rscale <= Rscale[1]) { // SBULL
			scaleText = "SBULL";
			score = 50;
		}
		else if (Rscale[2] < rscale <= Rscale[3]) { // Triple area
			scaleText = "T";
			score *= 3;
		}
		else if (Rscale[4] < rscale <= Rscale[5]) { // Double area
			scaleText = "D";
			score *= 2;
		}
		else if (rscale > Rscale[5]) { // Outside scoring area
			scaleText = "Null";
			score = 0;
		}

		// dart darts position
		cv::circle(mat_img, pos, 2, { 0,0,225 }, 2);
		// put score text
		cv::putText(mat_img, scaleText + scoreText, pos, 0, 0.7, { 0, 0, 255 }, 2);
	}
}

void DartBoard::crop_dartBoard_by_calibratedPoints(cv::Mat& mat_img)
{
	int w_pad = abs(dstPoints[3].x - dstPoints[2].x) / 5;
	int h_pad = abs(dstPoints[1].y - dstPoints[0].y) / 5;
	int row_start = (dstPoints[0].y - w_pad) < 0 ? 0 : dstPoints[0].y - w_pad;
	int row_end = (dstPoints[1].y + w_pad) > mat_img.rows ? mat_img.rows : dstPoints[1].y + w_pad;
	int col_start = (dstPoints[2].x - w_pad) < 0 ? 0 : dstPoints[2].x - w_pad;
	int col_end = (dstPoints[3].x + w_pad) > mat_img.cols ? mat_img.cols : dstPoints[3].x + w_pad;
	mat_img = mat_img(cv::Range(row_start, row_end), cv::Range(col_start, col_end));
}

#endif    // OPENCV
