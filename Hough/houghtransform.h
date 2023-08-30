#pragma once
#include "opencv2/opencv.hpp"

constexpr int WIDTH = 640;
constexpr int HEIGHT = 480;
constexpr int ROI_OFFSET = 385;
constexpr int ROI = 30;
typedef std::vector<cv::Vec4i> Line;


class houghTransform
{
public :
	houghTransform() {}; // 持失切
	~houghTransform() {}; // 社瑚切
	void Preprocess(const cv::Mat frame, const cv::Mat mask, cv::Mat& roi);
	void HoughLine(cv::Mat roi, int pos[], int& left_pos, int& right_pos);
	void DivideLine(const Line lines, Line& left_lines, Line& right_lines);
	void GetLinePos(const Line lines, bool is_left, int& line_pos);
	void GetLineParams(const Line lines, float& m, float& b);
	void DrawLine(cv::Mat& frame, int left_pos, int right_pos);
	void weightedAverageFilter(int left_pos, int right_pos);

private :
	int pre_left[10] = { 0, };
	int pre_right[10] = { 0, };
	float left_mean = 0;
	float right_mean = WIDTH;
};