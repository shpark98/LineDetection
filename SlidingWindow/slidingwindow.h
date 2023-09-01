#pragma once
#include "opencv2/opencv.hpp"
#include <numeric>

constexpr int WIDTH = 640;
constexpr int HEIGHT = 480;
constexpr int nWindows = 32;
constexpr int Window_HEIGHT = WIDTH / nWindows;



class SlidingWindow
{
public:
	SlidingWindow() {};
	~SlidingWindow() {};
	void Preprocess(const cv::Mat frame, const cv::Mat mask, cv::Mat& fin);
	void Warp(const cv::Mat &frame, cv::Mat &warp, cv::Mat &perspectiveTransform, cv::Mat &perpsectiveTransformInv);
	void SlidingWindowProcess(const cv::Mat& image, cv::Mat& out_img);
private:
};