#include <iostream>
#include "opencv2/opencv.hpp"
#include "houghtransform.h"

int main(int argc, char** argv)
{
	houghTransform houghtransform;
	cv::VideoCapture cap("Sub_project.avi");
	if (!cap.isOpened()) {
		std::cerr << "Image load failed!\n" << std::endl;
		exit(1);
	}

	cv::Mat mask = cv::imread("mask.png", cv::IMREAD_GRAYSCALE);
	if (mask.empty()) {
		std::cerr << "Mask load failed!\n" << std::endl;
		exit(1);
	}

	cv::Mat frame, roi;

	int histSize = 256; // 히스토그램 크기 (0부터 255까지의 픽셀 값 범위)
	float range[] = { 0, 256 };
	const float* histRange = { range };
	bool uniform = true, accumulate = false;
	cv::Mat hist;

	while (true)
	{
		if (!cap.read(frame))
		{
			std::cout << "Video End\n" << std::endl;
			break;
		}

		int pos[2] = { 0, };
		int left_pos = 0, right_pos = WIDTH;

		houghtransform.Preprocess(frame, mask, roi);
		houghtransform.HoughLine(roi, pos, left_pos, right_pos);
		houghtransform.DrawLine(frame,left_pos, right_pos);

		cv::imshow("frame", frame);
		cv::waitKey(1);
	}
}