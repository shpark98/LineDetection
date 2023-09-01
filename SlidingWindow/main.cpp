#include <iostream>
#include "slidingwindow.h"

int main(int argc, char**argv){
	SlidingWindow slidingwindow;

	cv::VideoCapture cap("Sub_project.avi");
	if (!cap.isOpened()) {
		std::cerr << "Image load failed\n" << std::endl;
		exit(1);
	}

	cv::Mat mask = cv::imread("mask.png", cv::IMREAD_GRAYSCALE);
	if (mask.empty()) {
		std::cerr << "Mask load failed\n" << std::endl;
		exit(1);
	}

	cv::Mat frame, fin, warp, perspectiveTransform, perpsectiveTransformInv, slidingwindowresult;

	while (true) {
		if (!cap.read(frame)) {
			std::cout << "Video End\n" << std::endl;
			break;
		}

		slidingwindow.Preprocess(frame, mask, fin);
		slidingwindow.Warp(fin, warp, perspectiveTransform, perpsectiveTransformInv);
		slidingwindow.SlidingWindowProcess(warp, slidingwindowresult);
		cv::imshow("slidingwindow", slidingwindowresult);
		cv::waitKey(1);
	}
}
