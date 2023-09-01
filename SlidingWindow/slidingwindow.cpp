#include "slidingwindow.h"

void SlidingWindow::Preprocess(const cv::Mat frame, const cv::Mat mask, cv::Mat& fin) {
	cv::Mat gray, blur, bin, morp;
	cv::Mat element3(3, 3, CV_8U, cv::Scalar(1));
    cv::Point srcPoints[4];

	cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
	cv::medianBlur(gray, blur, 9);
	cv::adaptiveThreshold(blur, bin, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 255, -5);
	cv::morphologyEx(bin, morp, cv::MORPH_OPEN, element3);
	cv::bitwise_and(morp, mask, fin);
}

void SlidingWindow::Warp(const cv::Mat &frame, cv::Mat& warp, cv::Mat &perspectiveTransform, cv::Mat &perpsectiveTransformInv) {

    cv::Point2f srcPoints[4] = {
       cv::Point2f(180, 300),
       cv::Point2f(0, 410),
       cv::Point2f(460, 300),
       cv::Point2f(640, 410)
    };

    // ��ǥ �̹����� ��ǥ (BEV�� ��ȯ�� ��ǥ)
    cv::Point2f dstPoints[4] = {
        cv::Point2f(0, 0),
        cv::Point2f(0, HEIGHT),
        cv::Point2f(WIDTH, 0),
        cv::Point2f(WIDTH, HEIGHT)
    };

    perspectiveTransform = cv::getPerspectiveTransform(srcPoints, dstPoints);
    perpsectiveTransformInv = cv::getPerspectiveTransform(dstPoints, dstPoints);
    cv::warpPerspective(frame, warp, perspectiveTransform, cv::Size(WIDTH, HEIGHT));

}

void SlidingWindow::SlidingWindowProcess(const cv::Mat& image, cv::Mat& result_image) {

 
}
