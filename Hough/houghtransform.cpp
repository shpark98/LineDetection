#include "houghtransform.h"

void houghTransform::Preprocess(const cv::Mat frame, const cv::Mat mask, cv::Mat &roi) {
	cv::Mat gray, stretch, blur, canny, morp, fin;
	cv::Mat element3(3, 3, CV_8U, cv::Scalar(1));
	double min, max = 0;

	// convert grayscale
	cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
	//cv::imshow("gray", gray);

	// histogram stretch
	cv::minMaxLoc(gray, &min, &max);
	stretch = (gray - min) * 255 / (max - min);
	//cv::imshow("stretch", stretch);

	// gaussian
	cv::GaussianBlur(stretch, blur, cv::Size(5, 5), 1.5);
	//cv::imshow("blur", blur);

	// canny
	cv::Canny(blur, canny, 100, 200);
	//cv::imshow("canny", canny);

	// morphology
	cv::morphologyEx(canny, morp, cv::MORPH_DILATE, element3);
	//cv::imshow("morp", morp);

	// mask
	cv::bitwise_and(morp, mask, fin);

	// roi
	roi = fin(cv::Range(ROI_OFFSET, ROI_OFFSET + ROI), cv::Range(0, WIDTH));
}

void houghTransform::HoughLine(cv::Mat roi, int pos[], int& left_pos, int& right_pos) {
	Line lines, left_lines, right_lines;
	
	// Line Detect
	cv::HoughLinesP(roi, lines, 1, CV_PI / 180, 30, 12.5, 5);
	
	// 찾은 lines가 없을 경우 0, 640으로 설정 후 return
	if (lines.empty()){
		pos[0] = 0, pos[1] = 640; 
		return;
	}

	DivideLine(lines, left_lines, right_lines);
	GetLinePos(left_lines, true, left_pos); 
	GetLinePos(right_lines, false, right_pos);
	weightedAverageFilter(left_pos, right_pos);
	
	pos[0] = left_pos, pos[1] = right_pos;

}

void houghTransform::DivideLine(const Line lines, Line& left_lines, Line& right_lines) {

	int x1, x2, y1, y2 = 0;
	float slope = 0;
	std::vector<float> slopes;
	Line selected_lines;

	for (auto& line : lines) {
		x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3]; // 찾은 선의 두 점의 좌표 (x1, y1), (x2, y2)
		if (x1 - x2 == 0) {
			slope = 0;
		}
		else {
			slope = float(y2 - y1) / float(x2 - x1);
		}
		if (0 < abs(slope)) {

			slopes.push_back(slope);
			selected_lines.push_back(line);
		}
	}

	for (int i = 0; i < slopes.size(); i++) {
		cv::Vec4i selelcted_line = selected_lines[i];
		slope = slopes[i];
		x1 = selelcted_line[0], y1 = selelcted_line[1], x2 = selelcted_line[2], y2 = selelcted_line[3];
		float x_mean = float((x1 + x2) / 2);

		// slope이 0보다 작으면서 영상 중간보다 왼쪽에 있으면서 가중치왼쪽평균에서 부터 30 픽셀 안에 있으면 왼쪽 차선으로 분류
		if (slope < 0 && x2 < WIDTH / 2 && (abs(left_mean - x_mean) < 30 || left_mean == 0))
		{
			left_lines.push_back(selelcted_line);
		}

		// slope이 0보다 크면서 영상 중간보다 오른쪽에 있으면서 가중치오른쪽평균 부터 30 픽셀안에 있으면 오른쪽 차선으로 분류
		else if (slope > 0 && x1 > WIDTH / 2 &&	(abs(right_mean - x_mean) < 30 || right_mean == WIDTH))
		{
			right_lines.push_back(selelcted_line);
		}
	}
}

void houghTransform::GetLinePos(const Line lines, bool is_left, int& line_pos) {

	// 하나의 line을 그릴 때 line_x1은 (line_x1, 480), line_x2는 (line_x2, 240)
	// line_pos는 각 차선의 중앙

	float m = 0, b = 0;
	int y = ROI / 2;
	GetLineParams(lines, m, b); // 기울기와 y 절편 구하기

	if (m == 0 && b == 0)
	{
		if (is_left)
		{
			line_pos = 0;
		}
		else
		{
			line_pos = WIDTH;
		}
	}
	else
	{
		line_pos = (y - b) / m; // 차선의 중앙 x 값을 구하기 위해서는 y= mx + b 식을 이용해 x인 line_pos 값을 구함
	}
}


void houghTransform::GetLineParams(const Line lines, float& m, float& b)
{
	float x_sum = 0.0, y_sum = 0.0, m_sum = 0.0;
	int x1, y1, x2, y2;
	int size = lines.size();

	if (!size) // size가 0 이면 기울기와 y 절편을 0으로 설정 후 return
	{
		m = 0; b = 0;
		return;
	}
	for (auto& line : lines) // 대표 차선 만들기
	{
		x1 = line[0], x2 = line[2];
		y1 = line[1], y2 = line[3];
		x_sum += x1 + x2; // x 좌표 합계
		y_sum += y1 + y2; // y 좌표 합계
		m_sum += (float(y2 - y1) / float(x2 - x1)); // 기울기 합계
	}
	float x_avg = float(x_sum) / float(size * 2); // x 좌표 평균
	float y_avg = float(y_sum) / float(size * 2); // y 좌표 평균 
	m = m_sum / float(size); // 기울기 평균
	b = y_avg - m * x_avg; // y 절편 값 
}

void houghTransform::weightedAverageFilter(int left_pos, int right_pos) {

	if (left_pos == 0) {
		for (int i = 0; i < 10; i++) {
			pre_left[i] = 0;
		}
		left_mean = 0;
	}
	else {
		for (int i = 0; i < 9; i++) {
			pre_left[i] = pre_left[i + 1];
		}
		pre_left[9] = left_pos;
		left_mean = 0;
		int k = 0;
		for (int i = 0; i < 10; i++) {
			if (pre_left[i] > 0) {
				left_mean += (pre_left[i] * ((i + 1) * (i + 1)));
				k += ((i + 1) * (i + 1));
			}
		}
		left_mean /= k;
	}
	if (right_pos == WIDTH) {
		for (int i = 0; i < 10; i++) {
			pre_right[i] = WIDTH;
		}
		right_mean = WIDTH;
	}
	else {
		for (int i = 0; i < 9; i++) {
			pre_right[i] = pre_right[i + 1];
		}
		pre_right[9] = right_pos;
		right_mean = 0;
		int k = 0;
		for (int i = 0; i < 10; i++) {
			if (pre_right[i] < WIDTH) {
				right_mean += (pre_right[i] * ((i + 1) * (i + 1)));
				k += ((i + 1) * (i + 1));
			}
		}
		right_mean /= k;
	}
}

void houghTransform::DrawLine(cv::Mat& frame, int left_pos, int right_pos) {

	//draw
	line(frame,
		cv::Point(left_pos, 400),
		cv::Point(left_pos, 400),
		cv::Scalar(0, 0, 255), 7, cv::LINE_AA);
	line(frame,
		cv::Point(right_pos, 400),
		cv::Point(right_pos, 400),
		cv::Scalar(0, 0, 255), 7, cv::LINE_AA);
		
}