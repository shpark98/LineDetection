// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file HoughTransformLaneDetector.cpp
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @author Jiho Han
 * @author Haeryong Lim
 * @author Chihyeon Lee
 * @brief hough transform lane detector class source file
 * @version 1.1
 * @date 2023-05-02
 */

#include <numeric>

#include "LaneKeepingSystem/HoughTransformLaneDetector.hpp"

namespace Xycar {

template <typename PREC>
void HoughTransformLaneDetector<PREC>::setConfiguration(const YAML::Node& config)
{
    mGaussianKernalSize = config["GAUSSIANBLUR"]["KERNALSIZE"].as<int32_t>();
    mImageWidth = config["IMAGE"]["WIDTH"].as<int32_t>();
    mImageHeight = config["IMAGE"]["HEIGHT"].as<int32_t>();
    mROIStartHeight = config["IMAGE"]["ROI_START_HEIGHT"].as<int32_t>();
    mROIHeight = config["IMAGE"]["ROI_HEIGHT"].as<int32_t>();
    mCannyEdgeLowThreshold = config["CANNY"]["LOW_THRESHOLD"].as<int32_t>();
    mCannyEdgeHighThreshold = config["CANNY"]["HIGH_THRESHOLD"].as<int32_t>();
    mHoughLineSlopeRange = config["HOUGH"]["ABS_SLOPE_RANGE"].as<PREC>();
    mHoughThreshold = config["HOUGH"]["THRESHOLD"].as<int32_t>();
    mHoughMinLineLength = config["HOUGH"]["MIN_LINE_LENGTH"].as<int32_t>();
    mHoughMaxLineGap = config["HOUGH"]["MAX_LINE_GAP"].as<int32_t>();
    mDebugging = config["DEBUG"].as<bool>();
}

template <typename PREC>
std::pair<PREC, PREC> HoughTransformLaneDetector<PREC>::getLineParameters(const Lines& lines, const Indices& lineIndices)
{
    // TODO : Implement this function
    PREC xSum = 0.0f;
    PREC ySum = 0.0f;
    PREC mSum = 0.0f;
    
    int32_t size = lineIndices.size();
    if (size == 0) return std::pair(0.f, 0.f);

    PREC m = 0.0f;
    PREC b = 0.0f;
    
    for (int32_t ind : lineIndices) {
        PREC x1, y1, x2, y2; 
        x1 = lines[ind][HoughIndex::x1];
        x2 = lines[ind][HoughIndex::x2];
        y1 = lines[ind][HoughIndex::y1];
        y2 = lines[ind][HoughIndex::y2];

        xSum += x1 + x2;
        ySum += y1 + y2;
        mSum += PREC(y2 - y1) / PREC(x2 - x1);
    }
    PREC xAvg = xSum / (size * 2);
    PREC yAvg = ySum / (size * 2);
    m = mSum / size;
    b = yAvg - m * xAvg;
  
    return { m, b };
}

template <typename PREC>
int32_t HoughTransformLaneDetector<PREC>::getLinePositionX(const Lines& lines, const Indices& lineIndices, Direction direction)
{
    // TODO : Implement this function
    int32_t positionX = 0;

    //global Width    mImageWidth, Height    mImageHeight;
    //global Offset  mROIStartHeight , Gap   mROIHeight;
    
    const auto [m, b] = getLineParameters(lines, lineIndices);


    PREC y = static_cast<PREC>(mROIHeight) * 0.5f;
    return std::round((y - b) / m);
   
    
    if(abs(m) <= std::numeric_limits<float>::epsilon() && abs(m) <= std::numeric_limits<float>::epsilon()) {
    //if (m == 0 && b == 0) {
        if (direction == Direction::LEFT)
            positionX = 0.0f;
        else if (direction == Direction::RIGHT)
            positionX = static_cast<PREC>mImageWidth;  //static_cast 
    }

    else {
        PREC y = static_cast<PREC>mROIHeight * 0.5f; //static_cast 
        positionX = std::round((y - b) / m);

    }

    return positionX;
    
}

template <typename PREC>
std::pair<Indices, Indices> HoughTransformLaneDetector<PREC>::divideLines(const Lines& lines)
{    
    int32_t x1, y1, x2, y2;
    uint32_t linesSize = static_cast<uint32_t>(lines.size());
    PREC slope = 0.0f;
    Indices leftLineIndices;
    Indices rightLineIndices;
    leftLineIndices.reserve(linesSize);
    rightLineIndices.reserve(linesSize);
    
    for (int32_t i = 0; i < linesSize; ++i) {
        const Line& line = lines[i];
        
        x1 = line[HoughIndex::x1];
        y1 = line[HoughIndex::y1];
        x2 = line[HoughIndex::x2];
        y2 = line[HoughIndex::y2];
        
        if (x2 - x1 == 0) slope = 0.0f;
        else slope = static_cast<PREC>((y2 - y1) / (x2 - x1));
        
        if (-mHoughLineSlopeRange <= slope && slope < 0.0f) leftLineIndices.emplace_back(i);
        else if (0.0f < slope && slope <= mHoughLineSlopeRange) rightLineIndices.emplace_back(i);
    }
    return { leftLineIndices, rightLineIndices };
}

template <typename PREC>
std::pair<int32_t, int32_t> HoughTransformLaneDetector<PREC>::getLanePosition(const cv::Mat& image)
{
    // TODO : Implement this function
    if (mDebugging) image.copyTo(mDebugFrame);
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    int32_t m = static_cast<int>(mean(gray)[0]); // grayscale(channel 1) average brightness
    int64_t alpha = 1.0f; // ratio

    gray = gray + (gray - m) * alpha; // Adjust the contrast ratio based on average brightness

    cv::Mat blurGray;
    double sigmaX = 0.0;
    cv::GaussianBlur(gray, blurGray, cv::Size(mGaussianKernalSize, mGaussianKernalSize), sigmaX);
    mBlurDebugFrame = blurGray.copyTo(blurGray);

    cv::Mat edgeImg;
    cv::Canny(blurGray, edgeImg, mCannyEdgeLowThreshold, mCannyEdgeHighThreshold);
    mCannyDebugFrame = edgeImg.copyTo(mCannyDebugFrame);
    cv::Mat roi = edgeImg(cv::Rect(cv::Point(0, mROIStartHeight), cv::Point(mImageWidth, mROIHeight + mROIStartHeight)));

    Lines allLines;
    cv::HoughLinesP(roi, allLines, 1, 3.14 / 180.0, mHoughThreshold, mHoughMinLineLength, mHoughMaxLineGap);

    if (allLines.empty()) return { 0, 640 };
    
    std::pair<Indices, Indices> leftRightLane = divideLines(allLines);
    
    int32_t leftPositionX = getLinePositionX(allLines, leftRightLane.first, Direction::LEFT);
    int32_t rightPositionX = getLinePositionX(allLines, leftRightLane.second, Direction::RIGHT);

    return { leftPositionX, rightPositionX };
}

template <typename PREC>
void HoughTransformLaneDetector<PREC>::drawLines(const Lines& lines, const Indices& leftLineIndices, const Indices& rightLineIndices)
{
    auto draw = [this](const Lines& lines, const Indices& indices) {
        for (const auto index : indices)
        {
            const auto& line = lines[index];
            auto r = static_cast<PREC>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();
            auto g = static_cast<PREC>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();
            auto b = static_cast<PREC>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();

            cv::line(mDebugFrame, { line[static_cast<uint8_t>(HoughIndex::x1)], line[static_cast<uint8_t>(HoughIndex::y1)] + mROIStartHeight },
                     { line[static_cast<uint8_t>(HoughIndex::x2)], line[static_cast<uint8_t>(HoughIndex::y2)] + mROIStartHeight }, { b, g, r }, kDebugLineWidth);
        }
    };

    draw(lines, leftLineIndices);
    draw(lines, rightLineIndices);
}

template <typename PREC>
void HoughTransformLaneDetector<PREC>::drawRectangles(int32_t leftPositionX, int32_t rightPositionX, int32_t estimatedPositionX)
{
    cv::rectangle(mDebugFrame, cv::Point(leftPositionX - kDebugRectangleHalfWidth, kDebugRectangleStartHeight + mROIStartHeight),
                  cv::Point(leftPositionX + kDebugRectangleHalfWidth, kDebugRectangleEndHeight + mROIStartHeight), kGreen, kDebugLineWidth);

    cv::rectangle(mDebugFrame, cv::Point(rightPositionX - kDebugRectangleHalfWidth, kDebugRectangleStartHeight + mROIStartHeight),
                  cv::Point(rightPositionX + kDebugRectangleHalfWidth, kDebugRectangleEndHeight + mROIStartHeight), kGreen, kDebugLineWidth);

    cv::rectangle(mDebugFrame, cv::Point(estimatedPositionX - kDebugRectangleHalfWidth, kDebugRectangleStartHeight + mROIStartHeight),
                  cv::Point(estimatedPositionX + kDebugRectangleHalfWidth, kDebugRectangleEndHeight + mROIStartHeight), kRed, kDebugLineWidth);

    cv::rectangle(mDebugFrame, cv::Point(mImageWidth / 2 - kDebugRectangleHalfWidth, kDebugRectangleStartHeight + mROIStartHeight),
                  cv::Point(mImageWidth / 2 + kDebugRectangleHalfWidth, kDebugRectangleEndHeight + mROIStartHeight), kBlue, kDebugLineWidth);
}

template class HoughTransformLaneDetector<float>;
template class HoughTransformLaneDetector<double>;
} // namespace Xycar
