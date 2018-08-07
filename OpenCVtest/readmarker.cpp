//#include <stdafx.h>
#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

int main(int argc, const char * argv[])
{
    // 共通パラメータ
    const auto name = cv::aruco::DICT_4X4_50;
    const auto dictionary = cv::aruco::getPredefinedDictionary(name);

    // マーカー画像を生成
    const auto markerSidePixels = 128;
    const auto columns = 4;
    const auto rows = 5;
    const auto margin = 20;

    auto width = columns * markerSidePixels + margin * (columns + 1);
    auto height = rows * markerSidePixels + margin * (rows + 1);

    auto id = 0;
    cv::Rect roi(0, 0, markerSidePixels, markerSidePixels);
    cv::Mat marker(cv::Size(width, height), CV_8UC1, cv::Scalar::all(255));

    for (auto y = 0; y < rows; y++)
    {
        roi.y = y * markerSidePixels + margin * (y + 1);

        for (auto x = 0; x < columns; x++)
        {
            roi.x = x * markerSidePixels + margin * (x + 1);

            cv::Mat roiMat(marker, roi);
            cv::Mat markerImage;
            cv::aruco::drawMarker(dictionary, id++, markerSidePixels, markerImage, 1);
            markerImage.copyTo(roiMat);
        }
    }

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters(new cv::aruco::DetectorParameters());

    // マーカーの検出
    cv::aruco::detectMarkers(marker, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    cv::Mat detected_marker;
    cv::cvtColor(marker, detected_marker, cv::COLOR_GRAY2RGB);
    cv::aruco::drawDetectedMarkers(detected_marker, markerCorners, markerIds, CvScalar(255, 0, 0));
    // rejectedCandidatesにはidがないのでnoArray
    cv::aruco::drawDetectedMarkers(detected_marker, rejectedCandidates, cv::noArray(), CvScalar(0, 0, 255)); 

    /* 表示 */
    cv::namedWindow("marker");
    cv::namedWindow("detected");
    cv::imshow("marker", marker);
    cv::imwrite("maker.png",marker);
    cv::imshow("detected", detected_marker);
    cv::waitKey(0);

    return 0;
}
