#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>

int main(int argc, const char* argv[])
{
    // dictionary生成
    const cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name = cv::aruco::DICT_4X4_50;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_name);

    // マーカー画像生成
    const int marker_id   = 0;
    const int side_pixels = 200;
    cv::Mat marker_image;
    cv::aruco::drawMarker(dictionary, marker_id, side_pixels, marker_image);

    // 生成したマーカー画像を表示
    cv::imshow("marker_image", marker_image);
    cv::waitKey(0);

    return 0;
}