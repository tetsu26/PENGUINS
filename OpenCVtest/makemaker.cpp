using namespace std;
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
using namespace cv;

int main(int argc, const char* argv[])
{
    // dictionary生成
    const aruco::PREDEFINED_DICTIONARY_NAME dictionary_name = aruco::DICT_4X4_50;
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(dictionary_name);

    // マーカー画像生成
    const int marker_id   = 0;
    const int side_pixels = 200;
    Mat marker_image;
    aruco::drawMarker(dictionary, marker_id, side_pixels, marker_image);

    // 生成したマーカー画像を表示
    imshow("marker_image", marker_image);
    waitKey(0);

    return 0;
}
