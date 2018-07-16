using namespace std;
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <cstring>
using namespace cv;

int main(int argc, const char* argv[])
{
    // dictionary生成
    const aruco::PREDEFINED_DICTIONARY_NAME dictionary_name = aruco::DICT_4X4_50;
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(dictionary_name);

    // マーカー画像生成
    int marker_id   = 0;
    const int side_pixels = 200;
    Mat marker_image;
    char image_name[5] ;
    for(int i=1;i<5;i++)
    {
 	image_name[1] = '\0';
	marker_id = i;
	aruco::drawMarker(dictionary, marker_id, side_pixels, marker_image);
        image_name[0] = char(48 + i);
	//cout << image_name << '\n';
	strcat(image_name,".png");
	cout << image_name << '\n';
	imshow(image_name,marker_image);
	imwrite(image_name,marker_image);
    }

    // 生成したマーカー画像を表示
    //imshow("marker_image", marker_image);
    //waitKey(0);

    return 0;
}
