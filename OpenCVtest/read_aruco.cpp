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
		//dictionary生成
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
    
	Mat InImage, OutImage;
	Mat dst_Image;
    InImage =  imread(argv[1], IMREAD_UNCHANGED);
    InImage.copyTo(OutImage);
    imshow("in", InImage);
    waitKey(0);
    
    int width, height = 0;
	width  = InImage.cols;
	height = InImage.rows;
	
	//カメラパラメータの読み込み
	/*
	FileStorage fs2("out_camera_data.xml", FileStorage::READ);
    if (!fs2.isOpened()){
        cout << "File can not be opened." << endl;
        return -1;
    }
    Mat cameraMatrix, distCoeffs;
    fs2["camera_matrix"] >> cameraMatrix;
    fs2["distortion_coefficients"] >> distCoeffs;

    cout << "camera matrix: " << cameraMatrix<< endl
        << "distortion coeffs: " << distCoeffs<< endl;
       */
        
	//入力画像に補正をかける とりあえず省略 
	/*
	undistort(InImage,dst_Image, cameraMatrix, distCoeffs);
	namedWindow("dst_Image");
	imshow("補正画像",dst_Image);
	imwrite("image_dst.png",dst_Image);
	waitKey(0);
	*/
	vector<int> markerIds;
	vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
	Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
	aruco::detectMarkers(InImage, dictionary, markerCorners, markerIds,parameters,rejectedCandidates);
	cvtColor(InImage, OutImage,COLOR_GRAY2RGB);
	aruco::drawDetectedMarkers(OutImage, markerCorners, markerIds);
	printf("%d\n",markerIds.size());
	imshow("out", OutImage);
	waitKey(0); 
	
	
	
}
