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
	
	FileStorage fs2("out_camera_data.xml", FileStorage::READ);
    if (!fs2.isOpened()){
        cout << "File can not be opened." << endl;
        return -1;
    }
     
    Mat cameraMatrix, distCoeffs;
    /*
    cameraMatrix = (Mat1f(3,3) << 1445.455931500428, 0, 960,0, 1445.455931500428, 540,0, 0, 1);
    distCoeffs = (Mat1f(5,1) <<0.1433177732062716, -0.1956378847181435, 0, 0, -0.3822995443596148);
    * */
    
    fs2["camera_matrix"] >> cameraMatrix;
    fs2["distortion_coefficients"] >> distCoeffs;

    cout << "camera matrix: " << cameraMatrix<< endl
        << "distortion coeffs: " << distCoeffs<< endl;
       
        
	//入力画像に補正をかける とりあえず省略 
	/*
	undistort(InImage,dst_Image, cameraMatrix, distCoeffs);
	namedWindow("dst_Image");
	imshow("補正画像",dst_Image);
	imwrite("image_dst.png",dst_Image);
	waitKey(0);
	*/
	vector<int> markerIds;
	vector<vector<Point2f>> markerCorners, rejectedCandidates;
	Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
	aruco::detectMarkers(InImage, dictionary, markerCorners, markerIds,parameters,rejectedCandidates);
	printf("%d\n",markerIds.size());
	vector<Vec3d> rvecs, tvecs;
	rvecs.clear();
	tvecs.clear();
	aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
	printf("hoge\n");
	for(auto a: rvecs){
		cout << a<< endl;
	}
	for (int ii = 0; ii < markerIds.size(); ii++){
		aruco::drawAxis(OutImage, cameraMatrix, distCoeffs, rvecs[ii], tvecs[ii], 0.1);
	}

	imwrite("out.jpg",OutImage);
	imshow("out", OutImage);
	waitKey(0); 
	
	
	
}
