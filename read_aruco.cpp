using namespace std;
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <cstring>
#include <math.h>
#include <fstream>
#include <sstream>
using namespace cv;

int main(int argc, const char* argv[])
{
		//dictionary生成
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
    
	Mat InImage, OutImage;
	Mat dst_Image;
    InImage =  imread(argv[1], IMREAD_UNCHANGED);
    InImage.copyTo(OutImage);
   // imshow("in", InImage);
    //waitKey(0);
    
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
    
    fs2["camera_matrix"] >> cameraMatrix;
    fs2["distortion_coefficients"] >> distCoeffs;

    //マーカーの読み取りをする
	vector<int> markerIds;
	vector<vector<Point2f>> markerCorners, rejectedCandidates;
	Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
	aruco::detectMarkers(InImage, dictionary, markerCorners, markerIds,parameters,rejectedCandidates);
	vector<Vec3d> rvecs, tvecs;
	rvecs.clear();
	tvecs.clear();
	aruco::estimatePoseSingleMarkers(markerCorners, 0.053, cameraMatrix, distCoeffs, rvecs, tvecs); //0.053 はマーカーの大きさ
	//得られたベクトルの表示
	/*
	printf("rvecs:\n");
	for(auto a: rvecs){
		cout <<a<< endl;
	}
	printf("tvecs:\n");
	for(auto a: tvecs){
		cout <<a<< endl;
	}
	*/
	for (int ii = 0; ii < markerIds.size(); ii++){
		aruco::drawAxis(OutImage, cameraMatrix, distCoeffs, rvecs[ii], tvecs[ii], 0.1);
	}
	aruco::drawDetectedMarkers(OutImage,markerCorners, markerIds);
	
	stringstream  FileName;
	FileName << "image/out" <<argv[2]<<".jpg";
	string result = FileName.str();

	imwrite(result,OutImage);
	//imshow("out", OutImage);
	//waitKey(0); 
	
	//LNSまでの距離を計算する
	int readFlag = 0;
	readFlag = markerIds.size();
	
	double distance[2] = {0},x[2],y[2],z[2],cos[2],deg[2];
	x [0]= tvecs[0][0];
	y [0]= tvecs[0][1];
	z [0]= tvecs[0][2];
	distance[0] = sqrt(x[0]*x[0]+y[0]*y[0]+z[0]*z[0]);
	cos[0]= x[0]/z[0];
	deg[0] = atan(cos[0]) * 180 / 3.14159265359;
	printf("%lf\n",deg[0]);
	
	if(readFlag == 2)
	{
		x [1]= tvecs[1][0];
		y [1]= tvecs[1][1];
		z [1]= tvecs[1][2];
		distance[1] = sqrt(x[1]*x[1]+y[1]*y[1]+z[1]*z[1]);
		cos[1]= x[1]/z[1];
		deg[1] = atan(cos[1]) * 180 / 3.14159265359;
	}
	
	//csvに入れてデータの受け渡しをする
	
	int flag[2] ={0}; //右向きなら１、左向きなら２を返	
	
	if(tvecs[0][0]  >=  0)
	{
		//printf("右向きだね〜\n");
		flag[0] = 1;
	}
	else
	{
		//printf("左向きだね〜\n");
		flag[0] = 2;
	}
	if(readFlag == 2){ //もし二つ見つけたら
		if(tvecs[1][0]  >=  0)
		{
			//printf("右向きだね〜\n");
			flag[1] = 1;
		}
		else
		{
			//printf("左向きだね〜\n");
			flag[1] = 2;
		}
	}
	
	ofstream outputfile("read_result.csv",ios::app);
    outputfile<<"\n"<<argv[2]<<","<<readFlag<<","<<markerIds[0]<<","<<distance[0]<<","<<deg[0];
    if(readFlag == 2)
    {
		outputfile<<"\n"<<argv[2]<<","<<readFlag<<","<<markerIds[1]<<","<<distance[1]<<","<<deg[1];
	}
    outputfile.close();
	
	//printf("距離は%lf[m]だゾ〜〜！角度は%lf[deg]だゾ！\n",distance,deg);
	
	return 0;
	
}
