#include <ctime>
#include <iostream>
#include <raspicam/raspicam_still_cv.h>
using namespace std; 
 
int main ( int argc,char **argv ) {
   
    time_t timer_begin,timer_end;
    raspicam::RaspiCam_Cv Camera;
    cv::Mat image;
    //set camera params
    Camera.set( CV_CAP_PROP_FORMAT, CV_8UC1 );
    //Open camera
    cout<<"Opening Camera..."<<endl;
    if (!Camera.open()) {cerr<<"Error opening the camera"<<endl;return -1;}
     //Start capture
    Camera.grab();
    cout<<"yaa1"<<endl;
    Camera.retrieve (image);
    cout<<"yaa2"<<endl;
    Camera.release();
    cout<<"yaa"<<endl;
    cv::imwrite("raspicam_cv_image.jpg",image);
    cout<<"Image saved at raspicam_cv_image.jpg"<<endl;
}
