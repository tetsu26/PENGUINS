#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>



int main(int argc, const char* argv[])
{
    if (argc < 2)
    {
        cerr << "Usage: (in.jpg|in.avi) [cameraParams.yml] [markerSize] [outImage]" << endl;
        exit(0);
    }

    bool isVideoFile = true;
    cv::Mat InImage;
    cv::VideoCapture cap(argv[1]);
    if (!cap.isOpened())
    {
        InImage = cv::imread(argv[1], cv::IMREAD_UNCHANGED);
        isVideoFile = false;
    }

    int width, height = 0;
    if (isVideoFile)
    {
        width  = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    }
    else
    {
        width  = InImage.cols;
        height = InImage.rows;
    }

    aruco::CameraParameters CamParam;
    aruco::MarkerDetector MDetector;
    std::vector<aruco::Marker> Markers;
    float MarkerSize = -1;
    unsigned int delay = isVideoFile ? 1 : 0;

    if (argc >= 3)
    {
        CamParam.readFromXMLFile(argv[2]);
        CamParam.resize(cv::Size(width, height));
    }
    if (argc >= 4) MarkerSize = atof(argv[3]);

    cv::namedWindow("image", cv::WINDOW_AUTOSIZE);

    for (;;)
    {
        if (isVideoFile) cap >> InImage;

        if (InImage.empty()) break;

        MDetector.detect(InImage, Markers, CamParam, MarkerSize);

        for (unsigned int i = 0; i < Markers.size(); i++)
        {
#if DEBUG
            cout << Markers[i] << endl;
#endif
            Markers[i].draw(InImage, cv::Scalar(0, 0, 255), 2);
        }
        if (CamParam.isValid() && MarkerSize != -1)
        {
            for (unsigned int i = 0; i < Markers.size(); i++)
            {
                aruco::CvDrawingUtils::draw3dCube(InImage, Markers[i], CamParam);
            }
        }
        cv::imshow("image", InImage);

        if (cv::waitKey(delay) >= 0) break;
    }

    cv::destroyAllWindows();
    return 0;
}
