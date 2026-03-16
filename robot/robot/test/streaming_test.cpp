#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // 가장 단순한 형태
    // libcamerasrc와 videoconvert 사이에 'video/x-raw,format=RGB'를 명시합니다.
    // format을 libcamerasrc caps에 명시적으로 지정
    std::string pipeline =
        "libcamerasrc ! "
        "video/x-raw,width=640,height=480,framerate=30/1,format=RGBx ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=true sync=false";
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open video stream." << std::endl;
        return -1;
    }

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) 
            break;

        cv::imshow("RPI5 Camera Stream", frame);
        if (cv::waitKey(1) == 27) 
            break;
    }

    cap.release();
    return 0;
}