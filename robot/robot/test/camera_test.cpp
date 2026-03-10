#include <iostream>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>


using namespace std;

int main() {
    // 1. Setup GStreamer pipeline
    std::string pipeline = "libcamerasrc ! "
                       "video/x-raw, width=640, height=480, format=RGB ! "
                       "videoconvert ! "
                       "video/x-raw, format=BGR ! appsink drop=true";
    
    // 2. Create VideoCapture object (Explicitly using GStreamer backend)
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    
    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to open the camera pipeline." << std::endl;
        return -1;
    }

    cv::Mat frame;
    std::cout << "Starting camera streaming. (Press ESC to exit)" << std::endl;

    while (true) {
        // 3. Capture frame
        cap >> frame;

        if (frame.empty()) {
            std::cerr << "Error: Received an empty frame." << std::endl;
            break;
        }

        // 4. Display image (Vision Processing can be performed here)
        cv::imshow("RPi 5 CSI Camera (OpenCV)", frame);

        // Exit if ESC key is pressed
        if (cv::waitKey(1) == 27) break;
    }

    cv::destroyAllWindows();
    cap.release();
    return 0;
}