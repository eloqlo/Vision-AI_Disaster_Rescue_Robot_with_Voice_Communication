#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace std;

int main() 
{
    string image_path = "/home/jh/Desktop/image.jpg";

    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);

    if (image.empty())
    {
        cout << "Could not read the image: " << endl;
        return 1;
    }

    cv::rotate(image, image, cv::ROTATE_180);

    for (int i=0; i<image.rows; i++) {
        for (int j=0; j<image.cols; j++) {

            cv::Vec3b bgrPixel = image.at<cv::Vec3b>(i,j);

            unsigned char grayScale = (bgrPixel[0] + bgrPixel[1] + bgrPixel[2]) / 3;

            image.at<cv::Vec3b>(i,j) = {grayScale, grayScale, grayScale};

            cv::Vec3b grayPixel = image.at<cv::Vec3b>(i,j);
        }
    }

    cv::imshow("After Rotate", image);

    int k = cv::waitKey(0); // wait for a keystroke in the window
    if (k == 's')
    {
        cv::imwrite("/home/jh/Desktop/gray_image.jpg", image);
    }

    return 0;
}