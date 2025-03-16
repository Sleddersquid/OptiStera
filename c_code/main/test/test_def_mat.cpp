#include <iostream>
#include <opencv2/opencv.hpp>


int main(int argc, char* argv[]) {
    cv::Mat mat = cv::Mat(3, 3, CV_8UC1);

    std::cout << "Test def mat" << mat << std::endl;
}
