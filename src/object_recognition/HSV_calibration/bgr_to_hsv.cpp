#include <iostream>

// 
#include <opencv2/opencv.hpp>


// Soruces: 
// https://docs.opencv.org/3.4/df/d9d/tutorial_py_colorspaces.html

void print_scalar(cv::Scalar scalar) {
    std::cout << "Scalar: " << scalar << std::endl;
}

// https://stackoverflow.com/questions/43217964/opencv-convert-scalar-to-different-color-space
cv::Scalar rgb_to_hsv(cv::Scalar rgb) {
    cv::Mat rgb_mat(1, 1, CV_8UC3, rgb);
    cv::Mat hsv_mat;

    cv::cvtColor(rgb_mat, hsv_mat, cv::COLOR_RGB2HSV);

    return cv::Scalar(hsv_mat.at<float>(0,0), hsv_mat.at<float>(0,1), hsv_mat.at<float>(0,2));
}


int main(const int argc, const char **argv) {
    // Looking at image, mask and color mask 1, we can see that the hand is detected, and the ball. Want to not detect the hand

    cv::Scalar hsv_hand; // Hand 
    cv::Scalar hsv_ball; // Hand 

    cv::Scalar rgb_hand(134, 89, 56); // Hand 
    cv::Scalar rgb_ball(133, 37, 15); // Hand 

    hsv_hand = rgb_to_hsv(rgb_hand);
    hsv_ball = rgb_to_hsv(rgb_ball);

    print_scalar(hsv_hand);
    print_scalar(hsv_ball);



}