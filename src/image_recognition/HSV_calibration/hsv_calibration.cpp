#include <iostream>

// 
#include <opencv2/opencv.hpp>


// Soruces: 
// https://stackoverflow.com/questions/44369850/correct-path-to-read-an-image-in-opencvc
// https://stackoverflow.com/questions/30065953/mask-color-image-in-opencv-c


int main(const int argc, const char **argv) {

    if (argc != 2) {
        std::cout << "Spesifiy path to the image: " << "\n"; 
        std::cout << argv[0] << " <num>" << std::endl;
        return 1;
    }

    cv::Mat image = cv::imread("../screenshots/RGB_" + std::string(argv[1]) + ".jpg", cv::IMREAD_COLOR);
    cv::Mat mask = cv::imread("../screenshots/MASK_" + std::string(argv[1]) + ".jpg", cv::IMREAD_COLOR);
    cv::Mat mask_color = cv::imread("../screenshots/COLOR_mask_" + std::string(argv[1]) + ".jpg", cv::IMREAD_COLOR);

    if(image.empty() || mask.empty()) {
        std::cerr << "Could not open or find the image or mask" << std::endl;
        return 2;
    }

    // show image
    cv::imshow("Image", image);
    cv::imshow("Mask", mask);
    cv::imshow("Mask with color", mask_color);


    if (cv::waitKey(0) == 'q') {
        return 0;
      }

}