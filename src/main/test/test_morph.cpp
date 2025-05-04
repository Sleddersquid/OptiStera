#include <iostream>
#include <opencv2/opencv.hpp>

/**
 * @brief Function to calculate the center of a contour using moments
 * @param contour - The contour to calculate the center of
 * @return The center of the contour as cv::Point
 */

cv::Point calculateCenter(const std::vector<cv::Point> &contour)
{
    cv::Moments M = cv::moments(contour);
    // std::cout << "M.m00: " << M.m00 << std::endl;
    if (M.m00 != 0)
    {
        // See https://docs.opencv.org/3.4/d8/d23/classcv_1_1Moments.html for center of mass (x¯, y¯)
        return cv::Point((int)(M.m10 / M.m00), (int)(M.m01 / M.m00));
    }
    return cv::Point(0, 0); // If no center is found, return (-1, -1)
}

template <typename T>
class benchmark
{
private:
    T min;
    T max;

public:
    benchmark();

    void insert(T value)
    {
        if (value < min)
        {
            min = value;
        }
        if (value > max)
        {
            max = value;
        }
    }

    T getDifference()
    {
        return max - min;
    }
};

int main(int argc, char *argv[])
{

    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <morph_mode : int>" << std::endl;
        std::cerr << "morph_mode: 0 = Erode, 1 = Dilate, 2 = Open, 3 = Close, 4 = Gradient, 5 = Tophat, 6 = Blackhat" << std::endl;
        return -1;
    }

    cv::VideoCapture cap("benchmark_video.mp4");

    if (!cap.isOpened())
    {
        std::cerr << "Error: Could not open video file." << std::endl;
        return -1;
    }

    int i = 0;

    int morph_mode = std::stoi(argv[1]);
    int morph_op = 0;

    switch (morph_mode)
    {
    case 0:
        morph_op = cv::MORPH_ERODE;
        std::cout << "Mode: Erode" << std::endl;
        break;
    case 1:
        morph_op = cv::MORPH_DILATE;
        std::cout << "Mode: Dialte" << std::endl;
        break;
    case 2:
        morph_op = cv::MORPH_OPEN;
        std::cout << "Mode: Open" << std::endl;
        break;
    case 3:
        morph_op = cv::MORPH_CLOSE;
        std::cout << "Mode: Close" << std::endl;
        break;
    case 4:
        morph_op = cv::MORPH_GRADIENT;
        std::cout << "Mode: Gradient" << std::endl;
        break;
    case 5:
        morph_op = cv::MORPH_TOPHAT;
        std::cout << "Mode: Tophat" << std::endl;
        break;
    case 6:
        morph_op = cv::MORPH_BLACKHAT;
        std::cout << "Mode: Blackhat" << std::endl;
        break;

    default:
        break;
    }

    cv::Mat frame;
    cv::Mat HSV_frame;
    cv::Mat mask;
    cv::Mat morphed_frame;

    cv::Scalar hsv_lower(0, 120, 120);  // 0, 150, 50
    cv::Scalar hsv_upper(15, 255, 255); // 15, 255, 255
    
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

    while (true)
    {

        cap >> frame;

        if (frame.empty())
        {
            std::cout << "End of video stream." << std::endl;
            break;
        }

        cv::GaussianBlur(frame, frame, cv::Size(7, 7), 0);
        
        cv::cvtColor(frame, HSV_frame, cv::COLOR_BGR2HSV);
        
        cv::inRange(HSV_frame, hsv_lower, hsv_upper, mask);
    
        cv::morphologyEx(mask, morphed_frame, morph_op, kernel, cv::Point(-1, -1), 2);

        cv::imshow("moprhed Frame", morphed_frame);


        i++;

        if(cv::waitKey(30) == 113) {
            break; // Exit if any key is pressed
        }
    }

    std::cout << "Total frames processed: " << i << std::endl;

    cap.release();
    cv::destroyAllWindows();

    return 0;
}
