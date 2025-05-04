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
        std::cerr << "Usage: " << argv[0] << " <denoise_mode : int>" << std::endl;
        return -1;
    }

    cv::VideoCapture cap("benchmark_video.mp4");

    if (!cap.isOpened())
    {
        std::cerr << "Error: Could not open video file." << std::endl;
        return -1;
    }

    int frame_count = 0;

    int denoise_mode = std::stoi(argv[1]);

    cv::Mat frame;
    cv::Mat gray_frame;

    while (true)
    {

        cap >> frame;

        if (frame.empty())
        {
            std::cout << "End of video stream." << std::endl;
            break;
        }


        switch (denoise_mode)
        {
        case 0:
            
            break;
    
        default:
            break;
        }









        frame_count++;
        // cv::imshow("Video Frame", gray_frame);

        // if(cv::waitKey(30) == 113) {
        //     break; // Exit if any key is pressed
        // }
    }

    std::cout << "Total frames processed: " << i << std::endl;

    cap.release();
    cv::destroyAllWindows();

    return 0;
}
