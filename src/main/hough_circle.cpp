#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

#define CAMERA_HEIGHT 864    // Can be SD: 480, HD: 720, FHD: 1080, QHD: 1440,  y
#define CAMERA_WIDTH 3000    // Can be SD: 640, HD: 1280, FHD: 1920, QHD: 2560, x
#define CAMERA_FRAMERATE 100 // If fps higher than what the thread can handle, it will just run lower fps.

// Has been modified
// From OPENCV

#include <lccv.hpp>


int main(int argc, char **argv)
{
    
    lccv::PiCamera cam(0);
    
    cam.options->video_width = CAMERA_WIDTH;
    cam.options->video_height = CAMERA_HEIGHT;
    cam.options->framerate = CAMERA_FRAMERATE;
    cam.options->verbose = true;
    // cam.options->denoise = "cdn_fast";
    cam.options->sharpness = 2.5f; // 6.5f
    cam.options->shutter = 1/1000.0f;
    // cam.options->setExposureMode(Exposure_Modes::EXPOSURE_SHORT);
    // cam.options->setWhiteBalance(WhiteBalance_Modes::WB_INDOOR);
    // cam.options->list_cameras = true;
    
    cam.startVideo();
    cv::Mat gray;
    cv::Mat image;
    cv::Mat mask;
    cv::Mat blur;
    
    while (true)
    {

        if (cam.getVideoFrame(image, 1000) == false)
        {
            std::cout << "Timeout error" << std::endl;
        }
        else
        {
            cv::cvtColor(image, blur, cv::COLOR_BGR2GRAY);

            cv::adaptiveThreshold(blur, mask, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 201, 3);

            cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)), cv::Point(-1, -1));

            cv::imshow("mask", mask);

            // cv::dilate(mask, mask, cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, cv::Size(7, 7)), cv::Point(-1, -1), 2);

            std::vector<cv::Vec3f> circles;
            HoughCircles(mask, circles, cv::HOUGH_GRADIENT, 1,
                         mask.rows / 3, // change this value to detect circles with different distances to each other
                         100, 30, 30, 50 // change the last two parameters
            );
            for (size_t i = 0; i < circles.size(); i++)
            {
                cv::Vec3i c = circles[i];
                cv::Point center = cv::Point(c[0], c[1]);
                // circle center
                cv::circle(image, center, 1, cv::Scalar(0, 100, 100), 3, cv::LINE_AA);
                // circle outline
                int radius = c[2];
                cv::circle(image, center, radius, cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
            }

            std::cout << "vector size: " << circles.size() << std::endl;

            cv::imshow("detected circles", image);

            if (cv::waitKey(1) == 'q')
            {
                break;
            }
        }
    }

    cam.stopVideo();
    cv::destroyAllWindows();

    return EXIT_SUCCESS;
}