#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

#define CAMERA_HEIGHT 864    // Can be SD: 480, HD: 720, FHD: 1080, QHD: 1440,  y
#define CAMERA_WIDTH 3000    // Can be SD: 640, HD: 1280, FHD: 1920, QHD: 2560, x
#define CAMERA_FRAMERATE 100 // If fps higher than what the thread can handle, it will just run lower fps.

// From

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
    // cv::Mat mask;
    // cv::Mat blur;
    cv::Mat thresh;
    
    while (true)
    {

        if (cam.getVideoFrame(image, 1000) == false)
        {
            std::cout << "Timeout error" << std::endl;
        }
        else
        {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

            // cv::GaussianBlur(gray, blur, cv::Size(7, 7), 0, 0);

            cv::adaptiveThreshold(gray, thresh, 255, cv::AdaptiveThresholdTypes::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 101, 1);

            cv::imshow("mask", thresh);

            count, labels, stats, centeroids = cv::connectedComponentsWithStats(thresh, )



            cv::imshow("detected circles", image);

            if (cv::waitKey(1) == 'q')
            {
                break;
            }
        }
    }

    cam.stopVideo();
    cv::destroyAllWindows();

    return 0;
}