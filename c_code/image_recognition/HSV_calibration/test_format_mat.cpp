#include <iostream>
#include <opencv2/opencv.hpp>
#include <lccv.hpp>

#define CAMERA_HEIGHT 720    // Can be SD: 480, HD: 720, FHD: 1080, QHD: 1440
#define CAMERA_WIDTH 1280    // Can be SD: 640, HD: 1280, FHD: 1920, QHD: 2560
#define CAMERA_FRAMERATE 100 // If fps higher than what the thread can handle, it will just run lower fps.


int main(const int argc, const char **argv)
{
    // Looking at image, mask and color mask 1, we can see that the hand is detected, and the ball. Want to not detect the hand

    std::cout << "Press ESC to stop. (Does not work if no window is displayed)" << std::endl;

    cv::Mat image, mask, HSV;
    lccv::PiCamera cam;

    // Was (0, 120, 120) and (10, 255, 255).
    // Lightings conditions such as sunlight might detect hands and face
    cv::Scalar hsv_lower(0, 120, 120);  // 0, 150, 50
    cv::Scalar hsv_upper(10, 255, 200); // 15, 255, 255

    cam.options->video_width = CAMERA_WIDTH;
    cam.options->video_height = CAMERA_HEIGHT;
    cam.options->framerate = CAMERA_FRAMERATE;
    cam.options->verbose = true;
    // cam.options->list_cameras = true;

    //   cv::namedWindow("Video", cv::WINDOW_NORMAL);
    //   cv::namedWindow("Mask", cv::WINDOW_NORMAL);

    cam.startVideo();

    while (true)
    {
        if (!cam.getVideoFrame(image, 1000))
        {
            std::cerr << "Timeout error" << std::endl;
        }
        else
        {
            std::cout << image.at<cv::Vec3b>(0, 0) << std::endl;

        }
    }
}