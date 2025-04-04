#include <iostream>

// For the image recognition and
#include <opencv2/opencv.hpp>
// A interface between the camera and OpenCV
#include <lccv.hpp>

#define CAMERA_HEIGHT 720    // Can be SD: 480, HD: 720, FHD: 1080, QHD: 1440
#define CAMERA_WIDTH 1280    // Can be SD: 640, HD: 1280, FHD: 1920, QHD: 2560
#define CAMERA_FRAMERATE 100 // If fps higher than what the thread can handle, it will just run lower fps.



int main() {
    cv::Mat image_camera1, image_camera2;
    lccv::PiCamera cam1(0), cam2(1);

    cv::namedWindow("Camera 1", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Camera 2", cv::WINDOW_AUTOSIZE);


    cam1.options->video_width = CAMERA_WIDTH;
    cam1.options->video_height = CAMERA_HEIGHT;
    cam1.options->framerate = CAMERA_FRAMERATE;
    cam1.options->verbose = true;

    cam2.options->video_width = CAMERA_WIDTH;
    cam2.options->video_height = CAMERA_HEIGHT;
    cam2.options->framerate = CAMERA_FRAMERATE;
    cam2.options->verbose = true;

    cam1.startVideo();
    cam2.startVideo();

    while (true) {
        if (!cam1.getVideoFrame(image_camera1, 1000)) {
            std::cerr << "Timeout error" << std::endl;
        }

        if (!cam2.getVideoFrame(image_camera2, 1000)) {
            std::cerr << "Timeout error" << std::endl;
        }

        cv::rotate(image_camera1, image_camera1, cv::ROTATE_90_CLOCKWISE);
        cv::rotate(image_camera2, image_camera2, cv::ROTATE_90_COUNTERCLOCKWISE);

        cv::imshow("Camera 1", image_camera1);
        cv::imshow("Camera 2", image_camera2);

        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    cam1.stopVideo();
    cam2.stopVideo();
    cv::destroyAllWindows();
    std::cout << "EOF" << std::endl;
    return 0;
}