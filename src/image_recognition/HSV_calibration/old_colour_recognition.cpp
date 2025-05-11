#include <iostream>

// For the image recognition and
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
// A interface between the camera and OpenCV
#include <lccv.hpp>

#include <open62541pp/open62541pp.hpp>

#include <thread>
#include <chrono>

// Sources
// From image_recognition/first_iteration/color_recognition.cpp
// From image_recognition/main/server_services_node.cpp

#define CAMERA_HEIGHT 864    // Can be SD: 480, HD: 720, FHD: 1080, QHD: 1440,  y
#define CAMERA_WIDTH 1804    // Can be SD: 640, HD: 1280, FHD: 1920, QHD: 2560, x
#define CAMERA_FRAMERATE 55 // If fps higher than what the thread can handle, it will just run lower fps.

/**
 * @brief Function to calculate the center of a contour using moments
 * @param contour - The contour to calculate the center of
 * @return The center of the contour as cv::Point
 */
cv::Point calculateCenter(const std::vector<cv::Point> &contour)
{
    cv::Moments M = cv::moments(contour);
    if (M.m00 != 0)
    {
        // See https://docs.opencv.org/3.4/d8/d23/classcv_1_1Moments.html for center of mass (x¯, y¯)
        return cv::Point(static_cast<int>(M.m10 / M.m00), static_cast<int>(M.m01 / M.m00));
    }
    return cv::Point(-1, -1); // If no center is found, return (-1, -1)
}

int main()
{

    cv::Point new_center(0, 0);
    // cv::Point old_center(0, 0);

    float radius;
    cv::Point2f enclosingCenter;

    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration<double, std::milli>(end - start);

    uint32_t delay_server_iterate = 0;

    // -------------------------- OpenCV -------------------------- //
    std::cout << "Press ESC to stop. (Does not work if no window is displayed)" << std::endl;

    cv::Mat image, mask, HSV;
    lccv::PiCamera cam(0);

    // Was (0, 120, 120) and (10, 255, 255).
    // Lightings conditions such as sunlight might detect hands and face
    cv::Scalar hsv_upper(108, 231, 2255); // 15, 255, 255
    cv::Scalar hsv_lower(97, 103, 0);  // 0, 150, 50

    cam.options->video_width = CAMERA_WIDTH;
    cam.options->video_height = CAMERA_HEIGHT;
    cam.options->framerate = CAMERA_FRAMERATE;
    cam.options->verbose = true;
    // cam.options->list_cameras = true;

    cam.startVideo();

    sleep(1);

    if (!cam.getVideoFrame(image, 1000))
    {
        std::cerr << "Timeout error" << std::endl;
    }
    else
    {
        start = std::chrono::high_resolution_clock::now();
        // This is from the python file "test_opencv.py" and https://github.com/hcglhcgl/BallDetection/blob/master/balls.cpp

        cv::imwrite("report/full_image_colour_recog.jpg", image);

        cv::cvtColor(image, HSV, cv::COLOR_BGR2HSV);

        cv::inRange(HSV, hsv_lower, hsv_upper, mask);

        cv::imwrite("report/pre_erode_colour_recog.jpg", mask);

        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 3);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 3);

        cv::imwrite("report/post_erode_colour_recog.jpg", mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty())
        {
            std::vector<cv::Point> largestContour = *std::max_element(contours.begin(), contours.end(),
                                                                      [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
                                                                      {
                                                                          return cv::contourArea(a) < cv::contourArea(b);
                                                                      }); // Lambda function. Finds the largest contour

            cv::minEnclosingCircle(largestContour, enclosingCenter, radius);

            new_center = calculateCenter(largestContour);
        }

        // if (new_center.x == 0 && new_center.y == 0) {
        //     continue;
        // }

        // if difference between old and new center is less than x, do not send data ???
        // if do fi

        // std::cout << "New center.x: " << new_center.x << " New center.y: " << new_center.y << std::endl;

        // // quit on q button
        // if (cv::waitKey(1) == 'q') {
        // if (cv::waitKey(1) == 'q' && !first_save) {
        //     cv::imwrite("pre-mask.jpg", pre_mask);
        //     cv::imwrite("post-mask.jpg", mask);
        //     cv::imwrite("image.jpg", image);
        //     first_save = true;
        //     std::cout << "Images saved" << std::endl;
        // }

        // // quit on q button
        // if (cv::waitKey(1) == 'q' && first_save) {
        //     break;
        // }
    }

    cam.stopVideo();
    std::cout << "End of opencv" << std::endl;

    std::cout << "End of file!" << std::endl;
    return 0;
}