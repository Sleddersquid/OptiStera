#include <iostream>

// For the image recognition and
#include <opencv2/opencv.hpp>
// A interface between the camera and OpenCV
#include <lccv.hpp>

#include <open62541pp/open62541pp.hpp>

#include <thread>
#include <chrono>

// Sources
// From image_recognition/first_iteration/color_recognition.cpp
// From image_recognition/main/server_services_node.cpp

#define CAMERA_HEIGHT 720    // Can be SD: 480, HD: 720, FHD: 1080, QHD: 1440
#define CAMERA_WIDTH 1280    // Can be SD: 640, HD: 1280, FHD: 1920, QHD: 2560
#define CAMERA_FRAMERATE 100 // If fps higher than what the thread can handle, it will just run lower fps.

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

    // -------------------------- OPCUA -------------------------- //
    opcua::ServerConfig config;
    config.setApplicationName("open62541pp server objectRecgonition");
    config.setApplicationUri("urn:open62541pp.server.application");
    config.setProductUri("https://open62541pp.github.io");

    opcua::Server server(std::move(config));

    // node ids -> {namespace, id}
    opcua::NodeId parentNodeId = {1, 1000};
    opcua::NodeId cameraNodeXId = {1, 1001};
    opcua::NodeId cameraNodeYId = {1, 1002};
    opcua::NodeId cameraNodeRadiusId = {1, 1003};

    opcua::Result<opcua::NodeId> parentNode =
        opcua::services::addVariable(
            server,
            opcua::ObjectId::ObjectsFolder,
            parentNodeId,
            "CameraValues",
            opcua::VariableAttributes{}
                .setAccessLevel(UA_ACCESSLEVELMASK_READ)
                .setDataType<int>()
                .setValue(opcua::Variant(1001)),
            opcua::VariableTypeId::BaseDataVariableType,
            opcua::ReferenceTypeId::HasComponent);

    opcua::Result<opcua::NodeId> Node_Camera_X =
        opcua::services::addVariable(
            server,
            parentNodeId,
            cameraNodeXId,
            "CameraX",
            opcua::VariableAttributes{}
                .setAccessLevel(UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE)
                .setDataType<int>()
                .setValue(opcua::Variant(new_center.x)),
            opcua::VariableTypeId::BaseDataVariableType,
            opcua::ReferenceTypeId::HasComponent);

    opcua::Result<opcua::NodeId> Node_Camera_Y =
        opcua::services::addVariable(
            server,
            parentNodeId,
            cameraNodeYId,
            "CameraY",
            opcua::VariableAttributes{}
                .setAccessLevel(UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE)
                .setDataType<int>()
                .setValue(opcua::Variant(new_center.y)),
            opcua::VariableTypeId::BaseDataVariableType,
            opcua::ReferenceTypeId::HasComponent);

    opcua::Result<opcua::NodeId> Node_Camera_Radius =
        opcua::services::addVariable(
            server,
            parentNodeId,
            cameraNodeRadiusId,
            "CameraRadius",
            opcua::VariableAttributes{}
                .setAccessLevel(UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE)
                .setDataType<int>()
                .setValue(opcua::Variant((int)radius)),
            opcua::VariableTypeId::BaseDataVariableType,
            opcua::ReferenceTypeId::HasComponent);

    // -------------------------- OpenCV -------------------------- //
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
            start = std::chrono::high_resolution_clock::now();
            // This is from the python file "test_opencv.py" and https://github.com/hcglhcgl/BallDetection/blob/master/balls.cpp
            cv::cvtColor(image, HSV, cv::COLOR_BGR2HSV);

            cv::inRange(HSV, hsv_lower, hsv_upper, mask);

            cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 3);
            cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 3);

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

            // if difference between old and new center is less than x, do not send data
            // if do fi

            // std::cout << "New center.x: " << new_center.x << " New center.y: " << new_center.y << std::endl;

            cv::circle(image, new_center, 5, cv::Scalar(0, 0, 255), -1); // Draw on mask, at point new_center,
            // image is the Image from the camera (stream)
            cv::imshow("Video", image);
            // mask is a black and white image from the InRange function
            cv::imshow("Mask", mask);

            opcua::services::writeDataValue(server, cameraNodeXId, opcua::DataValue(opcua::Variant(new_center.x)));
            opcua::services::writeDataValue(server, cameraNodeYId, opcua::DataValue(opcua::Variant(new_center.y)));
            opcua::services::writeDataValue(server, cameraNodeRadiusId, opcua::DataValue(opcua::Variant((int)radius)));

            auto end = std::chrono::high_resolution_clock::now();

            elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

            delay_server_iterate = server.runIterate();
            // std::this_thread::sleep_for(std::chrono::milliseconds(delay_server_iterate));

            // // quit on q button
            if (cv::waitKey(1) == 'q')
            {
                break;
            } // Can be one-liner but for readability
        }
    }

    server.stop();

    cam.stopVideo();
    cv::destroyWindow("Video");
    cv::destroyWindow("Mask");
    cv::destroyAllWindows();
    std::cout << "End of opencv" << std::endl;

    std::cout << "End of file!" << std::endl;
    return 0;
}
