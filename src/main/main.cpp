#include <iostream>

// For the image recognition and
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
// A interface between the camera and OpenCV
#include <lccv.hpp>

#include <open62541pp/open62541pp.hpp>

#include <thread>
#include <chrono>

#include "includes/kalman.cpp"
#include "includes/kalman.hpp"


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
    // std::cout << "M.m00: " << M.m00 << std::endl;
    if (M.m00 != 0)
    {
        // See https://docs.opencv.org/3.4/d8/d23/classcv_1_1Moments.html for center of mass (x¯, y¯)
        return cv::Point(static_cast<int>(M.m10 / M.m00), static_cast<int>(M.m01 / M.m00));
    }
    return cv::Point(0, 0); // If no center is found, return (-1, -1)
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

    // node ids = {namespace, id}
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

    cv::Mat mask, HSV, sub_image;
    cv::Rect roi;
    lccv::PiCamera cam(1);

    // Was (0, 120, 120) and (10, 255, 255).
    // Lightings conditions such as sunlight might detect hands and face
    cv::Scalar hsv_lower(0, 150, 120);  // 0, 150, 50
    cv::Scalar hsv_upper(10, 255, 200); // 15, 255, 255

    cam.options->video_width = CAMERA_WIDTH;
    cam.options->video_height = CAMERA_HEIGHT;
    cam.options->framerate = CAMERA_FRAMERATE;
    cam.options->verbose = true;
    // cam.options->list_cameras = true;

    cv::namedWindow("Video", cv::WINDOW_NORMAL);
    cv::namedWindow("Post-Mask", cv::WINDOW_NORMAL);

    bool object_found = false;
    bool first_object_find = false;
    float radius_multiplier = 2.0f;
    cv::Point old_refrence_point;
    // int x_cut, y_cut, x_height, y_width, x_noe, y_noe;
    cv::Point start_point(0, 0), end_point(0, 0);

    cam.startVideo();
    // std::cout << cv::getBuildInformation() << std::endl;
    // std::cout << cv::useOptimized() << std::endl;

    // ----------------- KALMANFILTER ----------------- //


    //**********************************************************
	// PARAMETRIZATION: KALMAN INIT
	//**********************************************************
	// First parameter sets the predefenied values for:
	// 1 -> Constant Velocity Model
	// 2 -> Accelaration Model
	//**********************************************************
	int iKalmanMode = 2;
    int iDebugMode = true;

    int iTextOffsetCorrection_1 = 15; // PutText function has a small offset in y axis depending on the simbol you print (* or _)
	int iTextOffsetCorrection_2 = 7;

    cv::Scalar K_corr_Color(0, 0, 255);
	cv::Scalar K_pred_Color(0, 255, 0);
	cv::Scalar GT_Color(255, 0, 0);

    int iTextSize1 = 1;		// Test 3.1 and 3.3
    int iTextSize2 = 1.5;	// Test 3.1 and 3.3
    int iTextThickness = 1; // Test 3.1 and 3.3
    int iTestsOffset = 20;	// Test 3.1 and 3.3

    int iFrameNum = 0;

    cv::Point kalmanBall;

    kalmantracking::kalman kalmanKF(iKalmanMode, iDebugMode);


    while (true)
    {
        cv::Mat image;
        if (!cam.getVideoFrame(image, 1000))
        {
            std::cerr << "Timeout error" << std::endl;
        }
        else
        {
            // start = std::chrono::high_resolution_clock::now();
            // This is from the python file "test_opencv.py" and https://github.com/hcglhcgl/BallDetection/blob/master/balls.cpp

            // CHECK if point within rectanle. But would not work on first iter
            // if object found, cut image x-, y- to x+, y+
            
            // if (object_found)
            // {

            //     start_point.x = std::max(0.f, std::abs(new_center.x - radius_multiplier*((int)radius)));
            //     start_point.y = std::max(0.f, std::abs(new_center.y - radius_multiplier*((int)radius)));
            //     end_point.x = std::min(1280.f, std::abs(new_center.x + radius_multiplier*((int)radius)));
            //     end_point.y = std::min(720.f, std::abs(new_center.y + radius_multiplier*((int)radius)));
    
            //     std::cout << "start_point.x: " << start_point.x << " start_point.y: " << start_point.y << " radius: " << ((int)radius) << std::endl;
            //     std::cout << "end_point.x: " << end_point.x << " end_point.y: " << end_point.y << std::endl;

            //     roi = cv::Rect(start_point, end_point);

            //     image = image(roi);

            // }
            
            // std::cout << "Image size: " << image.size() << std::endl;

            if(image.empty())
            {
                std::cerr << "Image is empty" << std::endl;
                continue;
            }

            cv::cvtColor(image, HSV, cv::COLOR_BGR2HSV);

            cv::inRange(HSV, hsv_lower, hsv_upper, mask);

            // cv::imshow("Pre-Mask", pre_mask);

            cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 3);
            cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 3);

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            std::vector<cv::Point> largestContour;

            if (!contours.empty())
            {
                largestContour = *std::max_element(contours.begin(), contours.end(),
                                                                          [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
                                                                          {
                                                                              return cv::contourArea(a) < cv::contourArea(b);
                                                                          }); // Lambda function. Finds the largest contour

                cv::minEnclosingCircle(largestContour, enclosingCenter, radius);
            }

            cv::circle(image, enclosingCenter, radius, cv::Scalar(0, 255, 0), 2); // Draw on image, at point at enclosingCenter,

            // new_center = calculateCenter(largestContour);

            std::cout << "enclosing centre.x: " << enclosingCenter.x << " enclosing centre.y: " << enclosingCenter.y << std::endl;
            // std::cout << "New center.x: " << new_center.x << " New center.y: " << new_center.y << std::endl;

            kalmanBall = kalmanKF.Predict(enclosingCenter);

            // Point p1 = Point(kalmanBall.x-10, kalmanBall.y-10); //Test 3.3
            // Point p2 = Point(kalmanBall.x + 10, kalmanBall.y + 10); //Test 3.3
            Point p1 = Point(kalmanBall.x - 10, kalmanBall.y - 10); // Test 3.3
            Point p2 = Point(kalmanBall.x + 10, kalmanBall.y + 10); // Test 3.3

            if (kalmanKF.getStatus() == "Predicted") 
            {
                cv::rectangle(image, p1, p2, K_pred_Color, 1, 8, 0);
            }
            else
            {
                cv::rectangle(image, p1, p2, K_corr_Color, 1, 8, 0);
            }

            for (int i = 0; i < kalmanKF.getGroundTruthList().size(); i++)
            {
                putText(image, "o", Point(kalmanKF.getGroundTruthList()[i].x, kalmanKF.getGroundTruthList()[i].y + iTextOffsetCorrection_2), FONT_HERSHEY_COMPLEX, iTextSize2, GT_Color, iTextThickness);
            }
            for (int i = 0; i < kalmanKF.getPredictedList().size(); i++)
            {
                putText(image, "*", Point(kalmanKF.getPredictedList()[i].x, kalmanKF.getPredictedList()[i].y + iTextOffsetCorrection_1), FONT_HERSHEY_COMPLEX, iTextSize2, K_pred_Color, iTextThickness);
            }
            for (int i = 0; i < kalmanKF.getCorrectedList().size(); i++)
            {
                putText(image, "*", Point(kalmanKF.getCorrectedList()[i].x, kalmanKF.getCorrectedList()[i].y + iTextOffsetCorrection_1), FONT_HERSHEY_COMPLEX, iTextSize2, K_corr_Color, iTextThickness);
            }

            if (iDebugMode)
                cout << "Ball.x =  " << enclosingCenter.x << endl
                     << "Ball.y =  " << enclosingCenter.y << endl;
            if (iDebugMode)
                cout << "KBall.x = " << kalmanBall.x << endl
                     << "KBall.y = " << kalmanBall.y << endl;

                     putText(image, "Ground Truth (ball)", Point(10, 15 + iTestsOffset), FONT_HERSHEY_SIMPLEX, iTextSize1, GT_Color, iTextThickness);   // Test 3.1 and 3.3
                     putText(image, "Kalman Prediction", Point(10, 40 + iTestsOffset), FONT_HERSHEY_SIMPLEX, iTextSize1, K_pred_Color, iTextThickness); // Test 3.1 and 3.3
                     putText(image, "Kalman Corrected", Point(10, 65 + iTestsOffset), FONT_HERSHEY_SIMPLEX, iTextSize1, K_corr_Color, iTextThickness);  // Test 3.1 and 3.3
 
                     // putText(image, "Ground Truth (ball)", Point(10, 50 + iTestsOffset), FONT_HERSHEY_SIMPLEX, iTextSize1, GT_Color, iTextThickness);//Test 3.2
                     // putText(image, "Kalman Prediction", Point(10, 100 + iTestsOffset), FONT_HERSHEY_SIMPLEX, iTextSize1, K_pred_Color, iTextThickness);//Test 3.2
                     // putText(image, "Kalman Corrected", Point(10, 150 + iTestsOffset), FONT_HERSHEY_SIMPLEX, iTextSize1, K_corr_Color, iTextThickness);//Test 3.2
 
                     // putText(image, "Frame " + to_string(iFrameNum), Point(10, 200 + iTestsOffset), FONT_HERSHEY_COMPLEX, iTextSize1, Scalar(255,255,255), iTextThickness);//Test 3.2
                     putText(image, "Frame " + to_string(iFrameNum), Point(10, 90 + iTestsOffset), FONT_HERSHEY_COMPLEX, iTextSize1, Scalar(255, 255, 255), iTextThickness); // Test 3.2 and 3.3



            // // No object found
            // if (new_center.x == 0 && new_center.y == 0) {
            //     object_found = false;
            //     // continue;
            // }
            // else
            // {
            //     // std::cout << "Object found" << std::endl;
            //     object_found = true;
            //     if (!first_object_find)
            //     {
            //         std::cout << "First object found" << std::endl;
            //         old_refrence_point = new_center;
            //         first_object_find = true;
            //     }
            // }

            // if (object_found)
            // {
            //     cv::rectangle(image, start_point, end_point, cv::Scalar(0, 0, 255), 3);
            // }

            // if difference between old and new center is less than x, do not send data ???
            // if do fi

            // std::cout << "New center.x: " << new_center.x << " New center.y: " << new_center.y << std::endl;

            // cv::circle(image, new_center, 4, cv::Scalar(0, 0, 255), -1); // Draw on mask, at point new_center
            cv::circle(image, enclosingCenter, 4, cv::Scalar(0, 0, 255), -1); // Draw on mask, at point new_center
            // image is the Image from the camera (stream)

            cv::rotate(image, image, cv::ROTATE_90_COUNTERCLOCKWISE);
            cv::rotate(mask, mask, cv::ROTATE_90_COUNTERCLOCKWISE);

            cv::imshow("Video", image);
            // mask is a black and white image from the InRange function
            cv::imshow("Post-Mask", mask);

            opcua::services::writeDataValue(server, cameraNodeXId, opcua::DataValue(opcua::Variant(new_center.x)));
            opcua::services::writeDataValue(server, cameraNodeYId, opcua::DataValue(opcua::Variant(new_center.y)));
            opcua::services::writeDataValue(server, cameraNodeRadiusId, opcua::DataValue(opcua::Variant((int)radius)));

            // auto end = std::chrono::high_resolution_clock::now();

            // elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

            delay_server_iterate = server.runIterate();
            // std::this_thread::sleep_for(std::chrono::milliseconds(delay_server_iterate));

            // To save images.
            // if (cv::waitKey(1) == 'q' && !first_save) {
            //     cv::imwrite("pre-mask.jpg", pre_mask);
            //     cv::imwrite("post-mask.jpg", mask);
            //     cv::imwrite("image.jpg", image);
            //     first_save = true;
            //     std::cout << "Images saved" << std::endl;
            // }

            // quit on q button
            iFrameNum++;

            if (cv::waitKey(1) == 'q')
            {
                break;
            }
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
