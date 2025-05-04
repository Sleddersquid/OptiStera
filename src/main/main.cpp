#include <iostream>

// For the image recognition and
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
// A interface between the camera and OpenCV
#include <lccv.hpp>

#include <open62541pp/open62541pp.hpp>

#include <thread>
#include <chrono>
#include <atomic>
#include <string>
#include <fstream>

#include <numeric> // for std::accumulate

#include "includes/kalman.cpp"
#include "includes/kalman.hpp"


// Sources
// From image_recognition/first_iteration/color_recognition.cpp
// From image_recognition/main/server_services_node.cpp

#define CAMERA_HEIGHT 720    // Can be SD: 480, HD: 720, FHD: 1080, QHD: 1440
#define CAMERA_WIDTH 1280    // Can be SD: 640, HD: 1280, FHD: 1920, QHD: 2560
#define CAMERA_FRAMERATE 100 // If fps higher than what the thread can handle, it will just run lower fps.
#define FPS_SAMPLES 100

#define TEMP_UPDATE 10 // in s

// #define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))


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

// From Wiring_constants.h
template<typename T>
T constrain(T p, T low, T high) {
    return ((p) < (low) ? (low) : ((p) > (high) ? (high) : (p)));
}

// Must be global for signal handler

// -------------------------- SIGNAL HANDLER -------------------------- //

std::thread temp_thread;

std::atomic<bool> get_temp_running = true;
std::atomic<bool> camera_running = true;

void signalHandler(int signum) {
    get_temp_running = false;
    camera_running = false;
}

int main()
{
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);
    
    cv::Point new_center(0, 0);
    // cv::Point old_center(0, 0);
    
    float radius;
    
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration<double, std::milli>(end - start);
    
    // uint32_t delay_server_iterate = 0;
    opcua::ServerConfig config;
    config.setApplicationName("open62541pp server objectRecognition");
    config.setApplicationUri("urn:open62541pp.server.application");
    config.setProductUri("https://open62541pp.github.io");

    UA_DurationRange publishingIntervalLimits_custom;
    publishingIntervalLimits_custom.min = 20;
    publishingIntervalLimits_custom.max = 25;

    config->publishingIntervalLimits = publishingIntervalLimits_custom;

    opcua::Server server(std::move(config));
    uint16_t server_delay_ms = 20;

    // node ids = {namespace, id}
    opcua::NodeId objectParentNodeId = {1, 1000};
    opcua::NodeId cameraNodeXId = {1, 1001};
    opcua::NodeId cameraNodeYId = {1, 1002};
    opcua::NodeId cameraNodeRadiusId = {1, 1003};
    opcua::NodeId enableObjectRecognitionId = {1, 1004};
    opcua::NodeId statusObjectRecognitionId = {1, 1005};
    bool object_recognition_enabled = false;

    opcua::Result<opcua::NodeId> parentNode =
        opcua::services::addVariable(
            server,
            opcua::ObjectId::ObjectsFolder,
            objectParentNodeId,
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
            objectParentNodeId,
            cameraNodeXId,
            "CameraX",
            opcua::VariableAttributes{}
                .setAccessLevel(UA_ACCESSLEVELMASK_READ)
                .setDataType<int>()
                .setValue(opcua::Variant(new_center.x)),
            opcua::VariableTypeId::BaseDataVariableType,
            opcua::ReferenceTypeId::HasComponent);

    opcua::Result<opcua::NodeId> Node_Camera_Y =
        opcua::services::addVariable(
            server,
            objectParentNodeId,
            cameraNodeYId,
            "CameraY",
            opcua::VariableAttributes{}
                .setAccessLevel(UA_ACCESSLEVELMASK_READ)
                .setDataType<int>()
                .setValue(opcua::Variant(new_center.y)),
            opcua::VariableTypeId::BaseDataVariableType,
            opcua::ReferenceTypeId::HasComponent);

    opcua::Result<opcua::NodeId> Node_Camera_Radius =
        opcua::services::addVariable(
            server,
            objectParentNodeId,
            cameraNodeRadiusId,
            "CameraRadius",
            opcua::VariableAttributes{}
                .setAccessLevel(UA_ACCESSLEVELMASK_READ)
                .setDataType<float>()
                .setValue(opcua::Variant(radius)),
            opcua::VariableTypeId::BaseDataVariableType,
            opcua::ReferenceTypeId::HasComponent);

    // Enable object recognition from the RCU
    opcua::Result<opcua::NodeId> enableObjectNode =
        opcua::services::addVariable(
            server,
            objectParentNodeId,
            enableObjectRecognitionId,
            "enableObjectRecognition",
            opcua::VariableAttributes{}
                .setAccessLevel(UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE)
                .setDataType<bool>()
                .setValue(opcua::Variant(false)),
            opcua::VariableTypeId::BaseDataVariableType,
            opcua::ReferenceTypeId::HasComponent);

    // Feedback of status of object recognition to the RCU
    opcua::Result<opcua::NodeId> statusObjectNode =
        opcua::services::addVariable(
            server,
            objectParentNodeId,
            statusObjectRecognitionId,
            "statusObjectRecognition",
            opcua::VariableAttributes{}
                .setAccessLevel(UA_ACCESSLEVELMASK_READ)
                .setDataType<bool>()
                .setValue(opcua::Variant(false)),
            opcua::VariableTypeId::BaseDataVariableType,
            opcua::ReferenceTypeId::HasComponent);

            
    temp_thread = std::thread([&server]() {
        float temp = 0.0f;
        std::string file_content;
        opcua::NodeId tempMeasurementNodeId = {1, 1006};
            
        // Server temp node
        opcua::Result<opcua::NodeId> Node_RaspberryPi_temp =
        opcua::services::addVariable(
            server,
            opcua::VariableId::Server_ServerStatus,
            tempMeasurementNodeId,
            "RaspberryPiTemperature",
            opcua::VariableAttributes{}
                .setAccessLevel(UA_ACCESSLEVELMASK_READ)
                .setDataType<float>()
                .setValue(opcua::Variant(0.0f)),
            opcua::VariableTypeId::BaseDataVariableType,
            opcua::ReferenceTypeId::HasComponent);

        auto last_read = std::chrono::high_resolution_clock::now();
        int32_t debounce_interval = TEMP_UPDATE * 1000; // in ms
        
        while (get_temp_running) {
            // Sleep here because of continue
            // std::this_thread::sleep_for(std::chrono::seconds(10));
            
            auto now = std::chrono::high_resolution_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_read).count() > debounce_interval){
                last_read = now;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }

            // Open file
            std::ifstream temp_file("/sys/class/thermal/thermal_zone0/temp");
            // if file not oppened
            if (!temp_file.is_open()) {
                std::cerr << "Error opening file" << std::endl;
                continue;
            }

            // Read the file
            std::getline(temp_file, file_content);

            // Close the file
            temp_file.close();

            temp = (std::stof(file_content) / 1000.0f);

            opcua::services::writeDataValue(server, tempMeasurementNodeId, opcua::DataValue(opcua::Variant(temp)));
            
            // std::cout << "tamp: " << temp << std::endl;
        }
    });


    // -------------------------- OpenCV -------------------------- //
    std::cout << "Press ESC to stop. (Does not work if no window is displayed)" << std::endl;

    cv::Mat mask_o, mask_r, HSV, sub_image, mask;
    // cv::Rect roi;


    // Was (0, 120, 120) and (10, 255, 255).
    // Lightings conditions such as sunlight might detect hands and face
    cv::Scalar hsv_lower(0, 120, 120);  // 0, 150, 50
    cv::Scalar hsv_upper(15, 255, 255); // 15, 255, 255

    // cv::Scalar hsv_upper_o(179, 255, 255);  // 0, 150, 50
    // cv::Scalar hsv_lower_o(177, 92, 89); // 15, 255, 255

    // cv::Scalar hsv_upper_r(6, 236, 255); // 15, 255, 255
    // cv::Scalar hsv_lower_r(0, 174, 163);  // 0, 150, 50

    lccv::PiCamera cam(0);

    cam.options->video_width = CAMERA_WIDTH;
    cam.options->video_height = CAMERA_HEIGHT;
    cam.options->framerate = CAMERA_FRAMERATE;
    cam.options->verbose = true;
    cam.options->setExposureMode(Exposure_Modes::EXPOSURE_SHORT);
    cam.options->setWhiteBalance(WhiteBalance_Modes::WB_INDOOR);
    // cam.options->list_cameras = true;

    cam.startVideo();

    // cv::namedWindow("Post-Mask", cv::WINDOW_NORMAL);
    // cv::namedWindow("Video", cv::WINDOW_AUTOSIZE);

    // bool object_found = false;
    // bool first_object_find = false;
    // float radius_multiplier = 2.0f;
    cv::Point old_refrence_point;
    // int x_cut, y_cut, x_height, y_width, x_noe, y_noe;
    cv::Point start_point(0, 0), end_point(0, 0);

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
    int iKalmanMode = 1;
    int iDebugMode = false;
    
    int iTextOffsetCorrection_1 = 15; // PutText function has a small offset in y axis depending on the simbol you print (* or _)
    int iTextOffsetCorrection_2 = 7;
    
    cv::Scalar K_corr_Color(0, 0, 255);
    cv::Scalar K_pred_Color(0, 255, 0);
    cv::Scalar GT_Color(255, 0, 0);
    
    int iTextSize1 = 1;     // Test 3.1 and 3.3
    int iTextSize2 = 1.5;   // Test 3.1 and 3.3
    int iTextThickness = 1; // Test 3.1 and 3.3
    int iTestsOffset = 20;  // Test 3.1 and 3.3
    
    int iFrameNum = 0;
    cv::Point2f enclosingCenter;
    cv::Point kallman_ball(0, 0);
    
    kalmantracking::kalman kalmanKF(iKalmanMode, iDebugMode);
    
    int image_count = 0;
    int frame_count = 0;
    float fps = -1;
    // std::vector<float> fps_counter_vec;
    
    bool enable_print = false;

    while (camera_running)
    {
        opcua::services::writeDataValue(server, cameraNodeXId, opcua::DataValue(opcua::Variant(kallman_ball.x)));
        opcua::services::writeDataValue(server, cameraNodeYId, opcua::DataValue(opcua::Variant(kallman_ball.y)));
        opcua::services::writeDataValue(server, cameraNodeRadiusId, opcua::DataValue(opcua::Variant(radius)));

        opcua::services::writeDataValue(server, statusObjectRecognitionId, opcua::DataValue(opcua::Variant(object_recognition_enabled)));
        
        object_recognition_enabled = opcua::services::readDataValue(server, enableObjectRecognitionId).value().value().to<bool>();
        server_delay_ms = server.runIterate();

        if(object_recognition_enabled == false)
        {
            // std::cout << "Object recognition disabled" << std::endl;
            // enclosingCenter = cv::Point(-1, -1);
            kallman_ball = cv::Point(-1, -1);
            radius = -1.0;
            // kalmanKF.Restart();
            std::this_thread::sleep_for(std::chrono::milliseconds(server_delay_ms));
            continue;
        }

        cv::Mat image;
        if (!cam.getVideoFrame(image, 1000))
        {
            object_recognition_enabled = false;
            std::cerr << "Timeout error" << std::endl;
        }
        else
        {
            if (image.empty()) {
                std::cerr << "Image is empty" << std::endl;
                continue;
            }

            object_recognition_enabled = true;

            frame_count++;

            cv::cvtColor(image, HSV, cv::COLOR_BGR2HSV);

            cv::GaussianBlur(HSV, HSV, cv::Size(7, 7), 0, 0);
            
            cv::inRange(HSV, hsv_lower, hsv_upper, mask);

            // cv::inRange(HSV, hsv_lower_o, hsv_upper_o, mask_o);
            // cv::inRange(HSV, hsv_lower_r, hsv_upper_r, mask_r);
            
            // mask = mask_o | mask_r;
            
            // if (cv::waitKey(1) == 32)
            // {
            //     // std::string filename = "RGB" + std::to_string(image_count++) + ".jpg";
            //     cv::imwrite("RBGB.jpg", image);
            //     cv::imwrite("HSV.jpg", HSV);
            //     std::cout << "Saved image: " << std::endl;
            // }
            // // cv::imshow("Pre-Mask", pre_mask);

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

            // std::cout << "Rasius: " << radius << std::endl;

            // if (frame_count >= 50)
            // {
            //     auto end = std::chrono::high_resolution_clock::now();
            //     fps = frame_count * 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

            //     frame_count = 0;
            //     fps_counter_vec.push_back(fps);

            //     start = std::chrono::high_resolution_clock::now();
            // }

            // if (fps > 0)
            // {
            //     std::ostringstream fps_label;
            //     fps_label << std::fixed << std::setprecision(2);
            //     fps_label << "FPS: " << fps;
            //     std::string fps_label_str = fps_label.str();
            //     std::cout << fps_label_str << std::endl;

            //     // cv::putText(image, fps_label_str.c_str(), cv::Point(CAMERA_HEIGHT - 50, 25), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
            // }

            // std::cout << "Ball.x =  " << enclosingCenter.x << std::endl
            //      << "Ball.y =  " << enclosingCenter.y << std::endl;
            // std::cout << "KBall.x = " << kallman_ball.x << std::endl
            //      << "KBall.y = " << kallman_ball.y << std::endl;

            // if (radius < 39.0f) {
            //     enclosingCenter = cv::Point(0, 0);
            // }

            // new_center = calculateCenter(largestContour);

            // enclosingCenter.x = iFrameNum*4 % CAMERA_WIDTH;
            // enclosingCenter.y = 512;

            // enclosingCenter.x = 0;
            // enclosingCenter.y = 0;

            // enclosingCenter = new_center;

            // std::cout << "enclosing centre.x: " << enclosingCenter.x << " enclosing centre.y: " << enclosingCenter.y << std::endl;
            // std::cout << "New center.x: " << new_center.x << " New center.y: " << new_center.y << std::endl;

            kallman_ball = kalmanKF.Predict((cv::Point2i)enclosingCenter);

            // Point p1 = Point(kallman_ball.x-10, kallman_ball.y-10)ø; //Test 3.3
            // // Point p2 = Point(kallman_ball.x + 10, kallman_ball.y + 10); //Test 3.3
            // Point p1 = Point(kallman_ball.x - 10, kallman_ball.y - 10); // Test 3.3
            // Point p2 = Point(kallman_ball.x + 10, kallman_ball.y + 10); // Test 3.3

            // // std::cout << kalmanKF.getStatus() << std::endl;

            // if (kalmanKF.getStatus() == "Predicted")
            // {
            //     cv::rectangle(image, p1, p2, K_pred_Color, 1, 8, 0);
            // }
            // else
            // {
            //     cv::rectangle(image, p1, p2, K_corr_Color, 1, 8, 0);
            // }

            // for (int i = 0; i < kalmanKF.getGroundTruthList().size(); i++)
            // {
            //     putText(image, "o", Point(kalmanKF.getGroundTruthList()[i].x - 7, kalmanKF.getGroundTruthList()[i].y + iTextOffsetCorrection_2), FONT_HERSHEY_COMPLEX, iTextSize2, GT_Color, iTextThickness);
            // }
            // for (int i = 0; i < kalmanKF.getPredictedList().size(); i++)
            // {
            //     putText(image, "*", Point(kalmanKF.getPredictedList()[i].x, kalmanKF.getPredictedList()[i].y + iTextOffsetCorrection_1), FONT_HERSHEY_COMPLEX, iTextSize2, K_pred_Color, iTextThickness);
            // }
            // for (int i = 0; i < kalmanKF.getCorrectedList().size(); i++)
            // {
            //     putText(image, "*", Point(kalmanKF.getCorrectedList()[i].x - 5, kalmanKF.getCorrectedList()[i].y + iTextOffsetCorrection_1), FONT_HERSHEY_COMPLEX, iTextSize2, K_corr_Color, iTextThickness);
            // }

            // putText(image, "Ground Truth (ball)", Point(10, 15 + iTestsOffset), FONT_HERSHEY_SIMPLEX, iTextSize1, GT_Color, iTextThickness);   // Test 3.1 and 3.3
            // putText(image, "Kalman Prediction", Point(10, 40 + iTestsOffset), FONT_HERSHEY_SIMPLEX, iTextSize1, K_pred_Color, iTextThickness); // Test 3.1 and 3.3
            // putText(image, "Kalman Corrected", Point(10, 65 + iTestsOffset), FONT_HERSHEY_SIMPLEX, iTextSize1, K_corr_Color, iTextThickness);  // Test 3.1 and 3.3

            // // // putText(image, "Ground Truth (ball)", Point(10, 50 + iTestsOffset), FONT_HERSHEY_SIMPLEX, iTextSize1, GT_Color, iTextThickness);//Test 3.2
            // // // putText(image, "Kalman Prediction", Point(10, 100 + iTestsOffset), FONT_HERSHEY_SIMPLEX, iTextSize1, K_pred_Color, iTextThickness);//Test 3.2
            // // // putText(image, "Kalman Corrected", Point(10, 150 + iTestsOffset), FONT_HERSHEY_SIMPLEX, iTextSize1, K_corr_Color, iTextThickness);//Test 3.2

            // // // putText(image, "Frame " + to_string(iFrameNum), Point(10, 200 + iTestsOffset), FONT_HERSHEY_COMPLEX, iTextSize1, Scalar(255,255,255), iTextThickness);//Test 3.2
            // putText(image, "Frame " + to_string(iFrameNum), Point(10, 90 + iTestsOffset), FONT_HERSHEY_COMPLEX, iTextSize1, Scalar(255, 255, 255), iTextThickness); // Test 3.2 and 3.3

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

            // // cv::circle(image, new_center, 4, cv::Scalar(0, 0, 255), -1); // Draw on image, at point new_center
            // cv::circle(image, enclosingCenter, 4, cv::Scalar(0, 0, 255), -1); // Draw on image, at point new_center
            // cv::circle(image, enclosingCenter, radius, cv::Scalar(0, 255, 0), 2); // Draw on image, at point at enclosingCenter,
            // // image is the Image from the camera (stream)

            // // cv::rotate(image, image, cv::ROTATE_90_COUNTERCLOCKWISE);
            // // cv::rotate(mask, mask, cv::ROTATE_90_COUNTERCLOCKWISE);

            // cv::imshow("Post-Mask", mask);
            // cv::imshow("Video", image);

            // mask is a black and white image from the InRange function
            // It is not until the server runs iterate that the values will be updated
            
            // auto end = std::chrono::high_resolution_clock::now();

            // elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

            // delay_server_iterate = server.runIterate();
            // std::this_thread::sleep_for(std::chrono::milliseconds(delay_server_iterate));

            kallman_ball.x = constrain(kallman_ball.x, 0, CAMERA_WIDTH);
            kallman_ball.y = constrain(kallman_ball.y, 0, CAMERA_HEIGHT);
            
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
        
        // for (auto fps : fps_counter_vec) {
            //     std::cout << "FPS: " << fps << std::endl;
    // }

    // std::cout << "Average FPS: " << std::accumulate(fps_counter_vec.begin(), fps_counter_vec.end(), 0.0) / fps_counter_vec.size() << std::endl;

    server.stop();
    cam.stopVideo();
    temp_thread.join(); // Join at bottom, to not hault camera if that would be the case
    
    // cv::destroyWindow("Video");
    // cv::destroyWindow("Mask");
    // cv::destroyAllWindows();
    std::cout << "End of opencv" << std::endl;

    std::cout << "End of file!" << std::endl;
    return 0;
}