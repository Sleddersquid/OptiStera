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

#define CAMERA_HEIGHT 864   // Can be SD: 480, HD: 720, FHD: 1080, QHD: 1440,  y
#define CAMERA_WIDTH 1804   // Can be SD: 640, HD: 1280, FHD: 1920, QHD: 2560, x
#define CAMERA_FRAMERATE 55 // If fps higher than what the thread can handle, it will just run lower fps.
#define FPS_SAMPLES 10

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

template <typename T>
T constrain(T p, T low, T high)
{ // From Arduino Wiring constants
    return ((p) < (low) ? (low) : ((p) > (high) ? (high) : (p)));
}

template <typename T>
class Local_max_min
{
private:
    T max = -1000000;
    T min = 1000000;
    std::vector<T> average_container;

public:
    Local_max_min() = default;

    void insert(T val)
    {
        if (val > max)
        {
            this->max = val;
        }
        if (val < min)
        {
            this->min = val;
        }
        average_container.push_back(val);
    }

    T getDiff()
    {
        // return std::accumulate(average_container.begin(), average_container.end(), 0.0) / (T)average_container.size();
        return this->max - this->min;
    }

    T getMax()
    {
        return this->max;
    }

    T getMin()
    {
        return this->min;
    }
};

// template <typename T, int N>
// class MaxVector {
//     private:
//         std::vector<T> noe;
//     public:
//         void insert(T val) {
//             if(max_array.size() >= N) {
//             }
//         }
// };

// -------------------------- SIGNAL HANDLER -------------------------- //

std::thread temp_thread;

// Must be global for signal handler
std::atomic<bool> temp_thread_running = true;
std::atomic<bool> camera_running = true;

// Signal handler for SIGTERM and SIGINT (add ctrl-c and stuff)
void signalHandler(int signum)
{
    temp_thread_running = false;
    camera_running = false;
}

int main()
{
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    // Over OPC-UA Serve and Node instansiation.
    cv::Point2f new_centre(0.0, 0.0);
    float radius;

    opcua::ServerConfig config;
    config.setApplicationName("open62541pp server objectRecognition");
    config.setApplicationUri("urn:open62541pp.server.application");
    config.setProductUri("https://open62541pp.github.io");

    UA_DurationRange publishingIntervalLimits_custom;
    publishingIntervalLimits_custom.min = 20;
    publishingIntervalLimits_custom.max = 25;

    // Pointer to the config object. DO NOT MODIFY THE POINTER OF CONFIG OBJECT
    config->publishingIntervalLimits = publishingIntervalLimits_custom;

    opcua::Server server(std::move(config));
    uint16_t server_delay_ms = 20;

    // node id = {namespace, id}
    opcua::NodeId objectParentNodeId = {1, 1000};
    opcua::NodeId cameraNodeXId = {1, 1001};
    opcua::NodeId cameraNodeYId = {1, 1002};
    opcua::NodeId cameraNodeRadiusId = {1, 1003};
    opcua::NodeId enableObjectRecognitionId = {1, 1004};
    opcua::NodeId statusObjectRecognitionId = {1, 1005};
    opcua::NodeId objectFoundId = {1, 1006};
    bool object_recognition_enabled = true; // Decides if object recognition should be enabled or not on startup
    bool object_found = false;              // If object found

    opcua::Result<opcua::NodeId> parentNode =
        opcua::services::addVariable(
            server,
            opcua::ObjectId::ObjectsFolder,
            objectParentNodeId,
            "CameraValues",
            opcua::VariableAttributes{}
                .setAccessLevel(UA_ACCESSLEVELMASK_READ)
                .setDataType<int>()
                .setValue(opcua::Variant(117)),
            opcua::VariableTypeId::BaseDataVariableType,
            opcua::ReferenceTypeId::Organizes);

    opcua::Result<opcua::NodeId> Node_Camera_X =
        opcua::services::addVariable(
            server,
            objectParentNodeId,
            cameraNodeXId,
            "CameraX",
            opcua::VariableAttributes{}
                .setAccessLevel(UA_ACCESSLEVELMASK_READ)
                .setDataType<UA_Float>()
                .setValue(opcua::Variant(new_centre.x)),
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
                .setDataType<UA_Float>()
                .setValue(opcua::Variant(new_centre.y)),
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
                .setDataType<UA_Float>()
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
                .setValue(opcua::Variant(object_recognition_enabled)),
            opcua::VariableTypeId::BaseDataVariableType,
            opcua::ReferenceTypeId::HasComponent);

    // Feedback of status of object recognition to the RCU. REMOVED
    opcua::Result<opcua::NodeId> statusObjectNode =
        opcua::services::addVariable(
            server,
            objectParentNodeId,
            statusObjectRecognitionId,
            "statusObjectRecognition",
            opcua::VariableAttributes{}
                .setAccessLevel(UA_ACCESSLEVELMASK_READ)
                .setDataType<UA_Boolean>()
                .setValue(opcua::Variant(object_recognition_enabled)),
            opcua::VariableTypeId::BaseDataVariableType,
            opcua::ReferenceTypeId::HasComponent);

    opcua::Result<opcua::NodeId> objectFoundNode =
        opcua::services::addVariable(
            server,
            objectParentNodeId,
            objectFoundId,
            "objectFound",
            opcua::VariableAttributes{}
                .setAccessLevel(UA_ACCESSLEVELMASK_READ)
                .setDataType<UA_Boolean>()
                .setValue(opcua::Variant(object_found)),
            opcua::VariableTypeId::BaseDataVariableType,
            opcua::ReferenceTypeId::HasComponent);

    // Look at this more closeley. It is kinda fuckery
    temp_thread = std::thread([&server]()
                              {
        float temp = 0.0f;
        std::string file_content;
        opcua::NodeId tempMeasurementNodeId = {1, 1007};
            
        // Server temp node
        opcua::Result<opcua::NodeId> Node_RaspberryPi_temp =
        opcua::services::addVariable(
            server,
            opcua::VariableId::Server_ServerStatus,
            tempMeasurementNodeId,
            "RaspberryPiTemperature",
            opcua::VariableAttributes{}
                .setAccessLevel(UA_ACCESSLEVELMASK_READ)
                .setDataType<UA_Float>()
                .setValue(opcua::Variant(temp)),
            opcua::VariableTypeId::BaseDataVariableType,
            opcua::ReferenceTypeId::HasComponent);

        auto last_read = std::chrono::high_resolution_clock::now();
        int32_t debounce_interval = TEMP_UPDATE * 1000; // in ms
        
        while (temp_thread_running) {
            
            auto now = std::chrono::high_resolution_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_read).count() > debounce_interval){
                last_read = now;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }

            std::ifstream temp_file("/sys/class/thermal/thermal_zone0/temp");
            // if file not oppened
            if (!temp_file.is_open()) {
                std::cerr << "Error opening file" << std::endl;
                continue;
            }

            std::getline(temp_file, file_content);

            temp_file.close();

            temp = (std::stof(file_content) / 1000.0f);

            opcua::services::writeDataValue(server, tempMeasurementNodeId, opcua::DataValue(opcua::Variant(temp)));
        } });

    // -------------------------- LCCV -------------------------- //
    lccv::PiCamera cam(0);

    cam.options->video_width = CAMERA_WIDTH;
    cam.options->video_height = CAMERA_HEIGHT;
    cam.options->framerate = CAMERA_FRAMERATE;
    cam.options->verbose = true;
    cam.options->denoise = "cdn_fast";
    cam.options->sharpness = 2.5f;
    cam.options->setExposureMode(Exposure_Modes::EXPOSURE_SHORT);
    cam.options->setWhiteBalance(WhiteBalance_Modes::WB_INDOOR);

    cam.startVideo();

    // ---------------------- KALMAN FILTER ---------------------- //

    // PARAMETRIZATION: KALMAN INIT
    //**********************************************************
    // First parameter sets the predefenied values for:
    // 1 -> Constant Velocity Model
    // 2 -> Accelaration Model
    //**********************************************************
    int iKalmanMode = 1;
    int iDebugMode = false;
    
    kalmantracking::kalman kalmanKF(iKalmanMode, iDebugMode);

    // ------------------------ OPENCV ------------------------ //

    cv::Point2f enclosingCenter;
    cv::Point2f kallman_ball(0, 0);

    cv::Mat image, sub_image;
    cv::Mat grey, mask;

    // For cropping
    cv::Point2f start_point(0.0, 0.0), end_point(0.0, 0.0);
    float image_cut_height = 120.0f;
    float image_cut_width = 110.0f;

    std::vector<cv::Point2f> measurements_point_vector;

    int folder_size = 0;

    while (camera_running)
    {
        opcua::services::writeDataValue(server, cameraNodeXId, opcua::DataValue(opcua::Variant(kallman_ball.x)));
        opcua::services::writeDataValue(server, cameraNodeYId, opcua::DataValue(opcua::Variant(kallman_ball.y)));
        opcua::services::writeDataValue(server, cameraNodeRadiusId, opcua::DataValue(opcua::Variant(radius)));

        opcua::services::writeDataValue(server, statusObjectRecognitionId, opcua::DataValue(opcua::Variant(object_recognition_enabled)));
        opcua::services::writeDataValue(server, objectFoundId, opcua::DataValue(opcua::Variant(object_found)));

        object_recognition_enabled = opcua::services::readDataValue(server, enableObjectRecognitionId).value().value().to<bool>();
        server_delay_ms = server.runIterate();

        // if (object_recognition_enabled == false)
        // {
        //     // enclosingCenter = cv::Point(-1, -1);
        //     kallman_ball = cv::Point(-1, -1);
        //     radius = -1.0;
        //     object_found = false;
        //     // kalmanKF.Restart();
        //     std::this_thread::sleep_for(std::chrono::milliseconds(server_delay_ms));
        //     continue;
        // }

        // start = std::chrono::high_resolution_clock::now();

        if (cam.getVideoFrame(image, 1000) == false)
        {
            object_recognition_enabled = false;
            std::cerr << "Timeout error" << std::endl;
        }
        else
        {
            // start = std::chrono::high_resolution_clock::now();
            if (image.empty())
            {
                std::cerr << "Image is empty" << std::endl;
                continue;
            }

            object_recognition_enabled = true;

            if (object_found)
            {
                sub_image = image(cv::Rect(start_point, end_point));
                cv::cvtColor(sub_image, grey, cv::COLOR_BGR2GRAY);
            }
            else
            {
                cv::cvtColor(image, grey, cv::COLOR_BGR2GRAY);
            }

            if (image.empty())
            {
                object_found = false;
                continue;
            }


            cv::adaptiveThreshold(grey, mask, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 201, 0);

            cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11, 11)), cv::Point(-1, -1));

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask.clone(), contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
            std::vector<cv::Point> largestContour;
            // std::vector<std::vector<cv::Point>> largestContour;

            // cv::drawContours(image, contours, -1, cv::Scalar(0, 255, 0), 2);

            if (!contours.empty())
            {
                if (object_found)
                {
                    largestContour = *std::max_element(contours.begin(), contours.end(),
                                                       [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
                                                       {
                                                           return cv::contourArea(a) < cv::contourArea(b);
                                                       }); // Lambda function. Finds the largest contour
                    cv::minEnclosingCircle(largestContour, enclosingCenter, radius);
                    if (!(std::abs(radius - 17) < 5))
                    {
                        object_found = false;
                        continue;
                    }
                    new_centre = enclosingCenter;
                }
                else
                {

                    for (auto iter = contours.begin(); iter != contours.end(); iter++)
                    {
                        float area = cv::contourArea(*iter);
                        if (area > 350 && area < 950)
                        {
                            largestContour = *iter;

                            if (largestContour.size() < 5)
                            {
                                continue;
                            }

                            cv::RotatedRect eclipse = cv::fitEllipse(largestContour);
                            float e_height = eclipse.size.height;
                            float e_width = eclipse.size.width;

                            if (std::abs(e_height - e_width) < 10 && std::abs(eclipse.center.y - 434) < 130 && std::abs(eclipse.center.x - 900) < 200)
                            {
                                if (object_found)
                                {
                                    new_centre = eclipse.center;
                                }
                                else
                                {
                                    measurements_point_vector.push_back(eclipse.center);
                                }
                                break;
                            }
                        }
                    }
                }
            }
            else
            {
                // std::cout << "No object found" << std::endl;
                object_found = false;
            }

            if (object_found == false && (measurements_point_vector.size() >= 2))
            {
                int confidence = 0;
                cv::Point2f average_find(0.0, 0.0);
                for (auto iter = measurements_point_vector.begin(); iter != measurements_point_vector.end() - 1; iter++)
                {
                    float diff_x = std::abs((*iter).x - (*std::next(iter, 1)).x);
                    float diff_y = std::abs((*iter).y - (*std::next(iter, 1)).y);
                    if (diff_x < 5 && diff_y < 5)
                    {
                        average_find += *iter;
                        confidence++;
                    }
                    if (confidence >= 2)
                    {
                        object_found = true;
                        std::cout << "average find x " << average_find.x / (float)confidence << " y " << average_find.y / (float)confidence << std::endl;
                        std::cout << "object found " << object_found << std::endl;
                        new_centre = average_find / (float)confidence;
                        measurements_point_vector.clear();
                        break;
                    }
                }
            }

            if (object_found)
            {
                new_centre.x = new_centre.x + (start_point.x);
                new_centre.y = new_centre.y + (start_point.y);
            }

            start_point.x = constrain(new_centre.x - image_cut_width / 2.0f, 0.0f, (float)CAMERA_WIDTH);
            start_point.y = constrain(new_centre.y - image_cut_height / 2.0f, 0.0f, (float)CAMERA_HEIGHT);
            end_point.x = constrain(new_centre.x + image_cut_width / 2.0f, 0.0f, (float)CAMERA_WIDTH);
            end_point.y = constrain(new_centre.y + image_cut_height / 2.0f, 0.0f, (float)CAMERA_HEIGHT);

            kallman_ball = kalmanKF.Predict((cv::Point2i)new_centre);

            if (kalmanKF.getStatus() == "Predicted")
            {
                object_found = false;
            }
        }
    }

    temp_thread_running = false; // For when no signal received. 

    server.stop();
    cam.stopVideo();
    temp_thread.join(); // Join at bottom, to not hault camera if that would be the case

    std::cout << "End of file!" << std::endl;
    return 0;
}