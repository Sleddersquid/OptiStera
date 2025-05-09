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

#define CAMERA_HEIGHT 864    // Can be SD: 480, HD: 720, FHD: 1080, QHD: 1440,  y
#define CAMERA_WIDTH 1804    // Can be SD: 640, HD: 1280, FHD: 1920, QHD: 2560, x
#define CAMERA_FRAMERATE 100 // If fps higher than what the thread can handle, it will just run lower fps.
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

    T getAverage()
    {
        // return std::accumulate(average_container.begin(), average_container.end(), 0.0) / (float)average_container.size();
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

// Must be global for signal handler
// -------------------------- SIGNAL HANDLER -------------------------- //

std::thread temp_thread;

std::atomic<bool> get_temp_running = true;
std::atomic<bool> camera_running = true;

void signalHandler(int signum)
{
    get_temp_running = false;
    camera_running = false;
}

int main()
{
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    // Over OPC-UA Serve and Node instansiation.
    cv::Point2f new_centre(0.0, 0.0);
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

    // Feedback of status of object recognition to the RCU
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
        } });

    // -------------------------- OpenCV -------------------------- //
    std::cout << "Press ESC to stop. (Does not work if no window is displayed)" << std::endl;

    // cv::Rect roi;

    // Was (0, 120, 120) and (10, 255, 255).
    // Lightings conditions such as sunlight might detect hands and face

    cv::Scalar hsv_upper(144, 255, 255); // 15, 255, 255
    cv::Scalar hsv_lower(35, 113, 0);    // 0, 150, 50

    // cv::Scalar hsv_upper(35, 255, 255); // 15, 255, 255
    // cv::Scalar hsv_lower(7, 110, 191);  // 0, 150, 50

    // cv::Scalar hsv_upper(35, 190, 141);  // 0, 150, 50
    // cv::Scalar hsv_lower(0, 150, 89); // 15, 255, 255

    // cv::Scalar hsv_upper_o(179, 255, 255);  // 0, 150, 50
    // cv::Scalar hsv_lower_o(177, 92, 89); // 15, 255, 255

    // cv::Scalar hsv_upper_r(6, 236, 255); // 15, 255, 255
    // cv::Scalar hsv_lower_r(0, 174, 163);  // 0, 150, 50

    lccv::PiCamera cam(0);

    cam.options->video_width = CAMERA_WIDTH;
    cam.options->video_height = CAMERA_HEIGHT;
    cam.options->framerate = CAMERA_FRAMERATE;
    cam.options->verbose = true;
    cam.options->denoise = "cdn_fast";
    cam.options->sharpness = 2.5f; // 6.5f
    // cam.options->shutter = 1 / 1000.0f;
    cam.options->setExposureMode(Exposure_Modes::EXPOSURE_SHORT);
    cam.options->setWhiteBalance(WhiteBalance_Modes::WB_INDOOR);
    // cam.options->list_cameras = true;

    cam.startVideo();

    Local_max_min<float> height_object;
    Local_max_min<float> width_object;
    Local_max_min<float> angle_object;
    Local_max_min<float> area_object;

    // cv::namedWindow("Image", cv::WINDOW_NORMAL);
    // cv::namedWindow("Mask", cv::WINDOW_NORMAL);

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
    cv::Point2f kallman_ball(0, 0);

    kalmantracking::kalman kalmanKF(iKalmanMode, iDebugMode);

    int image_count = 0;
    int frame_count = 0;
    float fps = -1;
    std::vector<float> fps_counter_vec;

    bool enable_print = false;

    cv::Mat image, sub_image;
    cv::Mat grey, blur;
    cv::Mat HSV, mask, range;

    // For cropping
    cv::Point2f start_point(0.0, 0.0), end_point(0.0, 0.0), sub_delta(0.0, 0.0);
    float image_cut_height = 100.0f;
    float image_cut_width = 90.0f;

    Local_max_min<float> difference_container;
    Local_max_min<float> distance_x;
    Local_max_min<float> distance_y;

    while (fps_counter_vec.size() < FPS_SAMPLES && camera_running)
    // while (camera_running)
    {
        opcua::services::writeDataValue(server, cameraNodeXId, opcua::DataValue(opcua::Variant(new_centre.x)));
        opcua::services::writeDataValue(server, cameraNodeYId, opcua::DataValue(opcua::Variant(new_centre.y)));
        opcua::services::writeDataValue(server, cameraNodeRadiusId, opcua::DataValue(opcua::Variant(radius)));

        opcua::services::writeDataValue(server, statusObjectRecognitionId, opcua::DataValue(opcua::Variant(object_recognition_enabled)));
        opcua::services::writeDataValue(server, objectFoundId, opcua::DataValue(opcua::Variant(object_found)));

        object_recognition_enabled = opcua::services::readDataValue(server, enableObjectRecognitionId).value().value().to<bool>();
        server_delay_ms = server.runIterate();

        // end = std::chrono::high_resolution_clock::now();
        // elapsed_time = std::chrono::duration<double, std::milli>(end - start);

        // fps_counter_vec.push_back(elapsed_time);

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
                image = image(cv::Rect(start_point, end_point));
            }

            // cv::cvtColor(blur, grey, cv::COLOR_BGR2GRAY);

            // // cv::adaptiveThreshold(grey, mask, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 201, 1); // 50, 100
            // cv::threshold(grey, mask, 20, 255, cv::THRESH_BINARY);

            // cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 3)), cv::Point(-1, -1), 2);

            // cv::dilate(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 3)), cv::Point(-1, -1), 2);

            // cv::rectangle(image, start_point, end_point, cv::Scalar(0, 255, 0), 2);
            // }
            // else
            // {
            cv::cvtColor(image, grey, cv::COLOR_BGR2GRAY);

            if(object_found && true) {
                cv::adaptiveThreshold(grey, mask, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 201, 0);
            } else {
                cv::adaptiveThreshold(grey, mask, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 201, 3);
            }
            
            cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)), cv::Point(-1, -1));

            // cv::dilate(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)), cv::Point(-1, -1));

            // cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11)), cv::Point(-1, -1));
            // cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7)), cv::Point(-1, -1));

            // cv::dilate(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)), cv::Point(-1, -1));
            // cv::dilate(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7)), cv::Point(-1, -1));
            // }

            // cv::inRange(HSV, hsv_lower_o, hsv_upper_o, mask_o);
            // cv::inRange(HSV, hsv_lower_r, hsv_upper_r, mask_r);

            // mask = mask_o | mask_r;

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask.clone(), contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
            std::vector<cv::Point> largestContour;
            // std::vector<std::vector<cv::Point>> largestContour;

            // cv::drawContours(image, contours, -1, cv::Scalar(0, 255, 0), 2);

            if (!contours.empty())
            {
                // std::cout << "Lesss goooo" << std::endl;
                for (auto iter = contours.begin(); iter != contours.end(); iter++)
                {
                    float area = cv::contourArea(*iter);

                    if (area < 200)
                    {
                        continue;
                    }
                    if (area > 900 && area < 1700)
                    {
                        largestContour = *iter;
                        cv::RotatedRect eclipse = cv::fitEllipse(largestContour);
                        float e_height = eclipse.size.height;
                        float e_width = eclipse.size.width;
                        
                        if (std::abs(e_height - e_width) < 20 && (std::abs(e_height - 43) < 20 && (std::abs(e_width - 40) < 20)) && std::abs(eclipse.angle - 65) < 15)
                        {
                            height_object.insert(e_height);
                            width_object.insert(e_width);
                            angle_object.insert(eclipse.angle);
                            area_object.insert(area);
                            std::cout << "--------------------" << enclosingCenter.x << ", " << enclosingCenter.y << std::endl;
                            std::cout << "Height " << e_height << " Width: " << e_width << " Angle: " << eclipse.angle << std::endl;
                            std::cout << "Area:" << area << std::endl;
                            enclosingCenter = eclipse.center;
                            cv::ellipse(image, eclipse, cv::Scalar(255, 0, 0), 2);
                            object_found = true;
                            break;
                        }
                    }
                }
            }
            else
            {
                // std::cout << "No object found" << std::endl;
                object_found = false;
            }

            if (frame_count >= 50 && object_found)
            {
                auto end = std::chrono::high_resolution_clock::now();
                fps = frame_count * 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

                frame_count = 0;
                fps_counter_vec.push_back(fps);

                start = std::chrono::high_resolution_clock::now();
            }

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

            // new_centre = calculateCenter(largestContour);

            // enclosingCenter.x = iFrameNum*4 % CAMERA_WIDTH;
            // enclosingCenter.y = 512;

            // enclosingCenter.x = 0;
            // enclosingCenter.y = 0;

            // enclosingCenter = new_centre;

            // std::cout << "enclosing centre.x: " << enclosingCenter.x << " enclosing centre.y: " << enclosingCenter.y << std::endl;
            // std::cout << "New center.x: " << new_centre.x << " New center.y: " << new_centre.y << std::endl;

            kallman_ball = kalmanKF.Predict((cv::Point2i)enclosingCenter);

            // Point p1 = Point(kallman_ball.x-10, kallman_ball.y-10); //Test 3.3
            // // Point p2 = Point(kallman_ball.x + 10, kallman_ball.y + 10); //Test 3.3
            // Point p1 = Point(kallman_ball.x - 10, kallman_ball.y - 10); // Test 3.3
            // Point p2 = Point(kallman_ball.x + 10, kallman_ball.y + 10); // Test 3.3

            // std::cout << "Object found: " << object_found << std::endl;

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

            // Since Kalmanfilter ball will try to predict, enclosing centre will be zero if no object, klamna ball won't be zero

            // sub_centre = new_centre;
            // If object found we need to add margin
            new_centre = enclosingCenter;

            if (object_found)
            {
                new_centre.x = new_centre.x + (start_point.x);
                new_centre.y = new_centre.y + (start_point.y);

                // difference_container.insert(std::abs(sub_delta.x - new_centre.x));
                // distance_x.insert(new_centre.x);
                // distance_y.insert(new_centre.y);

                frame_count++;
            }

            start_point.x = constrain(new_centre.x - image_cut_width / 2.0f, 0.0f, (float)CAMERA_WIDTH);
            start_point.y = constrain(new_centre.y - image_cut_height / 2.0f, 0.0f, (float)CAMERA_HEIGHT);
            end_point.x = constrain(new_centre.x + image_cut_width / 2.0f, 0.0f, (float)CAMERA_WIDTH);
            end_point.y = constrain(new_centre.y + image_cut_height / 2.0f, 0.0f, (float)CAMERA_HEIGHT);

            // cv::imshow("Post-Mask", mask);
            // cv::imshow("Video", image);

            // mask is a black and white image from the InRange function
            // It is not until the server runs iterate that the values will be updated

            // auto end = std::chrono::high_resolution_clock::now();

            // elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

            // delay_server_iterate = server.runIterate();
            // std::this_thread::sleep_for(std::chrono::milliseconds(delay_server_iterate));

            kallman_ball.x = constrain(kallman_ball.x, 0.0f, (float)CAMERA_WIDTH);
            kallman_ball.y = constrain(kallman_ball.y, 0.0f, (float)CAMERA_HEIGHT);

            // To save images.
            // if (cv::waitKey(1) == 'r') {
            //     // cv::imwrite("pre-mask.jpg", pre_mask);
            //     // cv::imwrite("post-mask.jpg", mask);
            //     cv::imwrite("image.jpg", image);
            //     std::cout << "Images saved" << std::endl;
            // }

            // std::cout << "difference: " << std::abs(sub_delta.x - new_centre.x) << std::endl;

            sub_delta = new_centre;

            // cv::circle(image, kallman_ball, 3, cv::Scalar(0, 0, 255), -1);

            iFrameNum++;

            cv::imshow("Image", image);
            cv::imshow("Mask", mask);

            // // quit on q button
            // // Remove when time for final push
            if (cv::waitKey(1) == 'q')
            {
                get_temp_running = false;
                camera_running = false;
                break;
            }
        }
    }

    std::cout << "Height diff: " << height_object.getAverage() << ", Min: " << height_object.getMin() << ", Max: " << height_object.getMax() << std::endl;
    std::cout << "Width diff: " << width_object.getAverage() << ", Min: " << width_object.getMin() << ", Max: " << width_object.getMax() << std::endl;
    std::cout << "Angle diff: " << angle_object.getAverage() << ", Min: " << angle_object.getMin() << ", Max: " << angle_object.getMax() << std::endl;
    std::cout << "Area diff: " << area_object.getAverage() << ", Min: " << area_object.getMin() << ", Max: " << area_object.getMax() << std::endl;

    std::cout << "Average FPS: " << std::accumulate(fps_counter_vec.begin(), fps_counter_vec.end(), 0.0) / fps_counter_vec.size() << std::endl;

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