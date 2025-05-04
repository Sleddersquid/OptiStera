#include <fstream>
#include <lccv.hpp>
#include <opencv2/opencv.hpp>

#define CAMERA_HEIGHT 640    // Can be SD: 480, HD: 720, FHD: 1080, QHD: 1440
#define CAMERA_WIDTH 640    // Can be SD: 640, HD: 1280, FHD: 1920, QHD: 2560
#define CAMERA_FRAMERATE 100 // If fps higher than what the thread can handle, it will just run lower fps.

std::vector<std::string> load_class_list()
{
    // Reading a list of class names from the file which in "Models/classes.txt" and keep them in a vector
    //  S�n�f isimleri Models dosyas�ndaki text file'dan al�n�r

    std::vector<std::string> class_list;
    std::ifstream ifs("../config_files/classes.txt");
    std::string line;
    while (getline(ifs, line))
    {
        class_list.push_back(line);
    }
    return class_list;
}

void load_net(cv::dnn::Net &net, bool is_cuda)
{
    // Loading yolov5s onnx model
    //  E�itilmi� Onnx modeli cekilir2

    auto result = cv::dnn::readNet("../config_files/yolov5s.onnx");
    if (is_cuda)
    {
        std::cout << "Using CUDA\n";
        result.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        result.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
    }
    else
    {
        std::cout << "CPU Mode\n";
        result.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        result.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
    net = result;
}

// Color definitons and Constant assigments
// Renk ve Sabit atamalar�
const std::vector<cv::Scalar> colors = {cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 0)};

const float INPUT_WIDTH = 640.0;
const float INPUT_HEIGHT = 640.0;
const float SCORE_THRESHOLD = 0.2;
const float NMS_THRESHOLD = 0.04; //  This threshold used for non-maximum suppression to remove overlapping bounding boxes
const float CONFIDENCE_THRESHOLD = 0.04;

struct Detection
{
    int class_id;
    float confidence;
    cv::Rect box;
};

cv::Mat format_yolov5(const cv::Mat &source)
{
    int col = source.cols;
    int row = source.rows;
    int _max = MAX(col, row);
    cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
    source.copyTo(result(cv::Rect(0, 0, col, row)));
    return result;
}

void detect(cv::Mat &image, cv::dnn::Net &net, std::vector<Detection> &output, const std::vector<std::string> &className)
{

    /*
    Performs object detection on a video media using the YOLOv5 model
    Converts  the  input  images  to  a blob  suitable  for model input
    Processes the model outputs to extract class IDs, confidences, and bounding box coordinates
    Applies confidence and threshold filtering and performs non-maximum suppression to eliminate redundant detections


    Girdi olarak verilen video medya �zerinde YOLOv5 modelini kullanarak nesne tespiti yapan tespit fonksiyonudur
    Model girdisi i�in uygun bir blob'a (Binary Large Object"�n k�saltmas�d�r) d�n��t�r�r
    Model ��k��lar�n� i�leyerek s�n�f kimliklerini, threshold'lar� ve kapsay�c� kutu koordinatlar�n� verir
    G�ven ve threshold de�eri filtrelemesi uygular ve gereksiz tespitleri eler
    */

    cv::Mat blob;

    auto input_image = format_yolov5(image);

    cv::dnn::blobFromImage(input_image, blob, 1. / 255., cv::Size(INPUT_WIDTH, INPUT_HEIGHT), cv::Scalar(), true, false);
    net.setInput(blob);
    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    float x_factor = input_image.cols / INPUT_WIDTH;
    float y_factor = input_image.rows / INPUT_HEIGHT;

    float *data = (float *)outputs[0].data;

    // const int dimensions = 85;
    const int rows = 25200;

    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (int i = 0; i < rows; ++i)
    {

        float confidence = data[4];
        if (confidence >= CONFIDENCE_THRESHOLD)
        {

            float *classes_scores = data + 5;
            cv::Mat scores(1, className.size(), CV_32FC1, classes_scores);
            cv::Point class_id;
            double max_class_score;
            minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
            if (max_class_score > SCORE_THRESHOLD)
            {

                confidences.push_back(confidence);

                class_ids.push_back(class_id.x);

                float x = data[0];
                float y = data[1];
                float w = data[2];
                float h = data[3];
                // Kapsay�c� kutunun x, y koordinatlar�; y�kseklik ve geni�li�i
                // Width, height and x,y coordinates of bounding box

                int left = int((x - 0.5 * w) * x_factor);
                int top = int((y - 0.5 * h) * y_factor);
                int width = int(w * x_factor);
                int height = int(h * y_factor);
                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }
        data += 85;
    }

    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, nms_result);
    for (int i = 0; i < nms_result.size(); i++)
    {
        int idx = nms_result[i];
        Detection result;
        result.class_id = class_ids[idx];
        result.confidence = confidences[idx];
        result.box = boxes[idx];
        output.push_back(result);
    }
}
int main(int argc, char **argv)
{
    // Loads the list of class names using load_class_list()
    std::vector<std::string> class_list = load_class_list();

    // Opens a video file for processing.
    lccv::PiCamera cam(0);

    // Was (0, 120, 120) and (10, 255, 255).
    // Lightings conditions such as sunlight might detect hands and face
    cv::Scalar hsv_lower(0, 120, 120);  // 0, 150, 50
    cv::Scalar hsv_upper(15, 255, 255); // 15, 255, 255

    cam.options->video_width = CAMERA_WIDTH;
    cam.options->video_height = CAMERA_HEIGHT;
    cam.options->framerate = CAMERA_FRAMERATE;
    cam.options->verbose = true;
    cam.options->setExposureMode(Exposure_Modes::EXPOSURE_SHORT);
    cam.options->setWhiteBalance(WhiteBalance_Modes::WB_INDOOR);

    // Use CUDA if existing
    bool is_cuda = argc > 1 && strcmp(argv[1], "cuda") == 0;

    // Loading YOLOv5 model using load_net()
    cv::dnn::Net net;
    load_net(net, is_cuda);

    auto start = std::chrono::high_resolution_clock::now();
    int frame_count = 0;
    float fps = -1;
    int total_frames = 0;

    // Calculate the new dimensions while preserving the aspect ratio
    int targetWidth = 800; // Adjust the desired width

    cam.startVideo();

    while (true)
    {
        cv::Mat frame;
        if (!cam.getVideoFrame(frame, 1000))
        {
            std::cerr << "Timeout error" << std::endl;
        }
        else
        {

            std::vector<Detection> output;
            detect(frame, net, output, class_list);

            frame_count++;
            total_frames++;

            int detections = output.size();

            for (int i = 0; i < detections; ++i)
            {
                auto detection = output[i];
                auto box = detection.box;
                auto classId = detection.class_id;
                const auto color = colors[classId % colors.size()];
                cv::rectangle(frame, box, color, 3);

                cv::rectangle(frame, cv::Point(box.x, box.y - 20), cv::Point(box.x + box.width, box.y), color, cv::FILLED);
                cv::putText(frame, class_list[classId].c_str(), cv::Point(box.x, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
            }

            if (frame_count >= 30)
            {
                auto end = std::chrono::high_resolution_clock::now();
                fps = frame_count * 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

                frame_count = 0;
                start = std::chrono::high_resolution_clock::now();
            }

            if (fps > 0)
            {
                std::ostringstream fps_label;
                fps_label << std::fixed << std::setprecision(2);
                fps_label << "FPS: " << fps;
                std::string fps_label_str = fps_label.str();

                std::cout << fps_label_str << std::endl;
                // cv::putText(frame, fps_label_str.c_str(), cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
            }

            // // Resize the frame with it's new shape as it's original ratio
            // int targetHeight = static_cast<int>(frame.rows * static_cast<float>(targetWidth) / frame.cols);
            // cv::Size newSize(targetWidth, targetHeight);
            // cv::Mat resized_frame;
            // cv::resize(frame, resized_frame, newSize);

            // cv::imshow("output", resized_frame);

            // int key = cv::waitKey(1);
            // if (key == 113) // ESC to exit
            // {
            //     break;
            // }
        }
    }

    cam.stopVideo();

    return 0;
}